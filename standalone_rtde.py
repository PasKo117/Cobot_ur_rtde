"""Shared RTDE connection and cooperative per-robot lock for local scripts.

The lock prevents two programs from this repository from owning one robot's
RTDE control input at the same time. It cannot lock unrelated external clients.
"""

from contextlib import ExitStack, contextmanager
import os
from pathlib import Path
import socket
import tempfile
import time

from robot_control import DIAG_IP, SURGEON_IP, tool_translation, valid_pose

ROBOTS = {'diagnost': DIAG_IP, 'surgeon': SURGEON_IP}


def dashboard_command(robot, command):
    """Dashboard commands are separate from RTDE; consume greeting and reply."""
    ip = ROBOTS.get(robot, robot)
    with RobotLease(ip), socket.create_connection((ip, 29999), timeout=5) as connection:
        connection.settimeout(5)
        connection.recv(4096)
        connection.sendall((command + '\n').encode('ascii'))
        response = connection.recv(4096).decode(errors='replace').strip()
        if not response or 'failed' in response.lower() or 'not allowed' in response.lower():
            raise RuntimeError(f'{ip}: {command}: {response}')
        return response


class RobotLease:
    def __init__(self, *ips):
        self.ips = sorted(set(ips))
        self.files = []

    def __enter__(self):
        try:
            for ip in self.ips:
                path = Path(tempfile.gettempdir()) / ('cobot_rtde_' + ip.replace('.', '_') + '.lock')
                handle = open(path, 'a+b')
                try:
                    handle.seek(0)
                    if not handle.read(1):
                        handle.write(b'0')
                        handle.flush()
                    handle.seek(0)
                    if os.name == 'nt':
                        import msvcrt
                        msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
                    else:
                        import fcntl
                        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                except OSError as exc:
                    handle.close()
                    raise RuntimeError(f'{ip}: RTDE уже занят другим скриптом проекта; остановите его') from exc
                self.files.append(handle)
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, *_):
        while self.files:
            handle = self.files.pop()
            handle.seek(0)
            if os.name == 'nt':
                import msvcrt
                msvcrt.locking(handle.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl
                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
            handle.close()


class RobotSession:
    def __init__(self, name='diagnost', *, control=True):
        self.name = name
        self.ip = ROBOTS.get(name, name)
        self.want_control = control
        self.lease = RobotLease(self.ip)
        self.ctrl = self.recv = None

    def __enter__(self):
        self.lease.__enter__()
        try:
            from rtde_receive import RTDEReceiveInterface
            if self.want_control:
                from rtde_control import RTDEControlInterface
                self.ctrl = RTDEControlInterface(self.ip)
            self.recv = RTDEReceiveInterface(self.ip)
            if (self.ctrl is not None and not self.ctrl.isConnected()) or not self.recv.isConnected():
                raise ConnectionError(f'RTDE не подключён: {self.name} ({self.ip})')
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, *_):
        if self.ctrl is not None:
            try:
                self.ctrl.speedStop(0.5)
            except Exception:
                pass
        for interface in (self.recv, self.ctrl):
            if interface is not None:
                try:
                    interface.disconnect()
                except Exception:
                    pass
        self.recv = self.ctrl = None
        self.lease.__exit__(None, None, None)

    def pose(self):
        return list(self.recv.getActualTCPPose())

    def joints(self):
        return list(self.recv.getActualQ())

    def force(self):
        return list(self.recv.getActualTCPForce())

    def move(self, pose, speed=0.05, acceleration=0.1):
        if self.ctrl is None:
            raise RuntimeError('Для движения нужен управляющий RTDE сеанс')
        if not 0 < speed <= 0.1 or not 0 < acceleration <= 0.5:
            raise ValueError('Для пробы: скорость 0..0.1 м/с, ускорение 0..0.5 м/с²')
        target = valid_pose(pose)
        if not self.ctrl.moveL(target, speed, acceleration, True):
            raise RuntimeError('Контроллер отклонил moveL')
        try:
            while self.ctrl.getAsyncOperationProgress() >= 0:
                time.sleep(0.02)
        except BaseException:
            self.ctrl.stopL(0.5)
            raise

    def translate(self, dz, speed=0.01):
        if abs(dz) > 0.05:
            raise ValueError('Пробный шаг ограничен 50 мм')
        self.move(tool_translation(self.pose(), dz), speed)

    def speed(self, vector, acceleration=0.1, duration=0.1):
        speed = valid_pose(vector)
        if sum(v*v for v in speed[:3]) > 0.1**2 or sum(v*v for v in speed[3:]) > 0.5**2:
            raise ValueError('Превышен предел пробной скорости')
        self.ctrl.speedL(speed, acceleration, duration)

    @contextmanager
    def freedrive(self):
        self.ctrl.freedriveMode()
        try:
            yield
        finally:
            self.ctrl.endFreedriveMode()


@contextmanager
def sessions(*names, control=True):
    """Open several robot sessions with their locks acquired in sorted IP order."""
    leases = RobotLease(*(ROBOTS.get(name, name) for name in names))
    with leases:
        with ExitStack() as stack:
            robots = {}
            for name in names:
                robot = RobotSession(name, control=control)
                # The outer lease owns the locks; do not acquire a second time.
                robot.lease = _BorrowedLease()
                robots[name] = stack.enter_context(robot)
            yield robots


class _BorrowedLease:
    def __enter__(self): return self
    def __exit__(self, *_): pass
