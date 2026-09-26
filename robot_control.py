"""The only process that owns RTDE connections to both robots.

All calls into RTDE are made on this process's main thread. GUI commands use a
queue; an Event interrupts asynchronous moves without a second RTDE client.
"""
import math
import json
import queue
import time
from pathlib import Path

DIAG_IP = '192.168.8.3'
SURGEON_IP = '192.168.8.4'


def valid_int(value, name, maximum=10000):
    number = int(value)
    if not 1 <= number <= maximum:
        raise ValueError(f'{name}: требуется целое число от 1 до {maximum}')
    return number


def valid_float(value, name, minimum=0.0, maximum=1.0, allow_zero=False):
    number = float(value)
    if not math.isfinite(number) or number < minimum or number > maximum or (not allow_zero and number == 0):
        raise ValueError(f'{name}: требуется конечное число в пределах {minimum}..{maximum}')
    return number


def valid_pose(pose):
    if len(pose) != 6:
        raise ValueError('Для движения нужен маршрут из шести координат TCP')
    numbers = [float(v) for v in pose]
    if not all(math.isfinite(v) for v in numbers):
        raise ValueError('Координаты TCP должны быть конечными числами')
    return numbers


def tool_translation(pose, dz):
    """Translate along the tool Z axis, using the axis-angle UR TCP pose."""
    x, y, z, rx, ry, rz = valid_pose(pose)
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-10:
        direction = (0.0, 0.0, 1.0)
    else:
        ux, uy, uz = rx / angle, ry / angle, rz / angle
        s, c = math.sin(angle), math.cos(angle)
        direction = (uy * s + ux * uz * (1 - c), -ux * s + uy * uz * (1 - c), c + uz * uz * (1 - c))
    return [x + dz * direction[0], y + dz * direction[1], z + dz * direction[2], rx, ry, rz]


class Cancelled(Exception):
    pass


class RobotWorker:
    def __init__(self, commands, messages, stop_event, shutdown, heartbeat,
                 diag_ip=DIAG_IP, surgeon_ip=SURGEON_IP):
        self.commands, self.messages = commands, messages
        self.stop_event, self.shutdown, self.heartbeat = stop_event, shutdown, heartbeat
        self.ips = {'diagnost': diag_ip, 'surgeon': surgeon_ip}
        self.ctrl, self.recv = {}, {}
        self.joysticks = {}
        self.manual = False
        self.last_heartbeat = 0
        self.last_telemetry = 0
        self.last_buttons = {'diagnost': set(), 'surgeon': set()}
        self.paths = {'diagnost': [], 'surgeon': []}
        self.start_pose = None
        self.route_data = []
        self.aphi_data = []
        self.aphi_start = None
        config = Path(__file__).with_name('joystick_map.json')
        self.buttons = json.loads(config.read_text(encoding='utf-8')) if config.exists() else {}

    def emit(self, kind, **data):
        self.messages.put((kind, data))

    def beat(self):
        now = time.monotonic()
        if now - self.last_heartbeat >= 1:
            self.heartbeat.put(('robot_worker', 'ALIVE', time.time()))
            self.last_heartbeat = now

    def connect(self):
        from rtde_control import RTDEControlInterface
        from rtde_receive import RTDEReceiveInterface
        for name, ip in self.ips.items():
            self.ctrl[name] = RTDEControlInterface(ip)
            self.recv[name] = RTDEReceiveInterface(ip)
            if not self.ctrl[name].isConnected() or not self.recv[name].isConnected():
                raise ConnectionError(f'Нет RTDE подключения: {name} ({ip})')

    def init_joysticks(self):
        import pygame
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() < 2:
            raise RuntimeError('Для ручного управления нужны два джойстика (ID 0 и 1)')
        for name, index in (('surgeon', 0), ('diagnost', 1)):
            joy = pygame.joystick.Joystick(index)
            joy.init()
            self.joysticks[name] = joy
            self.emit('INFO', text=f'{name}: {joy.get_name()}, кнопок: {joy.get_numbuttons()}')

    def stop_speed(self):
        for control in self.ctrl.values():
            control.speedStop(0.5)

    def check_stop(self):
        self.beat()
        if self.stop_event.is_set() or self.shutdown.is_set():
            raise Cancelled()

    def wait(self, seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            self.check_stop()
            time.sleep(min(0.02, max(0, until - time.monotonic())))

    def move(self, name, pose, speed=0.05, acceleration=0.1):
        self.check_stop()
        pose = valid_pose(pose)
        ctrl = self.ctrl[name]
        # RTDE accepts async as the fourth positional argument (async is a keyword).
        if not ctrl.moveL(pose, speed, acceleration, True):
            raise RuntimeError(f'{name}: команда moveL отклонена')
        try:
            while ctrl.getAsyncOperationProgress() >= 0:
                self.check_stop()
                self.telemetry()
                time.sleep(0.02)
            self.check_stop()
        except BaseException:
            ctrl.stopL(0.5)
            raise

    def telemetry(self):
        if time.monotonic() - self.last_telemetry < 0.5:
            return
        sample = {}
        for name in self.ips:
            receiver = self.recv[name]
            sample[name] = {'pose': list(receiver.getActualTCPPose()),
                            'joints': list(receiver.getActualQ()),
                            'force': list(receiver.getActualTCPForce())}
        self.emit('TELEMETRY', robots=sample)
        self.last_telemetry = time.monotonic()

    def button(self, joy, index):
        return index < joy.get_numbuttons() and bool(joy.get_button(index))

    def mapped_button(self, name, joy, key, default):
        index = int(self.buttons.get(name, {}).get(key, default))
        return self.button(joy, index)

    def manual_step(self):
        import pygame
        pygame.event.pump()
        for name, joy in self.joysticks.items():
            hat = joy.get_hat(0) if joy.get_numhats() else (0, 0)
            axis = lambda i: joy.get_axis(i) if i < joy.get_numaxes() and abs(joy.get_axis(i)) > 0.25 else 0.0
            velocity = 0.015 if name == 'surgeon' else 0.01
            speeds = [hat[0] * velocity, hat[1] * velocity,
                      (self.mapped_button(name, joy, 'z_plus', 4) -
                       self.mapped_button(name, joy, 'z_minus', 2)) * velocity,
                      -axis(1) * 0.19, -axis(0) * 0.19, -axis(2) * 0.19]
            # Physical button 0 selects tool-frame speed; otherwise base frame.
            if self.mapped_button(name, joy, 'tool_frame', 0):
                self.ctrl[name].speedL(self._tool_speed_to_base(speeds, self.recv[name].getActualTCPPose()), 0.1, 0.1)
            else:
                self.ctrl[name].speedL(speeds, 0.1, 0.1)
            pressed = {i for i in range(joy.get_numbuttons()) if self.button(joy, i)}
            rising = pressed - self.last_buttons[name]
            if self.buttons.get(name, {}).get('record') in rising:
                self.paths[name].append(list(self.recv[name].getActualTCPPose()))
                self.emit('PATH', robot=name, poses=self.paths[name])
            if self.buttons.get(name, {}).get('save') in rising:
                self.emit('PATH', robot=name, poses=self.paths[name])
            if self.buttons.get(name, {}).get('replay') in rising and self.paths[name]:
                self.stop_speed()
                for pose in self.paths[name]:
                    self.move(name, pose, 0.05, 0.1)
            self.last_buttons[name] = pressed

    @staticmethod
    def _tool_speed_to_base(speed, pose):
        # Rodrigues rotation: linear and angular vectors use the same rotation.
        rx, ry, rz = pose[3:]
        theta = math.sqrt(rx * rx + ry * ry + rz * rz)
        if theta < 1e-10:
            return speed
        ux, uy, uz = rx / theta, ry / theta, rz / theta
        u = (ux, uy, uz)
        c, s = math.cos(theta), math.sin(theta)
        def rotate(v):
            cross = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
            dot = sum(a*b for a, b in zip(u, v))
            return [v[i]*c + cross[i]*s + u[i]*dot*(1-c) for i in range(3)]
        return rotate(speed[:3]) + rotate(speed[3:])

    def execute(self, action, args):
        if action == 'manual_start':
            if not self.joysticks:
                self.init_joysticks()
            self.check_stop()
            self.manual = True
            return
        if action == 'manual_stop':
            self.manual = False
            self.stop_speed()
            return
        if action == 'align':
            name = args['robot']
            pose = list(self.recv[name].getActualTCPPose())
            if name == 'diagnost':
                pose[2] += 0.05  # Original Align_D.py movement.
            else:
                pose[3:] = [math.pi/2, 0.0, 0.0]
            self.move(name, pose, 0.05, 0.1)
        elif action == 'route':
            count, steps, pause, speed = args['count'], args['steps'], args['pause'], args['speed']
            self.start_pose = list(self.recv['diagnost'].getActualTCPPose())
            self.route_data = []
            started = time.monotonic()
            try:
                for _ in range(count):
                    for axis, delta in enumerate(steps):
                        if delta:
                            pose = list(self.recv['diagnost'].getActualTCPPose())
                            pose[axis] += delta
                            self.move('diagnost', pose, speed)
                    self.route_data.append((time.monotonic()-started, list(self.recv['diagnost'].getActualTCPPose()),
                                            list(self.recv['diagnost'].getActualTCPForce())))
                    self.wait(pause)
            finally:
                self.emit('ROUTE', rows=self.route_data, start=self.start_pose)
        elif action == 'aphi':
            self.aphi_start = list(self.recv['surgeon'].getActualTCPPose())
            self.aphi_data = []
            try:
                for _ in range(args['count']):
                    pose = tool_translation(self.recv['surgeon'].getActualTCPPose(), -args['step'])
                    self.move('surgeon', pose, args['speed'])
                    self.aphi_data.append(list(self.recv['surgeon'].getActualTCPPose()))
                    self.wait(args['pause'])
            finally:
                self.emit('APHI', poses=self.aphi_data)
        elif action == 'ashido_pos':
            # No implicit rotation or movement by tool length: these require calibration.
            self.emit('INFO', text='Исходная поза хирурга сохранена; автоматическая геометрия АСХИДО требует калибровки')
            self.aphi_start = list(self.recv['surgeon'].getActualTCPPose())
        elif action == 'ashido_start':
            if self.aphi_start is None:
                raise ValueError('Сначала установите начальную точку операции')
            # Run the validated axial advancement, without driving a second robot by an unverified transform.
            self.execute('aphi', args)
        elif action == 'return_start':
            if self.start_pose is None:
                raise ValueError('Начальная точка маршрута отсутствует')
            self.move('diagnost', self.start_pose, 0.05)
        elif action == 'aphi_return':
            if self.aphi_start is None:
                raise ValueError('Начальная точка воздействия отсутствует')
            self.move('surgeon', self.aphi_start, 0.05)
        elif action == 'replay':
            name = args['robot']
            for pose in args['poses']:
                self.move(name, valid_pose(pose), 0.05)
        else:
            raise ValueError(f'Неизвестная команда: {action}')

    def run(self):
        try:
            self.connect()
            self.heartbeat.put(('robot_worker', 'READY', time.time()))
            self.emit('INFO', text='RTDE подключен к обоим роботам')
            while not self.shutdown.is_set():
                self.beat()
                try:
                    action, args = self.commands.get_nowait()
                except queue.Empty:
                    action = None
                if self.stop_event.is_set():
                    if self.manual:
                        self.manual = False
                        self.stop_speed()
                    if action and action != 'manual_stop':
                        self.emit('STOPPED', action=action)
                        action = None
                    self.stop_event.clear()
                if action:
                    try:
                        self.manual = False if action not in ('manual_start',) else self.manual
                        self.stop_speed()
                        self.execute(action, args)
                        self.emit('DONE', action=action)
                    except Cancelled:
                        self.emit('STOPPED', action=action)
                    except Exception as error:
                        self.emit('ERROR', text=f'{action}: {error}')
                elif self.manual:
                    try:
                        self.manual_step()
                    except Cancelled:
                        self.manual = False
                        self.stop_speed()
                    except Exception as error:
                        self.manual = False
                        self.emit('ERROR', text=f'Джойстик: {error}')
                        self.stop_speed()
                try:
                    self.telemetry()
                except Exception as error:
                    self.emit('ERROR', text=f'RTDE: {error}')
                    break
                time.sleep(0.02 if self.manual else 0.05)
        except Exception as error:
            self.emit('ERROR', text=f'Подключение RTDE: {error}')
        finally:
            for control in self.ctrl.values():
                try:
                    control.speedStop(0.5)
                except Exception:
                    pass
            for interface in (*self.ctrl.values(), *self.recv.values()):
                try:
                    interface.disconnect()
                except Exception:
                    pass
            if self.joysticks:
                import pygame
                pygame.quit()
            self.heartbeat.put(('robot_worker', 'FINISHED', time.time()))


def main(commands, messages, stop_event, shutdown, heartbeat, diag_ip=DIAG_IP, surgeon_ip=SURGEON_IP):
    RobotWorker(commands, messages, stop_event, shutdown, heartbeat, diag_ip, surgeon_ip).run()
