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
DEFAULT_BUTTONS = {
    'surgeon': {'tool_frame': 0, 'z_plus': 4, 'z_minus': 2,
                'sync_on': 6, 'sync_off': 7, 'level_diagnost': 8,
                'save': 9, 'record': 10, 'replay': 11},
    'diagnost': {'tool_frame': 0, 'z_plus': 4, 'z_minus': 2,
                 'save': 9, 'record': 10, 'replay': 11},
}


def speed_fraction(percent):
    """Scale commanded speeds; 100% keeps the existing application limits."""
    if not isinstance(percent, int) or not 0 <= percent <= 100:
        raise ValueError('Скорость должна быть целым числом от 0 до 100%')
    return percent / 100.0


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
                 diag_ip=DIAG_IP, surgeon_ip=SURGEON_IP, speed_percent=None):
        self.commands, self.messages = commands, messages
        self.stop_event, self.shutdown, self.heartbeat = stop_event, shutdown, heartbeat
        self.ips = {'diagnost': diag_ip, 'surgeon': surgeon_ip}
        self.speed_percent = speed_percent
        self.last_manual_speed = None
        self.ctrl, self.recv = {}, {}
        self.joysticks = {}
        self.manual = False
        self.mode = 'async'
        self.calibration = None
        self.last_heartbeat = 0
        self.last_telemetry = 0
        self.last_buttons = {'diagnost': set(), 'surgeon': set()}
        self.paths = {'diagnost': [], 'surgeon': []}
        self.start_pose = None
        self.route_data = []
        self.aphi_data = []
        self.aphi_start = None
        config = Path(__file__).with_name('joystick_map.json')
        self.buttons = {name: mapping.copy() for name, mapping in DEFAULT_BUTTONS.items()}
        self.default_buttons = not config.exists()
        if config.exists():
            custom = json.loads(config.read_text(encoding='utf-8'))
            for name, mapping in custom.items():
                if name not in self.buttons or not isinstance(mapping, dict):
                    raise ValueError(f'Неизвестный джойстик в joystick_map.json: {name}')
                self.buttons[name].update(mapping)

    def load_calibration(self):
        """CalibrationManager saves R from diagnost base to surgeon base."""
        filename = Path(__file__).with_name('calibration_data.json')
        data = json.loads(filename.read_text(encoding='utf-8'))
        rotation = data['R']
        if len(rotation) != 3 or any(len(row) != 3 for row in rotation):
            raise ValueError('матрица R должна быть 3x3')
        rotation = [[float(v) for v in row] for row in rotation]
        if not all(math.isfinite(v) for row in rotation for v in row):
            raise ValueError('матрица R содержит неконечные значения')
        for i in range(3):
            for j in range(3):
                dot = sum(rotation[i][k] * rotation[j][k] for k in range(3))
                if abs(dot - (1 if i == j else 0)) > 0.01:
                    raise ValueError('матрица R не ортонормальна')
        determinant = (rotation[0][0]*(rotation[1][1]*rotation[2][2]-rotation[1][2]*rotation[2][1])
                       - rotation[0][1]*(rotation[1][0]*rotation[2][2]-rotation[1][2]*rotation[2][0])
                       + rotation[0][2]*(rotation[1][0]*rotation[2][1]-rotation[1][1]*rotation[2][0]))
        if abs(determinant - 1) > 0.01:
            raise ValueError('матрица R имеет неверный определитель')
        return rotation

    def diagnost_planar_speed(self, surgeon_speed):
        """R maps diagnost -> surgeon; transpose maps velocity back to diagnost."""
        linear = surgeon_speed[:3]
        result = [sum(self.calibration[i][axis] * linear[i] for i in range(3))
                  for axis in range(2)]
        return result + [0.0, 0.0, 0.0, 0.0]

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
            for action, button in self.buttons.get(name, {}).items():
                if button is None and action not in ('tool_frame', 'z_plus', 'z_minus'):
                    continue
                if not isinstance(button, int) or not 0 <= button < joy.get_numbuttons():
                    raise ValueError(f'{name}: индекс {action}={button} отсутствует на джойстике')
            self.emit('INFO', text=f'{name}: {joy.get_name()}, кнопок: {joy.get_numbuttons()}')
        if self.default_buttons:
            self.emit('INFO', text='joystick_map.json отсутствует; используются индексы из README')

    def stop_speed(self):
        for control in self.ctrl.values():
            control.speedStop(0.5)

    def check_stop(self):
        self.beat()
        if self.stop_event.is_set() or self.shutdown.is_set() or self.current_speed == 0:
            raise Cancelled()

    @property
    def current_speed(self):
        return self.speed_percent.value if self.speed_percent is not None else 100

    def wait(self, seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            self.check_stop()
            time.sleep(min(0.02, max(0, until - time.monotonic())))

    def move(self, name, pose, speed=0.05, acceleration=0.1, cancel_check=None):
        self.check_stop()
        pose = valid_pose(pose)
        ctrl = self.ctrl[name]
        while True:
            percent = self.current_speed
            self.check_stop()
            # RTDE accepts async as the fourth positional argument (async is a keyword).
            if not ctrl.moveL(pose, speed * speed_fraction(percent), acceleration, True):
                raise RuntimeError(f'{name}: команда moveL отклонена')
            try:
                while ctrl.getAsyncOperationProgress() >= 0:
                    self.check_stop()
                    if cancel_check is not None and cancel_check():
                        raise Cancelled()
                    if self.current_speed != percent:
                        # Restart the same target from the current pose at the new speed.
                        ctrl.stopL(0.5)
                        break
                    self.telemetry()
                    time.sleep(0.02)
                else:
                    self.check_stop()
                    return
            except BaseException:
                ctrl.stopL(0.5)
                raise

    def telemetry(self):
        if time.monotonic() - self.last_telemetry < 0.5:
            return
        sample = {}
        for name in self.ips:
            receiver = self.recv[name]
            try:
                sample[name] = {'pose': list(receiver.getActualTCPPose()),
                                'joints': list(receiver.getActualQ()),
                                'force': list(receiver.getActualTCPForce())}
            except Exception as error:
                raise ConnectionError(f'{name} ({self.ips[name]}): канал RTDEReceive: {error}') from error
        self.emit('TELEMETRY', robots=sample)
        self.last_telemetry = time.monotonic()

    def button(self, joy, index):
        return isinstance(index, int) and 0 <= index < joy.get_numbuttons() and bool(joy.get_button(index))

    def mapped_button(self, name, joy, key, default):
        index = int(self.buttons.get(name, {}).get(key, default))
        return self.button(joy, index)

    def manual_step(self):
        import pygame
        pygame.event.pump()
        percent = self.current_speed
        factor = speed_fraction(percent)
        if percent == 0 and self.last_manual_speed != 0:
            self.stop_speed()
        self.last_manual_speed = percent
        surgeon = self.joysticks['surgeon']
        surgeon_pressed = {i for i in range(surgeon.get_numbuttons()) if self.button(surgeon, i)}
        surgeon_rising = surgeon_pressed - self.last_buttons['surgeon']
        surgeon_map = self.buttons.get('surgeon', {})
        if surgeon_map.get('sync_off') in surgeon_rising and self.mode != 'async':
            self.ctrl['diagnost'].speedStop(0.5)
            self.mode = 'async'
            self.emit('MODE', mode='async')
        elif surgeon_map.get('sync_on') in surgeon_rising and self.mode != 'sync':
            try:
                self.calibration = self.load_calibration()
            except (OSError, KeyError, TypeError, ValueError) as error:
                self.emit('INFO', text=f'Синхронный режим недоступен: {error}')
            else:
                self.ctrl['diagnost'].speedStop(0.5)
                self.mode = 'sync'
                self.emit('MODE', mode='sync')
        for name, joy in self.joysticks.items():
            pressed = {i for i in range(joy.get_numbuttons()) if self.button(joy, i)}
            rising = pressed - self.last_buttons[name]
            if name == 'diagnost' and self.mode == 'sync':
                self.last_buttons[name] = pressed
                continue
            hat = joy.get_hat(0) if joy.get_numhats() else (0, 0)
            axis = lambda i: joy.get_axis(i) if i < joy.get_numaxes() and abs(joy.get_axis(i)) > 0.25 else 0.0
            velocity = 0.015 if name == 'surgeon' else 0.01
            speeds = [hat[0] * velocity * factor, hat[1] * velocity * factor,
                      (self.mapped_button(name, joy, 'z_plus', 4) -
                       self.mapped_button(name, joy, 'z_minus', 2)) * velocity * factor,
                      -axis(1) * 0.19 * factor, -axis(0) * 0.19 * factor, -axis(2) * 0.19 * factor]
            # Physical button 0 selects tool-frame speed; otherwise base frame.
            base_speeds = (self._tool_speed_to_base(speeds, self.recv[name].getActualTCPPose())
                           if self.mapped_button(name, joy, 'tool_frame', 0) else speeds)
            if factor:
                self.ctrl[name].speedL(base_speeds, 0.1, 0.1)
            if factor and name == 'surgeon' and self.mode == 'sync':
                self.ctrl['diagnost'].speedL(self.diagnost_planar_speed(base_speeds), 0.1, 0.1)
            buttons = self.buttons.get(name, {})
            if name == 'surgeon' and buttons.get('level_diagnost') in rising and self.mode == 'async':
                self.stop_speed()
                pose = list(self.recv['diagnost'].getActualTCPPose())
                pose[3:] = [0.0, math.pi, 0.0]
                self.move('diagnost', pose, 0.05, 0.1)
                self.emit('INFO', text='Ориентация диагноста выровнена')
            elif name == 'surgeon' and buttons.get('level_diagnost') in rising:
                self.emit('INFO', text='Для выравнивания диагноста сначала включите асинхронный режим (физическая кнопка 8)')
            if self.buttons.get(name, {}).get('record') in rising:
                self.paths[name].append(list(self.recv[name].getActualTCPPose()))
                self.emit('PATH', robot=name, poses=[p[:] for p in self.paths[name]])
            if self.buttons.get(name, {}).get('save') in rising:
                self.last_buttons[name] = pressed
                if self.paths[name]:
                    self.manual = False
                    self.stop_speed()
                    self.mode = 'async'
                    self.calibration = None
                    self.emit('MODE', mode=self.mode)
                    self.emit('MANUAL_PAUSED')
                    self.emit('SAVE_REQUEST', robot=name, poses=[p[:] for p in self.paths[name]])
                    return
                self.emit('INFO', text=f'{name}: нет записанных точек')
            if self.buttons.get(name, {}).get('replay') in rising and self.paths[name]:
                self.stop_speed()
                replay_button = buttons['replay']
                released = False

                def replay_cancelled():
                    nonlocal released
                    pygame.event.pump()
                    held = self.button(joy, replay_button)
                    if not held:
                        released = True
                    return released and held

                for pose in self.paths[name]:
                    self.move(name, pose, 0.05, 0.1, cancel_check=replay_cancelled)
                self.emit('INFO', text=f'{name}: воспроизведено точек: {len(self.paths[name])}')
            elif self.buttons.get(name, {}).get('replay') in rising:
                self.emit('INFO', text=f'{name}: нет записанных точек для воспроизведения')
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
            self.mode = 'async'
            self.calibration = None
            self.manual = True
            self.emit('MODE', mode=self.mode)
            return
        if action == 'manual_stop':
            self.manual = False
            self.stop_speed()
            self.mode = 'async'
            self.calibration = None
            self.emit('MODE', mode=self.mode)
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
                        self.mode = 'async'
                        self.emit('MANUAL_PAUSED')
                    if action and action != 'manual_stop':
                        self.emit('STOPPED', action=action)
                        action = None
                    self.stop_event.clear()
                if action:
                    try:
                        was_manual = self.manual
                        self.manual = False if action not in ('manual_start',) else self.manual
                        if was_manual:
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
                        self.mode = 'async'
                        self.emit('MANUAL_PAUSED')
                    except Exception as error:
                        self.manual = False
                        self.emit('ERROR', text=f'Джойстик: {error}')
                        self.stop_speed()
                        self.mode = 'async'
                        self.emit('MANUAL_PAUSED')
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


def main(commands, messages, stop_event, shutdown, heartbeat, diag_ip=DIAG_IP,
         surgeon_ip=SURGEON_IP, speed_percent=None):
    RobotWorker(commands, messages, stop_event, shutdown, heartbeat,
                diag_ip, surgeon_ip, speed_percent).run()
