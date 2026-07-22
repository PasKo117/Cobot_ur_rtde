import time
import pygame
import multiprocessing as mp
from copy import deepcopy
import logging
from logging.handlers import RotatingFileHandler
from datetime import datetime
import os
import math
from ur_rtde import RTDEControlInterface, RTDEReceiveInterface


class Robot:
    def __init__(self, IP, is_master, logger, auto, lock, shared_path, heartbeat, slave_IP=None):
        self.IP = IP
        self.is_master = is_master
        self.logger = logger
        self.auto = auto
        self.lock = lock
        self.shared_path = shared_path
        self.heartbeat = heartbeat
        self.slave_IP = slave_IP

        self.path = []
        self.last_heartbeat = time.time()

        self.ctrl = None
        self.recv = None
        self.slave_ctrl = None
        self.slave_recv = None
        self.joystick = None
        self.deadzone = 0.25  # УВЕЛИЧЕНО для надёжного отсечения дрифта

    def init_robot(self):
        self.logger.debug(f"Инициализация робота IP: {self.IP} (is_master={self.is_master})...")
        try:
            if self.is_master:
                self.logger.debug(f"Создание RTDEControlInterface для мастера {self.IP}...")
                self.ctrl = RTDEControlInterface(self.IP)
                if not self.ctrl.isConnected():
                    raise ConnectionError(f"Не удалось подключиться к RTDE контроллеру мастера {self.IP}")
                self.recv = RTDEReceiveInterface(self.IP)
                self.logger.info(f"Успешное подключение к мастеру IP: {self.IP}")

                if self.slave_IP:
                    self.logger.debug(f"Создание RTDEControlInterface для слейва {self.slave_IP}...")
                    self.slave_ctrl = RTDEControlInterface(self.slave_IP)
                    if not self.slave_ctrl.isConnected():
                        raise ConnectionError(f"Не удалось подключиться к RTDE контроллеру слейва {self.slave_IP}")
                    self.slave_recv = RTDEReceiveInterface(self.slave_IP)
                    self.logger.info(f"Успешное подключение к слейву IP: {self.slave_IP}")
            else:
                self.logger.debug(
                    f"Создание ТОЛЬКО RTDEReceiveInterface для слейв-процесса {self.IP} (чтение данных)...")
                self.recv = RTDEReceiveInterface(self.IP)
                self.logger.info(f"Успешное подключение к слейв-процессу (только чтение) IP: {self.IP}")

        except Exception as e:
            self.logger.error(f"Ошибка при подключении к {self.IP}: {e}")
            self.heartbeat.put((mp.current_process().name, "ERROR", str(e), time.time()))
            exit(1)

    def init_joystick(self):
        pygame.init()
        pygame.event.pump()
        joystick_count = pygame.joystick.get_count()
        self.logger.info(f"=== Pygame обнаружил джойстиков: {joystick_count} ===")

        for i in range(joystick_count):
            j = pygame.joystick.Joystick(i)
            j.init()
            self.logger.info(
                f"  -> Джойстик {i}: {j.get_name()} (Осей: {j.get_numaxes()}, Кнопок: {j.get_numbuttons()})")

        joystick_id = 0 if self.is_master else 1

        if joystick_id < joystick_count:
            self.joystick = pygame.joystick.Joystick(joystick_id)
            self.joystick.init()
            self.logger.info(f"✅ Успешно инициализирован джойстик ID={joystick_id}")
        else:
            self.logger.error(f"❌ ОШИБКА: Джойстик с ID={joystick_id} не найден!")

    def start_robot(self):
        self.init_joystick()
        self.init_robot()
        self.main_loop()

    def get_speeds(self):
        pygame.event.pump()

        linear_vel = 0.05
        rot_vel = 0.07
        dz = self.deadzone

        def apply_dz(val):
            return val if abs(val) > dz else 0.0

        # Читаем ВСЕ доступные оси, чтобы найти рабочую
        axes = [apply_dz(float(self.joystick.get_axis(i))) for i in range(self.joystick.get_numaxes())]

        hat = self.joystick.get_hat(0)
        hat_x = float(hat[1]) if hat is not None else 0.0
        hat_y = float(hat[0]) if hat is not None else 0.0

        btn5 = self.joystick.get_button(4)
        btn3 = self.joystick.get_button(2)
        use_tool_frame = (self.joystick.get_button(0) == 1)

        speeds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # 1. Линейные X, Y от крестовины
        speeds[0] = hat_x * -linear_vel
        speeds[1] = hat_y * -linear_vel

        # 2. Линейная Z от кнопок 2 и 3
        if btn5 == 1 and btn3 == 0:
            speeds[2] = linear_vel
        elif btn3 == 1 and btn5 == 0:
            speeds[2] = -linear_vel
        else:
            speeds[2] = 0.0

        # 3. Угловые скорости (ВНИМАНИЕ: Настройте индексы осей под ваш джойстик!)
        # По логам, ось 3 у вас сломана/залипла на -1.0. Мы временно используем ось 2 для Rz.
        # Если ось 2 тоже не та, посмотрите в консоль ниже и поменяйте цифры.
        speeds[3] = axes[1] * - rot_vel  # Rx (обычно левый стик Y)
        speeds[4] = axes[0] * rot_vel  # Ry (обычно левый стик X)
        speeds[5] = axes[2] * - rot_vel  # Rz (ВРЕМЕННО ось 2, так как ось 3 залипла)

        # === ПОЛНАЯ ДИАГНОСТИКА ОСЕЙ ===
        if not hasattr(self, 'last_telemetry') or time.time() - self.last_telemetry > 1.0:
            print("\n" + "=" * 90)
            print(f"ДЖОЙСТИК | Режим: {'ИНСТРУМЕНТ' if use_tool_frame else 'БАЗА'}")
            print(
                f"ВСЕ ОСИ (сырые): 0:{axes[0]:5.2f} | 1:{axes[1]:5.2f} | 2:{axes[2]:5.2f} | 3:{axes[3] if len(axes) > 3 else 0.0:5.2f} | 4:{axes[4] if len(axes) > 4 else 0.0:5.2f} | 5:{axes[5] if len(axes) > 5 else 0.0:5.2f}")
            print(
                f"Крестовина: X={hat_x:2.0f} | Y={hat_y:2.0f} | Кнопки: 2={btn5} | 3={btn3} | Курок(0)={self.joystick.get_button(0)}")
            print(f"ИТОГОВЫЕ СКОРОСТИ [X, Y, Z, Rx, Ry, Rz]:")
            print(
                f"   [{speeds[0]:6.3f}, {speeds[1]:6.3f}, {speeds[2]:6.3f}, {speeds[3]:6.3f}, {speeds[4]:6.3f}, {speeds[5]:6.3f}]")
            print("=" * 90 + "\n")
            self.last_telemetry = time.time()

        return speeds, use_tool_frame

    def update_heartbeat(self):
        current_time = time.time()
        if current_time - self.last_heartbeat > 2.0:
            self.heartbeat.put((mp.current_process().name, "ALIVE", time.time()))
            self.last_heartbeat = current_time

    def _get_rotation_matrix(self, pose):
        rx, ry, rz = pose[3], pose[4], pose[5]
        theta = math.sqrt(rx ** 2 + ry ** 2 + rz ** 2)
        if theta < 1e-6:
            return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        ux, uy, uz = rx / theta, ry / theta, rz / theta
        c, s, t = math.cos(theta), math.sin(theta), 1.0 - math.cos(theta)
        return [
            [t * ux * ux + c, t * ux * uy - s * uz, t * ux * uz + s * uy],
            [t * ux * uy + s * uz, t * uy * uy + c, t * uy * uz - s * ux],
            [t * ux * uz - s * uy, t * uy * uz + s * ux, t * uz * uz + c]
        ]

    def transform_tool_to_base_velocity(self, v_tool, current_pose):
        R = self._get_rotation_matrix(current_pose)
        v_lin_tool, v_ang_tool = v_tool[:3], v_tool[3:]
        v_lin_base = [sum(R[i][j] * v_lin_tool[j] for j in range(3)) for i in range(3)]
        v_ang_base = [sum(R[i][j] * v_ang_tool[j] for j in range(3)) for i in range(3)]
        return v_lin_base + v_ang_base

    def get_slave_speeds(self):
        if not self.is_master or not self.slave_recv:
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        T = self._get_rotation_matrix(self.recv.getActualTCPPose())
        xx, xy = T[0][0], T[0][1]
        yx, yy = T[0][1], -T[1][1]
        z1, z2 = -T[0][2], -T[1][2]
        s0, s1, s2 = float(self.speeds_raw[0]), float(self.speeds_raw[1]), float(self.speeds_raw[2])
        return [float(s2 * z1 + s0 * xx + s1 * yx), float(s2 * z2 + s0 * xy + s1 * yy), 0.0, 0.0, 0.0, 0.0]

    def main_loop(self):
        self.running = True
        stop = False
        path = []
        pygame.event.pump()

        while self.running:
            self.update_heartbeat()

            if not self.is_master:
                time.sleep(0.1)
                continue

            if self.lock.value:
                if not stop:
                    self.ctrl.stopScript()
                    if self.slave_ctrl: self.slave_ctrl.stopScript()
                    stop = True
                time.sleep(0.1)
                continue
            stop = False

            pygame.event.pump()
            current_pose = self.recv.getActualTCPPose()
            if self.slave_recv:
                self.slave_pos = self.slave_recv.getActualTCPPose()

            self.speeds_raw, use_tool_frame = self.get_speeds()

            if use_tool_frame:
                final_speeds = self.transform_tool_to_base_velocity(self.speeds_raw, current_pose)
            else:
                final_speeds = self.speeds_raw

            final_speeds = [float(x) for x in final_speeds]

            try:
                self.ctrl.speedL(final_speeds, acceleration=0.1, time=0.0)
                if self.slave_ctrl:
                    if self.auto.value:
                        slave_speeds = self.get_slave_speeds()
                        self.slave_ctrl.speedL([float(x) for x in slave_speeds], acceleration=0.1, time=0.0)
                    else:
                        self.slave_ctrl.speedL([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], acceleration=0.1, time=0.0)
            except Exception as e:
                error_msg = f"{type(e).__name__}:{str(e)}"
                self.logger.error(f"ОШИБКА (ВОЗМОЖНО, АВАРИЯ НА ПУЛЬТЕ!): {error_msg}")
                self.logger.error(">>> СБРОСЬТЕ АВАРИЮ (Protective Stop) НА ФИЗИЧЕСКОМ ПУЛЬТЕ РОБОТА! <<<")
                self.heartbeat.put((mp.current_process().name, "ERROR", error_msg, time.time()))
                # Не делаем break, чтобы процесс не падал, но и не спамим командами
                time.sleep(1.0)
                continue

            # Управление режимами
            if self.joystick.get_button(6) and not self.joystick.get_button(7) and self.auto.value == 0:
                self.auto.value = 1
                self.logger.info("Система переведена в СИНХРОННЫЙ режим")
            if self.joystick.get_button(7) and not self.joystick.get_button(6) and self.auto.value == 1:
                self.auto.value = 0
                self.logger.info("Система переведена в АСИНХРОННЫЙ режим")

            if self.joystick.get_button(8) and self.auto.value == 0 and self.slave_ctrl:
                self.logger.info("Возврат слейва в исходную позицию")
                self.slave_ctrl.moveL(
                    (float(self.slave_pos[0]), float(self.slave_pos[1]), float(self.slave_pos[2]), 0.0, 3.14, 0.0),
                    speed=0.2, acceleration=0.2)

            if self.joystick.get_button(10):
                path.append(list(current_pose))

            if self.joystick.get_button(11) and len(path) > 0:
                self.logger.info(f"Воспроизведение пути из {len(path)} точек")
                for pose in path:
                    self.ctrl.moveL(pose, speed=0.2, acceleration=0.2)
                    while self.ctrl.isProgramRunning():
                        pygame.event.pump()
                        if self.joystick.get_button(11):
                            self.ctrl.stopScript()
                            break
                        time.sleep(0.01)
                path = []

            if self.joystick.get_button(9) and len(path) > 0:
                self.shared_path[0] = deepcopy(path)
                self.logger.info(f"Путь сохранен в общую память. Точек: {len(path)}")
                path = []
                time.sleep(1.0)

            time.sleep(0.01)

        self.close()

    def close(self):
        self.logger.info(f"Завершение работы и отключение от робота {self.IP}")
        if self.joystick: self.joystick.quit()
        if self.ctrl:
            try:
                self.ctrl.disconnect()
            except:
                pass
        if self.recv:
            try:
                self.recv.disconnect()
            except:
                pass
        if self.is_master:
            if self.slave_ctrl:
                try:
                    self.slave_ctrl.disconnect()
                except:
                    pass
            if self.slave_recv:
                try:
                    self.slave_recv.disconnect()
                except:
                    pass


def main(IP, is_master, auto, lock, shared_path, heartbeat, slave_IP=None):
    proc_logger = logging.getLogger(f"RobotControl.{mp.current_process().name}")
    proc_logger.setLevel(logging.DEBUG)
    if not proc_logger.handlers:
        formatter = logging.Formatter(
            fmt='%(asctime)s.%(msecs)03d | %(levelname)-8s | %(processName)-15s | %(threadName)-15s | %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S')
        file_handler = RotatingFileHandler(
            filename=f'logs/processes_status_log_{datetime.now().strftime("%Y-%m-%d")}.log', maxBytes=10 * 1024 * 1024,
            backupCount=5, encoding='utf-8')
        file_handler.setFormatter(formatter)
        proc_logger.addHandler(file_handler)

    proc_logger.info(f"Запуск рабочего процесса | PID: {os.getpid()}")
    robot = Robot(IP, is_master, proc_logger, auto, lock, shared_path, heartbeat, slave_IP)
    robot.start_robot()