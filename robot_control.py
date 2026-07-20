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

        # Интерфейсы
        self.ctrl = None
        self.recv = None
        self.slave_ctrl = None
        self.slave_recv = None
        self.joystick = None
        self.deadzone = 0.2

    def init_robot(self):
        self.logger.debug(f"Инициализация робота IP: {self.IP} (is_master={self.is_master})...")
        try:
            if self.is_master:
                # МАСТЕР: Создает интерфейсы управления и чтения для себя и для слейва
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
                # СЛЕЙВ-ПРОЦЕСС: Создает ТОЛЬКО ReceiveInterface, чтобы не занимать регистры управления!
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
            self.logger.info(
                f"✅ Успешно инициализирован джойстик ID={joystick_id} для процесса {'МАСТЕР' if self.is_master else 'СЛЕЙВ'}")
        else:
            self.logger.error(f"❌ ОШИБКА: Джойстик с ID={joystick_id} не найден! Доступно только {joystick_count} шт.")

        self.deadzone = 0.15

    def start_robot(self):
        self.init_joystick()
        self.init_robot()
        self.main_loop()

    def get_speeds(self):
        linear_vel = 0.05  # 5 см/сек
        rot_vel = 0.2     # рад/сек
        speeds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        dz = getattr(self, 'deadzone', 0.3) # Мёртвая зона

        # === ЧТЕНИЕ ОСЕЙ ПО ВАШЕЙ СХЕМЕ ===
        axis0 = float(self.joystick.get_axis(0))
        axis1 = float(self.joystick.get_axis(1))
        axis2 = float(self.joystick.get_axis(2))
        axis3 = float(self.joystick.get_axis(3))
        but4 = float(self.joystick.get_button(3))
        but6 = float(self.joystick.get_button(5))
        hat = self.joystick.get_hat(0)

        # === DEADMAN SWITCH (Курок - кнопка 0) ===
        trigger_pressed = (self.joystick.get_button(0) == 1)

        if trigger_pressed:

            # 1. Крестовина -> XY
            if hat is not None:
                speeds[0] = float(hat[1]) * linear_vel * (-1)  # Вверх/вниз крестовины -> X
                speeds[1] = float(hat[0]) * linear_vel  # Влево/вправо крестовины -> Y

            # Вспомогательная функция для применения мёртвой зоны вокруг нуля
            def apply_dz(val):
                return val if abs(val) > dz else 0.0

            # 2. повороты
            speeds[3] = apply_dz(axis0) * rot_vel   #X
            speeds[4] = apply_dz(axis1) * rot_vel   #Y
            speeds[5] = apply_dz(axis2) * rot_vel * (-1)  #Z

            # 4. Z (Ось 3 - Вверх/Вниз)
            if  but4 == 1:
                speeds[2] = -linear_vel
            elif but6 == 1:
                speeds[2] = linear_vel
            else:
                speeds[2] = 0.0



        # === ДЕТАЛЬНАЯ ТЕЛЕМЕТРИЯ ===
        if not hasattr(self, 'last_telemetry'):
            self.last_telemetry = 0

        if time.time() - self.last_telemetry > 1.0:
            joy_id = self.joystick.get_id()
            print("\n" + "=" * 70)
            print(f"ДЖОЙСТИК ID: {joy_id} | Процесс: {'МАСТЕР' if self.is_master else 'СЛЕЙВ'}")
            print(f"Сырые оси:   X(0)={axis0:6.3f} | Y(1)={axis1:6.3f} | Rz(2)={axis2:6.3f} | Z(3)={axis3:6.3f}")
            print(f"Курок (Кн.0): {'ЗАЖАТ' if trigger_pressed else 'ОТПУЩЕН'}")
            print(f"Скорости [X, Y, Z, Rx, Ry, Rz]:")
            print(f"   [{speeds[0]:6.3f}, {speeds[1]:6.3f}, {speeds[2]:6.3f}, {speeds[3]:6.3f}, {speeds[4]:6.3f}, {speeds[5]:6.3f}]")
            print("=" * 70 + "\n")
            self.last_telemetry = time.time()

        return [float(x) for x in speeds]

    def update_heartbeat(self):
        current_time = time.time()
        if current_time - self.last_heartbeat > 2.0:
            self.heartbeat.put((mp.current_process().name, "ALIVE", time.time()))
            self.last_heartbeat = current_time

    def _pose_to_matrix(self, pose):
        x, y, z, rx, ry, rz = pose
        theta = math.sqrt(rx ** 2 + ry ** 2 + rz ** 2)
        if theta < 1e-6:
            return [[1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, y], [0.0, 0.0, 1.0, z], [0.0, 0.0, 0.0, 1.0]]
        ux, uy, uz = rx / theta, ry / theta, rz / theta
        c, s, t = math.cos(theta), math.sin(theta), 1.0 - math.cos(theta)
        return [
            [t * ux * ux + c, t * ux * uy - s * uz, t * ux * uz + s * uy, x],
            [t * ux * uy + s * uz, t * uy * uy + c, t * uy * uz - s * ux, y],
            [t * ux * uz - s * uy, t * uy * uz + s * ux, t * uz * uz + c, z],
            [0.0, 0.0, 0.0, 1.0]
        ]

    def get_slave_speeds(self):
        if not self.is_master or not self.slave_recv:
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        T = self._pose_to_matrix(self.recv.getActualTCPPose())
        z1, z2 = float(-T[0][2]), float(-T[1][2])
        yx, yy = float(T[0][1]), float(-T[1][1])
        xx, xy = float(T[0][0]), float(-T[1][0])

        s0, s1, s2 = float(self.speeds[0]), float(self.speeds[1]), float(self.speeds[2])

        return [
            float(s2 * z1 + s0 * xx + s1 * yx),
            float(s2 * z2 + s0 * xy + s1 * yy),
            0.0, 0.0, 0.0, 0.0
        ]

    def main_loop(self):
        self.running = True
        stop = False
        path = []
        pygame.event.pump()

        while self.running:
            self.update_heartbeat()

            # Если это не мастер, он просто ждет и шлет heartbeat (данные читаются системой мониторинга из app.py)
            if not self.is_master:
                time.sleep(0.1)
                continue

            # Логика мастера
            if self.lock.value:
                if not stop:
                    self.ctrl.stopScript()
                    if self.slave_ctrl:
                        self.slave_ctrl.stopScript()
                    stop = True
                time.sleep(0.1)
                continue
            stop = False

            pygame.event.pump()
            self.pos = self.recv.getActualTCPPose()
            if self.slave_recv:
                self.slave_pos = self.slave_recv.getActualTCPPose()

            self.speeds = self.get_speeds()

            # Дополнительная страховка прямо перед вызовом
            self.speeds = [float(x) for x in self.speeds]


            try:
                # Движение мастера (ЗАМЕНЕНО time=0.0 НА time=0.0)
                self.ctrl.speedL(self.speeds, acceleration=0.1, time=0.0)

                # Движение слейва (если мастер и есть слейв)
                if self.slave_ctrl:
                    if self.auto.value:
                        slave_speeds = self.get_slave_speeds()
                        slave_speeds = [float(x) for x in slave_speeds]
                        self.slave_ctrl.speedL(slave_speeds, acceleration=0.1, time=0.0)
                    else:
                        # В асинхронном режиме останавливаем слейв
                        self.slave_ctrl.speedL([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], acceleration=0.1, time=0.0)
            except Exception as e:
                error_msg = f"{type(e).__name__}:{str(e)}"
                self.logger.error(f"Ошибка при работе с роботом IP: {self.IP}, сообщение: {error_msg}")
                self.heartbeat.put((mp.current_process().name, "ERROR", error_msg, time.time()))
                break

            # Переключение режимов (только мастер)
            if self.joystick.get_button(6) and not self.joystick.get_button(7) and self.auto.value == 0:
                self.auto.value = 1
                self.logger.info("Система переведена в СИНХРОННЫЙ режим")
            if self.joystick.get_button(7) and not self.joystick.get_button(6) and self.auto.value == 1:
                self.auto.value = 0
                self.logger.info("Система переведена в АСИНХРОННЫЙ режим")

            # Возврат слейва в исходную (кнопка 8)
            if self.joystick.get_button(8) and self.auto.value == 0 and self.slave_ctrl:
                self.logger.info("Возврат слейва в исходную позицию")
                self.slave_ctrl.moveL(
                    (float(self.slave_pos[0]), float(self.slave_pos[1]), float(self.slave_pos[2]), 0.0, 3.14, 0.0),
                    speed=0.2, acceleration=0.2
                )

            # Запись пути (кнопка 10)
            if self.joystick.get_button(10):
                path.append(list(self.recv.getActualTCPPose()))  # Сохраняем как список float

            # Воспроизведение пути (кнопка 11)
            if self.joystick.get_button(11) and len(path) > 0:
                self.logger.info(f"Воспроизведение пути из {len(path)} точек")
                for pose in path:
                    self.ctrl.moveL(pose, speed=0.2, acceleration=0.2)
                    while self.ctrl.isProgramRunning():
                        pygame.event.pump()
                        if self.joystick.get_button(11):  # Повторное нажатие отменяет
                            self.ctrl.stopScript()
                            break
                        time.sleep(0.01)
                path = []  # Очистить после выполнения

            # Сохранение пути в общую память (кнопка 9)
            if self.joystick.get_button(9) and len(path) > 0:
                self.shared_path[0] = deepcopy(path)
                self.logger.info(f"Путь сохранен в общую память. Точек: {len(path)}")
                path = []
                time.sleep(1.0)  # Защита от дребезга кнопки

            time.sleep(0.01)

        self.close()

    def close(self):
        self.logger.info(f"Завершение работы и отключение от робота {self.IP}")
        if self.joystick:
            self.joystick.quit()
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
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        file_handler = RotatingFileHandler(
            filename=f'logs/processes_status_log_{datetime.now().strftime("%Y-%m-%d")}.log',
            maxBytes=10 * 1024 * 1024,
            backupCount=5,
            encoding='utf-8'
        )
        file_handler.setFormatter(formatter)
        proc_logger.addHandler(file_handler)

    proc_logger.info(f"Запуск рабочего процесса | PID: {os.getpid()}")

    robot = Robot(IP, is_master, proc_logger, auto, lock, shared_path, heartbeat, slave_IP)
    robot.start_robot()