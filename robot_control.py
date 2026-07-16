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

    def init_robot(self):
        self.logger.debug(f"Подключение к роботу IP: {self.IP}...")
        try:
            self.ctrl = RTDEControlInterface(self.IP)
            self.recv = RTDEReceiveInterface(self.IP)
            
            if not self.ctrl.isConnected():
                raise ConnectionError(f"Не удалось подключиться к RTDE контроллеру {self.IP}")
            self.logger.info(f"Успешное подключение к роботу IP: {self.IP}")
            
            if self.is_master:
                if self.slave_IP:
                    self.logger.debug(f"Подключение к slave роботу (RTDE) IP: {self.slave_IP}...")
                    self.slave_ctrl = RTDEControlInterface(self.slave_IP)
                    self.slave_recv = RTDEReceiveInterface(self.slave_IP)
                    if not self.slave_ctrl.isConnected():
                        raise ConnectionError(f"Не удалось подключиться к RTDE контроллеру slave {self.slave_IP}")
                    self.logger.info(f"Успешное подключение к slave роботу IP: {self.slave_IP}")
                else:
                    self.logger.error("Отсутствует slave IP")
                    self.heartbeat.put((mp.current_process().name, "ERROR", "Отсутствует slave IP", time.time()))
                    exit(1)
        except Exception as e:
            self.logger.error(f"Ошибка при подключении к {self.IP}: {e}")
            self.heartbeat.put((mp.current_process().name, "ERROR", str(e), time.time()))
            exit(1)

    def init_joystick(self):
        pygame.init()

        self.deadzone = 0.2

        if self.is_master:
            joystick_id = 0
        else:
            joystick_id = 1

        self.joystick = pygame.joystick.Joystick(joystick_id)
        self.joystick.init()
        self.logger.info(f"Инициализирован джойстик id={joystick_id}, name: {self.joystick.get_name()}")

    def start_robot(self):
        self.init_joystick()
        self.init_robot()
        self.set_robot_settings()
        self.main_loop()

    def set_robot_settings(self, linear_velocity=0.3, rotational_velocity=0.1, acceleration=0.1):
        self.linear_velocity = linear_velocity  # максимальная линейная скорость
        self.rotational_velocity = rotational_velocity  # максимальная круговая скорость
        self.acceleration = acceleration  # ускорение

    def get_speeds(self):
        speeds = [0, 0, 0, 0, 0, 0]  # массив скоростей

        speeds[1] = 1 * self.joystick.get_hat(0)[1] * self.linear_velocity  # скорость по оси у
        speeds[0] = 1 * self.joystick.get_hat(0)[0] * self.linear_velocity  # скорость по оси х
        if self.joystick.get_button(2) and not self.joystick.get_button(3):  # движение по оси z
            speeds[2] = 1 * -self.linear_velocity  # скорость по оси z
        if self.joystick.get_button(3) and not self.joystick.get_button(2):  # движение против оси z
            speeds[2] = 1 * self.linear_velocity  # скорость по оси z

        axis0 = self.joystick.get_axis(0)
        axis1 = self.joystick.get_axis(1)
        axis2 = self.joystick.get_axis(2)
        axis3 = self.joystick.get_axis(3)

        speeds[3] = (axis1 if abs(
            axis1) > self.deadzone else 0) * self.rotational_velocity  # круговая скорость в плоскости xy
        speeds[4] = (axis0 if abs(
            axis0) > self.deadzone else 0) * self.rotational_velocity  # круговая скорость в плоскости xz
        speeds[5] = (axis3 if abs(
            axis3) > self.deadzone else 0) * self.rotational_velocity  # круговая скорость в плоскости yz

        return speeds

    def update_heartbeat(self):
        current_time = time.time()
        if current_time - self.last_heartbeat > 2.0:
            self.heartbeat.put((mp.current_process().name, "ALIVE", time.time()))
            self.last_heartbeat = current_time

    def _pose_to_matrix(self, pose):
        """Конвертирует 6-мерный вектор позы UR [x,y,z,rx,ry,rz] в матрицу 4x4"""
        x, y, z, rx, ry, rz = pose
        theta = math.sqrt(rx**2 + ry**2 + rz**2)
        if theta < 1e-6:
            return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]
        
        ux, uy, uz = rx/theta, ry/theta, rz/theta
        c, s, t = math.cos(theta), math.sin(theta), 1 - c
        
        return [
            [t*ux*ux + c,   t*ux*uy - s*uz, t*ux*uz + s*uy, x],
            [t*ux*uy + s*uz, t*uy*uy + c,    t*uy*uz - s*ux, y],
            [t*ux*uz - s*uy, t*uy*uz + s*ux, t*uz*uz + c,    z],
            [0, 0, 0, 1]
        ]

    def get_slave_speeds(self):
        if not self.is_master:
            return
        T = self.robot.get_pose().array  # матрица перехода основание-конечное звено
        # запись элементов матрицы в переменные
        z1, z2 = -T[0][2], -T[1][2]
        yx, yy = T[0][1], -T[1][1]
        xx, xy = T[0][0], -T[1][0]

        speeds = [self.speeds[2] * z1 + self.speeds[0] * xx + self.speeds[1] * yx,
                  self.speeds[2] * z2 + self.speeds[0] * xy + self.speeds[1] * yy, 0, 0, 0, 0]
        return speeds

    def main_loop(self):
        self.running = True
        stop = False
        pygame.event.pump()

        while self.running:
            self.update_heartbeat()

            if self.lock.value:
                if not stop:
                    self.ctrl.stopScript()
                    stop = True
                continue
            else:
                stop = False

            self.pos = self.recv.getActualTCPPose()
            if self.is_master:
                self.slave_pos = self.slave_recv.getActualTCPPose()

            self.speeds = self.get_speeds()

            try:
                if self.joystick.get_button(0):  # если зажат курок
                    self.ctrl.speedL(self.speeds, acceleration=0.1, dt=0.008)  # перемещение робота в системе координат конечного звена
                    if self.auto.value:  # если включён режим синхронного перемещения
                        if self.is_master:
                            self.slave_speeds = self.get_slave_speeds()
                            self.slave_ctrl.speedL(self.slave_speeds, acceleration=0.1, dt=0.008)

                else:
                    self.robot.speedL(self.speeds, acceleration=0.1, dt=0.008)  # если курок не зажат, то робот перемещается в системе координат основания

            except Exception as e:
                error_msg = f"{type(e).__name__}:{str(e)[:100]}"
                self.logger.error(f"Ошибка при работе с роботом IP: {self.IP}, сообщение: {error_msg}")
                self.heartbeat.put((mp.current_process().name, "ERROR", error_msg, time.time()))
                break

            if self.joystick.get_button(6) and (not self.joystick.get_button(
                    7)) and self.auto.value == 0 and self.is_master:  # включение синхронного режима
                self.auto.value = 0
                self.logger.info(f"Система переведена в синхронный режим")

            if self.joystick.get_button(7) and (not self.joystick.get_button(
                    6)) and self.auto.value == 1 and self.is_master:  # включение асинхронного режима
                self.auto.value = 1
                self.logger.info(f"Система переведена в асинхронный режим")

            if self.joystick.get_button(8):
                if self.auto.value and self.is_master:
                    self.slave_ctrl.moveL(
                        (self.slave_pos[0], self.slave_pos[1], self.slave_pos[2], 0, 3.14, 0), 
                        velocity=0.2, acceleration=0.2, dt=0.008
                    )  # выравнивание хирурга

            if self.joystick.get_button(10):
                self.path.append(self.recv.getActualTCPPose())
                self.logger.debug(f"Записана точка {self.path[-1]}")
                
            if self.joystick.get_button(11):
                following_path = True
                for pose in self.path:
                    self.logger.debug(f"Перемещение в точку с координатами {pose}")
                    self.ctrl.moveL(pose, velocity=0.2, acceleration=0.2, dt=0.008)
                    while self.ctrl.isProgramRunning():
                        pygame.event.pump()
                        if self.joystick.get_button(11):
                            following_path = False
                            self.ctrl.stopScript()
                            break

            if self.joystick.get_button(9):
                self.shared_path[0] = deepcopy(self.path)
                self.logger.debug(f"Записана траектория {self.shared_path[0]}")
                self.path = []
                time.sleep(2)

        self.close()

    def close(self):
        if self.joystick:
            self.joystick.quit()
        if self.ctrl:
            self.ctrl.disconnect()
        if self.recv:
            self.recv.disconnect()
        if self.is_master:
            if self.slave_ctrl:
                self.slave_ctrl.disconnect()
            if self.slave_recv:
                self.slave_recv.disconnect()


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
