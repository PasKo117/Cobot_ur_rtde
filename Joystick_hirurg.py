"""
Joystick control for Surgeon Robot (Migrated to ur_rtde)
"""
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

auto = None
f_lock = None
p_lock = None
SP = []

proc_logger = None
hb_queue = None
last_heartbeat = None
last_robot_activity = None
operation_timeout = 10.0

# Глобальные интерфейсы
ctrl = None
recv = None
diag_ctrl = None
diag_recv = None

def _pose_to_matrix(pose):
    """Конвертирует 6-мерный вектор позы UR [x,y,z,rx,ry,rz] в матрицу 4x4"""
    x, y, z, rx, ry, rz = pose
    theta = math.sqrt(rx**2 + ry**2 + rz**2)
    if theta < 1e-6:
        return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]
    ux, uy, uz = rx/theta, ry/theta, rz/theta
    c = math.cos(theta)
    s = math.sin(theta)
    t = 1 - c
    return [
        [t*ux*ux + c,   t*ux*uy - s*uz, t*ux*uz + s*uy, x],
        [t*ux*uy + s*uz, t*uy*uy + c,    t*uy*uz - s*ux, y],
        [t*ux*uz - s*uy, t*uy*uz + s*ux, t*uz*uz + c,    z],
        [0, 0, 0, 1]
    ]

class Cmd(object):
    def __init__(self):
        self.reset()

    def reset(self):
        self.axis0 = 0
        self.axis1 = 0
        self.axis2 = 0
        self.axis3 = 0
        self.btn0 = 0
        self.btn1 = 0
        self.btn2 = 0
        self.btn3 = 0
        self.btn4 = 0
        self.btn5 = 0
        self.btn6 = 0
        self.btn7 = 0
        self.btn8 = 0
        self.btn9 = 0
        self.btn10 = 0
        self.btn11 = 0
        self.hat0 = [0, 0]


class Service(object):
    def __init__(self, linear_velocity=0.3, rotational_velocity=0.1, acceleration=0.1):
        global ctrl, recv, diag_ctrl, diag_recv
        self.joystick = None
        self.ctrl = ctrl
        self.recv = recv
        self.diag_ctrl = diag_ctrl
        self.diag_recv = diag_recv

        self.linear_velocity = linear_velocity
        self.rotational_velocity = rotational_velocity
        self.acceleration = acceleration
        self.cmd = Cmd()

    def get_btn_state(self):
        self.cmd.reset()
        pygame.event.pump()

        for i in range(0, self.joystick.get_numaxes()):
            val = self.joystick.get_axis(i)
            if i in (2, 5) and val != 0:
                val += 1
            if abs(val) < 0.2:
                val = 0
            tmp = f"self.cmd.axis{i} = {val}"
            if val != 0:
                exec(tmp)

        for i in range(0, self.joystick.get_numbuttons()):
            if self.joystick.get_button(i) != 0:
                tmp = f"self.cmd.btn{i} = 1"
                exec(tmp)

    def init_joystick(self):
        pygame.init()
        self.joystick = pygame.joystick.Joystick(0) # ID 0 для хирурга
        self.joystick.init()
        print(f'Initialized Joystick : {self.joystick.get_name()}')

    def loop(self):
        print("Starting surgeon loop")
        stop = False
        path = []
        global last_heartbeat, hb_queue

        while True:
            current_time = time.time()
            if current_time - last_heartbeat > 2.0:
                hb_queue.put((mp.current_process().name, "ALIVE", time.time()))
                last_heartbeat = current_time

            if p_lock.value:
                if not stop:
                    self.ctrl.stopScript()
                    stop = True
                time.sleep(0.1)
                continue

            stop = False
            self.get_btn_state()

            hpos = self.recv.getActualTCPPose()
            dpos = self.diag_recv.getActualTCPPose()

            speeds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            speeds[1] = 1 * self.joystick.get_hat(0)[1] * self.linear_velocity
            speeds[0] = 1 * self.joystick.get_hat(0)[0] * self.linear_velocity
            if self.cmd.btn2 and not self.cmd.btn3:
                speeds[2] = 1 * -self.linear_velocity
            if self.cmd.btn3 and not self.cmd.btn2:
                speeds[2] = 1 * self.linear_velocity

            speeds[3] = -1 * self.cmd.axis1 * self.rotational_velocity
            speeds[4] = -1 * self.cmd.axis0 * self.rotational_velocity
            speeds[5] = self.cmd.axis3 * self.rotational_velocity

            speeds = [-i for i in speeds]

            try:
                if self.cmd.btn0:
                    self.ctrl.speedToolL(speeds, acceleration=0.1, time=0.0)

                    T = _pose_to_matrix(hpos)
                    z1, z2 = -T[0][2], -T[1][2]
                    yx, yy = T[0][1], -T[1][1]
                    xx, xy = T[0][0], -T[1][0]

                    if not auto.value: # Синхронный режим
                        diag_speeds = [
                            speeds[2] * z1 + speeds[0] * xx + speeds[1] * yx,
                            speeds[2] * z2 + speeds[0] * xy + speeds[1] * yy,
                            0, 0, 0, 0
                        ]
                        self.diag_ctrl.speedL(diag_speeds, acceleration=0.1, time=0.0)
                else:
                    self.ctrl.speedL(speeds, acceleration=0.1, time=0.0)

            except Exception as e:
                error_msg = f"{type(e).__name__}:{str(e)[:100]}"
                proc_logger.error(f"Ошибка при работе с роботом: {error_msg}")
                hb_queue.put((mp.current_process().name, "ERROR", error_msg, time.time()))
                break

            # ИСПРАВЛЕННАЯ ЛОГИКА ПЕРЕКЛЮЧЕНИЯ РЕЖИМОВ
            if self.cmd.btn6 and (not self.cmd.btn7) and auto.value == 0:
                auto.value = 1
                proc_logger.info("Система переведена в СИНХРОННЫЙ режим")

            if self.cmd.btn7 and (not self.cmd.btn6) and auto.value == 1:
                auto.value = 0
                proc_logger.info("Система переведена в АСИНХРОННЫЙ режим")

            if self.cmd.btn8:
                if not auto.value:
                    self.diag_ctrl.moveL((dpos[0], dpos[1], dpos[2], 0, 3.14, 0), speed=0.2, acceleration=0.2)

            if self.cmd.btn10:
                path.append(self.recv.getActualTCPPose())
                proc_logger.debug(f"Записана точка {path[-1]}")

            if self.cmd.btn11:
                following_path = True
                for pose in path:
                    proc_logger.debug(f"Перемещение в точку с координатами {pose}")
                    self.ctrl.moveL(pose, speed=0.2, acceleration=0.2)
                    while self.ctrl.isProgramRunning():
                        self.get_btn_state()
                        if self.cmd.btn11:
                            following_path = False
                            self.ctrl.stopScript()
                            break

            if self.cmd.btn9:
                SP[0] = deepcopy(path)
                proc_logger.debug(f"Записана траектория {SP[0]}")
                path = []
                time.sleep(2)

            time.sleep(0.01)

    def close(self):
        if self.joystick:
            self.joystick.quit()
        global ctrl, recv, diag_ctrl, diag_recv
        if ctrl: ctrl.disconnect()
        if recv: recv.disconnect()
        if diag_ctrl: diag_ctrl.disconnect()
        if diag_recv: diag_recv.disconnect()


def main(qu, a, pl, shared_path):
    global auto, p_lock, SP, hb_queue, proc_logger, last_heartbeat, ctrl, recv, diag_ctrl, diag_recv

    hb_queue = qu
    last_heartbeat = time.time()
    last_robot_activity = time.time()
    operation_timeout = 10.0

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
    proc_logger.debug("Подключение к роботам (RTDE)...")

    try:
        # Подключение к хирургу
        ctrl = RTDEControlInterface("192.168.8.4")
        recv = RTDEReceiveInterface("192.168.8.4")
        if not ctrl.isConnected():
            raise ConnectionError("Не удалось подключиться к роботу хирурга")

        # Подключение к диагносту
        diag_ctrl = RTDEControlInterface("192.168.8.3")
        diag_recv = RTDEReceiveInterface("192.168.8.3")
        if not diag_ctrl.isConnected():
            raise ConnectionError("Не удалось подключиться к роботу диагноста")

        proc_logger.info("Успешное подключение к обоим роботам")

        auto = a
        p_lock = pl
        SP = shared_path

        service = Service(linear_velocity=0.015, rotational_velocity=0.19, acceleration=0.1)
        service.init_joystick()
        service.loop()

    except Exception as e:
        proc_logger.error(f"Критическая ошибка инициализации: {e}")
        hb_queue.put((mp.current_process().name, "ERROR", str(e), time.time()))
    finally:
        service.close()


if __name__ == "__main__":
    print("Запуск в режиме тестирования (standalone)")