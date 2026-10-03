"""
Joystick control (Migrated to ur_rtde)
"""
import os
import time
import pygame
import sys
import multiprocessing as mp
import logging
from logging.handlers import RotatingFileHandler
from datetime import datetime

from ur_rtde import RTDEControlInterface, RTDEReceiveInterface
from copy import deepcopy

auto = None
f_lock = None
SP = []
p_lock = None
proc_logger = None
hb_queue = None
last_heartbeat = None
last_robot_activity = None
operation_timeout = 10.0

# Глобальные интерфейсы робота
ctrl = None
recv = None

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
        global ctrl, recv
        self.joystick = None
        self.ctrl = ctrl
        self.recv = recv

        self.linear_velocity = linear_velocity
        self.rotational_velocity = rotational_velocity
        self.acceleration = acceleration
        self.cmd = Cmd()

    def init_joystick(self):
        pygame.init()
        self.joystick = pygame.joystick.Joystick(1) # ID 1 для диагноста
        self.joystick.init()
        print(f'Initialized Joystick : {self.joystick.get_name()}')

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

    def loop(self, initiated_by=None):
        print("Starting diagnost loop")
        stop = False
        path = []
        following_path = False
        global last_heartbeat, hb_queue

        while True:
            current_time = time.time()
            if current_time - last_heartbeat > 2.0:
                hb_queue.put((mp.current_process().name, "ALIVE", time.time()))
                last_heartbeat = current_time

            if not auto.value:
                time.sleep(0.1)
                continue

            if f_lock.value or p_lock.value:
                if not stop:
                    self.ctrl.stopScript()
                    stop = True
                time.sleep(0.1)
                continue

            stop = False
            self.get_btn_state()
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
                else:
                    self.ctrl.speedL(speeds, acceleration=0.1, time=0.0)
            except Exception as e:
                error_msg = f"{type(e).__name__}:{str(e)[:100]}"
                proc_logger.error(f"Ошибка при работе с роботом: {error_msg}")
                hb_queue.put((mp.current_process().name, "ERROR", error_msg, time.time()))
                break

            if self.cmd.btn10:
                path.append(self.recv.getActualTCPPose())
                print('point recorded')

            if self.cmd.btn11:
                following_path = True
                for pose in path:
                    print("moving to recorded point")
                    self.ctrl.moveL(pose, speed=0.3, acceleration=1.0)
                    while self.ctrl.isProgramRunning():
                        self.get_btn_state()
                        if self.cmd.btn11:
                            following_path = False
                            self.ctrl.stopScript()
                            break
                path = []

            if self.cmd.btn8:
                SP[0] = deepcopy(path)
                print("Path saved to shared memory")

            if initiated_by:
                if not initiated_by.auto.value:
                    break

            time.sleep(0.01) # Небольшая задержка для снижения нагрузки на CPU

    def close(self):
        if self.joystick:
            self.joystick.quit()
        global ctrl, recv
        if ctrl:
            ctrl.disconnect()
        if recv:
            recv.disconnect()


def main(qu, a, shared_path, fl, pl):
    global auto, SP, f_lock, p_lock, hb_queue, proc_logger, last_heartbeat, ctrl, recv

    SP = shared_path
    auto = a
    f_lock = fl
    p_lock = pl
    hb_queue = qu

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
    proc_logger.debug("Подключение к роботу (RTDE)...")

    try:
        ctrl = RTDEControlInterface("192.168.8.3")
        recv = RTDEReceiveInterface("192.168.8.3")
        if not ctrl.isConnected():
            raise ConnectionError("Не удалось подключиться к RTDE контроллеру")

        proc_logger.info("Успешное подключение к роботу")
        hb_queue.put((mp.current_process().name, "READY", time.time()))
        last_heartbeat = time.time()

        service = Service(linear_velocity=0.01, rotational_velocity=0.19, acceleration=0.1)
        service.init_joystick()

        try:
            service.loop()
        finally:
            print('Джойстик диагноста отключён')
            service.close()

    except Exception as e:
        proc_logger.error(f"Критическая ошибка инициализации: {e}")
        hb_queue.put((mp.current_process().name, "ERROR", str(e), time.time()))


if __name__ == "__main__":
    print("Запуск в режиме тестирования (standalone)")
    try:
        ctrl = RTDEControlInterface("192.168.8.3")
        recv = RTDEReceiveInterface("192.168.8.3")

        service = Service(linear_velocity=0.01, rotational_velocity=0.19, acceleration=0.1)
        service.init_joystick()

        # Заглушки для глобальных переменных в тестовом режиме
        class DummyLock:
            value = False
        auto = DummyLock()
        auto.value = True
        f_lock = DummyLock()
        p_lock = DummyLock()
        last_heartbeat = time.time()

        service.loop()
    finally:
        service.close()