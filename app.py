import tkinter as tk
# import tkinter.messagebox as mb
# from tkinter import ttk
from tkinter import filedialog

import customtkinter as ctk
# import CTkFileDialog as filedialog
from CTkMessagebox import CTkMessagebox as mb

import sys
print(sys.version)

import subprocess
import socket
import signal
import os
from time import sleep
import time
from datetime import datetime
from ur_rtde import RTDEControlInterface, RTDEReceiveInterface
import asyncio
from threading import Thread
import threading
import multiprocessing as mp
import math
import asyncio
from pathlib import Path
import logging
from logging.handlers import RotatingFileHandler
import csv

import Align_D
import Align_H
import Power_On_H
import camDiagn
import camHirurg
import ESTOP_RESET_D
import ESTOP_RESET_H
import Joystick_diagnost
import Joystick_hirurg
import Power_On_D
import Power_Off_H
import Power_Off_D
import robot_control

rob_us_data = []
us_lock = False
stop_route = threading.Event()
starting_pose = []

auto = mp.Value("i", 1)
force_lock = mp.Value("i", 0)
control_lock = mp.Value("i", 0)
program_lock = mp.Value("i", 0)

# Настройки внешнего вида
ctk.set_appearance_mode("Dark")  # Темы: "Dark", "Light", "System"
ctk.set_default_color_theme("blue")  # Темы: "blue", "green", "dark-blue"


def setup_status_logging(log_dir="logs", max_bytes=10 * 1024 * 1024, backup_count=5):
    log_dir = Path(log_dir)
    log_dir.mkdir(exist_ok=True)

    log_file = log_dir / f'processes_status_log_{datetime.now().strftime("%Y-%m-%d")}.log'

    logger = logging.getLogger("ProcessStatus")
    logger.setLevel(logging.DEBUG)

    if logger.handlers:
        return logger

    formatter = logging.Formatter(
        fmt='%(asctime)s.%(msecs)03d | %(levelname)-8s | %(processName)-15s | %(threadName)-15s | %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )

    file_handler = RotatingFileHandler(
        filename=log_file,
        maxBytes=max_bytes,
        backupCount=backup_count,
        encoding='utf-8'
    )

    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(formatter)

    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(formatter)

    logger.addHandler(file_handler)
    logger.addHandler(console_handler)

    logger.info("=" * 80)
    logger.info(f'ЗАПУСК ЕППУИХ | PID: {os.getpid()}')
    logger.info("=" * 80)

    return logger

def _pose_to_matrix(pose):
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

def _matrix_to_pose(T):
    x, y, z = T[0][3], T[1][3], T[2][3]
    R = [[T[0][0], T[0][1], T[0][2]],
         [T[1][0], T[1][1], T[1][2]],
         [T[2][0], T[2][1], T[2][2]]]
    theta = math.acos(max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1) / 2)))
    if theta < 1e-6:
        rx, ry, rz = 0.0, 0.0, 0.0
    else:
        rx = (R[2][1] - R[1][2]) / (2 * math.sin(theta)) * theta
        ry = (R[0][2] - R[2][0]) / (2 * math.sin(theta)) * theta
        rz = (R[1][0] - R[0][1]) / (2 * math.sin(theta)) * theta
    return [x, y, z, rx, ry, rz]

def _multiply_matrices(A, B):
    return [[sum(a * b for a, b in zip(A_row, B_col)) for B_col in zip(*B)] for A_row in A]

def translate_base(ctrl, recv, dx, dy, dz, vel, acc):
    pose = recv.getActualTCPPose()
    pose[0] += dx
    pose[1] += dy
    pose[2] += dz
    ctrl.moveL(pose, velocity=vel, acceleration=acc)

def translate_tool(ctrl, recv, dx, dy, dz, vel, acc):
    pose = recv.getActualTCPPose()
    T = _pose_to_matrix(pose)
    delta_T = [[1, 0, 0, dx], [0, 1, 0, dy], [0, 0, 1, dz], [0, 0, 0, 1]]
    T_new = _multiply_matrices(T, delta_T)
    new_pose = _matrix_to_pose(T_new)
    ctrl.moveL(new_pose, velocity=vel, acceleration=acc)

def rotate_tool_x(ctrl, recv, angle_rad, vel, acc):
    pose = recv.getActualTCPPose()
    T = _pose_to_matrix(pose)
    T_rot = [
        [1, 0, 0, 0],
        [0, math.cos(angle_rad), -math.sin(angle_rad), 0],
        [0, math.sin(angle_rad), math.cos(angle_rad), 0],
        [0, 0, 0, 1]
    ]
    T_new = _multiply_matrices(T, T_rot)
    new_pose = _matrix_to_pose(T_new)
    ctrl.moveL(new_pose, velocity=vel, acceleration=acc)

class CSVLogger:
    def __init__(self, filename="logs/telemetry_log.csv"):
        self.filename = filename
        self.enabled = False
        self._ensure_header()

    def _ensure_header(self):
        if not os.path.exists(self.filename):
            with open(self.filename, "w", newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(
                    ["timestamp", "robot", "temperature", "X", "Y", "Z", "J1", "J2", "J3", "J4", "J5", "J6", "FX", "FY",
                     "FZ", "MX", "MY", "MZ"])

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def log_data(self, data):
        if not self.enabled:
            return

        try:
            timestamp = datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
            with open(self.filename, "a", newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([timestamp] + [data])
        except Exception as e:
            logger.error(f'Ошибка логгирования телеметрии: {type(e).__name__}:{str(e)[:100]}')


def force_control(threshold):
    ctrl = RTDEControlInterface("192.168.8.3")
    recv = RTDEReceiveInterface("192.168.8.3")
    global stop_route
    while True:
        forces = recv.getActualTCPForce()
        if forces[2] > threshold:
            force_lock.value = 1
            ctrl.speedToolL([0] * 6, 0.5, dt=0.008)
            stop_route.set()
            time.sleep(1)
            translate_tool(ctrl, recv, 0, 0, -0.1, 0.5, 0.5)
            while ctrl.isProgramRunning():
                time.sleep(0.01)
            time.sleep(10)
            force_lock.value = 0
        else:
            force_lock.value = 0
        time.sleep(0.05)


def route_follow(nsteps, S, pause_time, vel):
    ctrl = RTDEControlInterface("192.168.8.3")
    recv = RTDEReceiveInterface("192.168.8.3")
    global rob_us_data, us_lock, stop_route, starting_pose
    force = [recv.getActualTCPForce()[2]]
    t = [0]
    coordinates = [0]
    start = time.time()
    us_lock = True
    starting_pose = recv.getActualTCPPose()
    for x in range(nsteps):
        if stop_route.is_set():
            stop_route.clear()
            rob_us_data = [t, coordinates, force]
            app.show_warning("Маршрут остановлен")
            ctrl.disconnect()
            recv.disconnect()
            break

        translate_base(ctrl, recv, S[0], 0, 0, vel, 0.1)
        while ctrl.isProgramRunning():
            if stop_route.is_set():
                stop_route.clear()
                rob_us_data = [t, coordinates, force]
                app.show_warning("Маршрут остановлен")
                ctrl.disconnect()
                recv.disconnect()
                break
            time.sleep(0.01)
            pass

        translate_base(ctrl, recv, S[1], 0, 0, vel, 0.1)
        while ctrl.isProgramRunning():
            if stop_route.is_set():
                stop_route.clear()
                rob_us_data = [t, coordinates, force]
                app.show_warning("Маршрут остановлен")
                ctrl.disconnect()
                recv.disconnect()
                break
            time.sleep(0.01)
            pass

        translate_base(ctrl, recv, S[2], 0, 0, vel, 0.1)
        while ctrl.isProgramRunning():
            if stop_route.is_set():
                stop_route.clear()
                rob_us_data = [t, coordinates, force]
                app.show_warning("Маршрут остановлен")
                ctrl.disconnect()
                recv.disconnect()
                break
            time.sleep(0.01)
            pass

        t.append(time.time() - start)
        time.sleep(pause_time)
        coordinates.append(recv.getActualTCPPose())
        force.append(recv.getActualTCPForce()[2])

        if stop_route.is_set():
            rob_us_data = [t, coordinates, force]
            app.show_warning("Маршрут остановлен")
            ctrl.disconnect()
            recv.disconnect()
            us_lock = False
            stop_route.clear()
            return
    rob_us_data = [t, coordinates, force]
 
    us_lock = False
    ctrl.disconnect()
    recv.disconnect()


def aphi(angle, nsteps, step, pause_time, vel, tool_length=0.645):
    global us_lock, stop_route

    ctrl = RTDEControlInterface("192.168.8.4")
    recv = RTDEReceiveInterface("192.168.8.4")

    for s in range(nsteps):
        translate_tool(ctrl, recv, 0, 0, -step, vel, 0.1)
        while ctrl.isProgramRunning():
            time.sleep(0.01)
        time.sleep(pause_time)
    while ctrl.isProgramRunning():
        time.sleep(0.01)
    us_lock = False
    ctrl.disconnect()
    recv.disconnect()

def ashido_init(nsteps, step, angle=0, tool_length=0.645):
    global us_lock
    ctrl = RTDEControlInterface("192.168.8.4")
    recv = RTDEReceiveInterface("192.168.8.4")
    pose = recv.getActualTCPPose()
    if angle != 0:
        ctrl.moveL((pose[0], pose[1], pose[2], 3.14 / 2, 0, 0), velocity=0.2, acceleration=0.2)
        while ctrl.isProgramRunning():
            time.sleep(0.01)
            
    angle_rad = angle * (math.pi / 180)
    r = tool_length + 0.195
    h = r * math.sin(angle_rad) + 0.005
    delta = r - r * math.cos(angle_rad)

    translate_base(ctrl, recv, 0, 0, h, 0.5, 0.1)
    while ctrl.isProgramRunning():
        time.sleep(0.01)
        
    translate_tool(ctrl, recv, 0, 0, delta, 0.1, 0.1)
    while ctrl.isProgramRunning():
        time.sleep(0.01)
        
    rotate_tool_x(ctrl, recv, angle_rad, 0.1, 0.1)
    while ctrl.isProgramRunning():
        time.sleep(0.01)
        
    translate_tool(ctrl, recv, 0, 0, nsteps * step, 0.1, 0.1)
    while ctrl.isProgramRunning():
        time.sleep(0.01)

    us_lock = False
    ctrl.disconnect()
    recv.disconnect()
    program_lock.value = 0


async def ashido(nsteps, step, pause_time, vel, angle=0, is_hirurg=0, tool_length=0.645):
    global rob_us_data, us_lock, stop_route
    loop = asyncio.get_running_loop()

    hirurg_ctrl = RTDEControlInterface("192.168.8.4")
    hirurg_recv = RTDEReceiveInterface("192.168.8.4")
    diag_ctrl = RTDEControlInterface("192.168.8.3")
    diag_recv = RTDEReceiveInterface("192.168.8.3")

    time.sleep(5)

    if not (hirurg_ctrl.isConnected() and diag_ctrl.isConnected()):
        hirurg_ctrl.disconnect(); hirurg_recv.disconnect()
        diag_ctrl.disconnect(); diag_recv.disconnect()
        return -1
    if angle != 0:
        dv = vel * math.cos(angle * math.pi / 180) if vel * math.cos(angle * math.pi / 180) >= 0.001 else 0.001
        ds = step * math.cos(angle * math.pi / 180) if step * math.cos(angle * math.pi / 180) >= 0.001 else 0.001
        dv = vel
        ds = step

    t = [0]
    coordinates = [0]
    start = time.time()
    us_lock = True
    starting_pose = diag_recv.getActualTCPPose()

    for s in range(nsteps):
        await loop.run_in_executor(None, translate_tool, hirurg_ctrl, hirurg_recv, 0, 0, -step, vel, vel)
        await loop.run_in_executor(None, translate_base, diag_ctrl, diag_recv, 0, -ds, 0, dv, dv)
        while hirurg_ctrl.isProgramRunning() or diag_ctrl.isProgramRunning():
            await asyncio.sleep(0.01)
            
        t.append(time.time() - start)
        coordinates.append(diag_recv.getActualTCPPose())

    rob_us_data = [t, coordinates]
    us_lock = False
    hirurg_ctrl.disconnect()
    hirurg_recv.disconnect()
    diag_ctrl.disconnect()
    diag_recv.disconnect()
    program_lock.value = 0


class RobotControlUI(ctk.CTk):
    def __init__(self):
        super().__init__()

        # Конфигурация главного окна
        self.title('ЕППУИХ - Система управления манипуляторами')
        self.geometry("1100x750")
        self.minsize(900, 650)

        # Переменные системы
        self.threads = []
        self.processes = {}
        self.heartbeat = mp.Queue()
        self.heartbeat_timeout = 10.0
        self.restart_delays = {}
        self.tel_logging_var = ctk.BooleanVar(value=False)

        self.ppui_stop_event = threading.Event()
        self.monitor_stop_event = threading.Event()
        self.watchdog_stop_event = threading.Event()

        self.surgeon_ip = "192.168.8.4"
        self.diagnost_ip = "192.168.8.3"

        # Переменные для телеметрии
        self.telemetry_vars = {
            'diag_status': ctk.StringVar(value="Откл"),
            'hir_status': ctk.StringVar(value="Откл"),
            'diag_las': ctk.StringVar(value="0.00"),
            'hir_las': ctk.StringVar(value="0.00"),
            'diag_force': ctk.StringVar(value="0.00"),
            'hir_force': ctk.StringVar(value="0.00")
        }

        self.telemetry_logger = None

        # Создание интерфейса
        self.create_tabs()

        self.monitor = Thread(target=self.system_monitor, daemon=True)
        # self.monitor.start()

        self.watchdog_thread = Thread(target=self.watchdog, daemon=True, name="Watchdog")
        self.watchdog_thread.start()
        logger.info(f"Запущен фоновый монитор сердцебиения (поток: {self.watchdog_thread.name})")

        # self.force_control_thread = Thread(target=force_control, args = (17,))
        # self.force_control_thread.start()

    def create_tabs(self):
        """Создание вкладок"""
        self.tab_view = ctk.CTkTabview(self, corner_radius=10)
        self.tab_view.pack(expand=True, fill="both", padx=10, pady=10)

        # Создание вкладок
        self.control_tab = self.tab_view.add('Управление')
        self.route_tab = self.tab_view.add('Исследование')
        self.aphi_tab = self.tab_view.add('Воздействие')
        self.ashido_tab = self.tab_view.add('Операция')
        self.cam_tab = self.tab_view.add('Камеры')
        self.telemetry_tab = self.tab_view.add('Телеметрия')

        # Заполнение вкладок
        self.create_control_tab()
        self.create_route_tab()
        self.create_aphi_tab()
        self.create_ashido_tab()
        self.create_cam_tab()
        self.create_telemetry_tab()

    def create_control_tab(self):
        """Вкладка основного управления (РПК)"""
        # Сетка для вкладок
        self.control_tab.grid_columnconfigure(0, weight=1)
        self.control_tab.grid_columnconfigure(1, weight=1)
        self.control_tab.grid_rowconfigure(0, weight=0)
        self.control_tab.grid_rowconfigure(1, weight=0)

        # --- Левая колонка: Система и Роботы ---
        left_frame = ctk.CTkFrame(self.control_tab, corner_radius=10)
        left_frame.grid(column=0, row=0, rowspan=2, padx=10, pady=10, sticky="nsew")

        # Система
        sys_label = ctk.CTkLabel(left_frame, text="СИСТЕМА", font=ctk.CTkFont(size=14, weight="bold"))
        sys_label.grid(column=0, row=0, columnspan=2, pady=(10, 5))

        self.system_launch_btn = ctk.CTkButton(left_frame, text='Запуск системы', command=self.system_launch,
                                               fg_color="#2CC985", hover_color="#25A56E")
        self.system_launch_btn.grid(column=0, row=1, padx=10, pady=5, sticky="ew")

        self.system_stop_btn = ctk.CTkButton(left_frame, text='Остановка системы', command=self.system_stop,
                                             state="disabled", fg_color="#E74C3C", hover_color="#C0392B")
        self.system_stop_btn.grid(column=1, row=1, padx=10, pady=5, sticky="ew")

        self.control_initiate_btn = ctk.CTkButton(left_frame, text='Инициализировать контроллеры',
                                                  command=self.control_initiate, fg_color="#2CC985",
                                                  hover_color="#25A56E")
        self.control_initiate_btn.grid(column=0, row=2, padx=10, pady=5, sticky="ew")

        self.control_disable_btn = ctk.CTkButton(left_frame, text='Отключить контроллеры',
                                                  command=self.control_initiate, state="disabled", fg_color="#E74C3C", hover_color="#C0392B")
        self.control_disable_btn.grid(column=1, row=2, padx=10, pady=5, sticky="ew")

        # Разделитель
        sep1 = ctk.CTkFrame(left_frame, height=2, fg_color="#555555")
        sep1.grid(column=0, row=4, columnspan=2, pady=10, sticky="ew")

        # Управление
        robot_label = ctk.CTkLabel(left_frame, text="РОБОТЫ", font=ctk.CTkFont(size=14, weight="bold"))
        robot_label.grid(column=0, row=4, columnspan=2, pady=(10, 5))

        self.control_launch_btn = ctk.CTkButton(left_frame, text='Запуск управления', command=self.control_launch,
                                                state="disabled")
        self.control_launch_btn.grid(column=0, row=5, padx=10, pady=5, sticky="ew")

        self.control_stop_btn = ctk.CTkButton(left_frame, text='Остановка упр.', command=self.control_stop,
                                              state="disabled",
                                              fg_color="#E74C3C", hover_color="#C0392B")
        self.control_stop_btn.grid(column=1, row=5, padx=10, pady=5, sticky="ew")

        self.align_btn = ctk.CTkButton(left_frame, text='Выравнивание', command=self.align, state="disabled")
        self.align_btn.grid(column=0, row=6, padx=10, pady=5, sticky="ew")

        self.unlock_btn = ctk.CTkButton(left_frame, text='Разблокировка', command=self.unlock,
                                        fg_color="#E67E22", hover_color="#D35400")
        self.unlock_btn.grid(column=1, row=6, padx=10, pady=5, sticky="ew")

        # --- Правая колонка: Маршруты ---
        right_frame = ctk.CTkFrame(self.control_tab, corner_radius=10)
        right_frame.grid(column=1, row=0, rowspan=2, padx=10, pady=10, sticky="nsew")

        route_label = ctk.CTkLabel(right_frame, text="МАРШРУТЫ (ФАЙЛЫ)", font=ctk.CTkFont(size=14, weight="bold"))
        route_label.grid(column=0, row=0, pady=10)

        self.save_d_route_btn = ctk.CTkButton(right_frame, text='Сохранить маршрут диагноста',
                                              command=self.save_d_route)
        self.save_d_route_btn.grid(column=0, row=1, padx=10, pady=5, sticky="ew")

        self.save_h_route_btn = ctk.CTkButton(right_frame, text='Сохранить маршрут хирурга', command=self.save_h_route)
        self.save_h_route_btn.grid(column=0, row=2, padx=10, pady=5, sticky="ew")

        self.launch_d_route_btn = ctk.CTkButton(right_frame, text='Запуск маршрута диагноста',
                                                command=self.launch_d_route,
                                                fg_color="#3498DB", hover_color="#2980B9")
        self.launch_d_route_btn.grid(column=0, row=3, padx=10, pady=5, sticky="ew")

        self.launch_h_route_btn = ctk.CTkButton(right_frame, text='Запуск маршрута хирурга',
                                                command=self.launch_h_route,
                                                fg_color="#3498DB", hover_color="#2980B9")
        self.launch_h_route_btn.grid(column=0, row=4, padx=10, pady=5, sticky="ew")

        right_frame.grid_columnconfigure(0, weight=1)

    def create_route_tab(self):
        """Вкладка настройки маршрута (ППУИ)"""
        self.route_tab.grid_columnconfigure(0, weight=1)

        # Параметры
        params_frame = ctk.CTkFrame(self.route_tab, corner_radius=10)
        params_frame.grid(column=0, row=0, padx=10, pady=10, sticky="nsew")
        params_frame.grid_columnconfigure(1, weight=1)

        ctk.CTkLabel(params_frame, text="ПАРАМЕТРЫ ДВИЖЕНИЯ", font=ctk.CTkFont(size=14, weight="bold")).grid(column=0,
                                                                                                             row=0,
                                                                                                             columnspan=4,
                                                                                                             pady=10)

        # Количество шагов
        ctk.CTkLabel(params_frame, text="Количество шагов:").grid(column=0, row=1, sticky="e", padx=10, pady=5)
        self.num_steps_entry = ctk.CTkEntry(params_frame, width=100,placeholder_text='30')
        self.num_steps_entry.grid(column=1, row=1, sticky="w", padx=5, pady=5)

        # Величина шага
        ctk.CTkLabel(params_frame, text="Величина шага (м):").grid(column=0, row=2, sticky="e", padx=10, pady=5)

        ctk.CTkLabel(params_frame, text="X:").grid(column=2, row=2, padx=5)
        self.step_x_entry = ctk.CTkEntry(params_frame, width=60, placeholder_text='0.005')
        self.step_x_entry.grid(column=3, row=2, padx=2, pady=5)

        ctk.CTkLabel(params_frame, text="Y:").grid(column=4, row=2, padx=5)
        self.step_y_entry = ctk.CTkEntry(params_frame, width=60,placeholder_text='0.005')
        self.step_y_entry.grid(column=5, row=2, padx=2, pady=5)

        ctk.CTkLabel(params_frame, text="Z:").grid(column=6, row=2, padx=5)
        self.step_z_entry = ctk.CTkEntry(params_frame, width=60,placeholder_text='0.005')
        self.step_z_entry.grid(column=7, row=2, padx=2, pady=5)

        # Время и скорость
        ctk.CTkLabel(params_frame, text="Время остановки (с):").grid(column=0, row=3, sticky="e", padx=10, pady=5)
        self.pause_time_entry = ctk.CTkEntry(params_frame, width=100, placeholder_text='1')
        self.pause_time_entry.grid(column=1, row=3, sticky="w", padx=5, pady=5)

        ctk.CTkLabel(params_frame, text="Скорость (м/с):").grid(column=0, row=4, sticky="e", padx=10, pady=5)
        self.velocity_entry = ctk.CTkEntry(params_frame, width=100, placeholder_text='0.005')
        self.velocity_entry.grid(column=1, row=4, sticky="w", padx=5, pady=5)

        # Кнопки действий
        btn_frame = ctk.CTkFrame(self.route_tab, corner_radius=10)
        btn_frame.grid(column=0, row=1, padx=10, pady=10, sticky="nsew")
        btn_frame.grid_columnconfigure((0, 1, 2, 3), weight=1)

        self.start_route_btn = ctk.CTkButton(btn_frame, text="▶ Начать маршрут", command=self.start_route,
                                             fg_color="#2CC985", hover_color="#25A56E")
        self.start_route_btn.grid(column=0, row=0, padx=5, pady=10)

        self.stop_route_btn = ctk.CTkButton(btn_frame, text="⏹ Остановить", command=self.stop_routef,
                                            fg_color="#E74C3C", hover_color="#C0392B")
        self.stop_route_btn.grid(column=1, row=0, padx=5, pady=10)

        self.return_to_start_btn = ctk.CTkButton(btn_frame, text="↩ Вернуться в начало", command=self.return_to_start)
        self.return_to_start_btn.grid(column=2, row=0, padx=5, pady=10)

        self.save_route_btn = ctk.CTkButton(btn_frame, text="💾 Сохранить данные", command=self.save_route)
        self.save_route_btn.grid(column=3, row=0, padx=5, pady=10)

    def create_aphi_tab(self):
        """Вкладка АПХИ"""
        self.aphi_tab.grid_columnconfigure(0, weight=1)

        settings_frame = ctk.CTkFrame(self.aphi_tab, corner_radius=10)
        settings_frame.grid(column=0, row=0, padx=20, pady=20, sticky="nsew")
        settings_frame.grid_columnconfigure(1, weight=1)

        ctk.CTkLabel(settings_frame, text="НАСТРОЙКИ ПРОДВИЖЕНИЯ", font=ctk.CTkFont(size=14, weight="bold")).grid(
            column=0, row=0, columnspan=2, pady=15)

        # Поля ввода
        entries_config = [
            ("Угол продвижения (°)", "move_angle_entry"),
            ("Количество шагов", "aphi_num_steps_entry"),
            ("Величина шага (м)", "aphi_step_val_entry"),
            ("Время остановки (с)", "aphi_pause_time_entry"),
            ("Скорость (м/с)", "aphi_velocity_entry")
        ]

        for i, (label_text, attr_name) in enumerate(entries_config):
            ctk.CTkLabel(settings_frame, text=label_text).grid(column=0, row=i + 1, sticky="e", padx=10, pady=8)
            entry = ctk.CTkEntry(settings_frame, width=150)
            entry.grid(column=1, row=i + 1, sticky="w", padx=10, pady=8)
            setattr(self, attr_name, entry)

        # Кнопки
        btn_frame = ctk.CTkFrame(self.aphi_tab, corner_radius=10)
        btn_frame.grid(column=0, row=1, padx=20, pady=10)

        self.start_aphi_btn = ctk.CTkButton(btn_frame, text="▶ Начать продвижение", command=self.start_aphi,
                                            fg_color="#2CC985", hover_color="#25A56E", width=180)
        self.start_aphi_btn.grid(column=0, row=0, padx=10, pady=10)

        self.stop_aphi_btn = ctk.CTkButton(btn_frame, text="⏹ Остановить", command=self.stop_aphi,
                                           fg_color="#E74C3C", hover_color="#C0392B", width=150)
        self.stop_aphi_btn.grid(column=1, row=0, padx=10, pady=10)

        self.return_to_zero_btn = ctk.CTkButton(btn_frame, text="↩ Сброс позиции", command=self.fuck_go_back, width=150)
        self.return_to_zero_btn.grid(column=2, row=0, padx=10, pady=10)

        self.save_aphi_btn = ctk.CTkButton(btn_frame, text="💾 Сохранить путь", command=self.save_aphi, width=150)
        self.save_aphi_btn.grid(column=3, row=0, padx=10, pady=10)

    def create_ashido_tab(self):
        """Вкладка АСХИДО"""
        # Центрирование контента
        self.ashido_tab.grid_rowconfigure(0, weight=1)
        self.ashido_tab.grid_columnconfigure(0, weight=1)

        frame = ctk.CTkFrame(self.ashido_tab, corner_radius=15, border_width=2, border_color="#3498DB")
        frame.place(relx=0.5, rely=0.5, anchor="center")

        ctk.CTkLabel(frame, text="ПОЗИЦИОНИРОВАНИЕ ХИРУРГА", font=ctk.CTkFont(size=16, weight="bold")).grid(column=0,
                                                                                                            row=0,
                                                                                                            pady=20)

        self.ashido_position_btn = ctk.CTkButton(frame, text="📍 Установить в начальную точку", command=self.ashido_pos,
                                                 width=300, height=40)
        self.ashido_position_btn.grid(column=0, row=1, pady=15)

        self.ashido_start_btn = ctk.CTkButton(frame, text="🚀 НАЧАТЬ РАБОТУ", command=self.ashido_start,
                                              fg_color="#2CC985", hover_color="#25A56E", width=300, height=50,
                                              font=ctk.CTkFont(size=14, weight="bold"))
        self.ashido_start_btn.grid(column=0, row=2, pady=15)

    def create_cam_tab(self):
        """Вкладка Камеры"""
        self.cam_tab.grid_rowconfigure(0, weight=1)
        self.cam_tab.grid_columnconfigure(0, weight=1)

        frame = ctk.CTkFrame(self.cam_tab, corner_radius=15)
        frame.place(relx=0.5, rely=0.5, anchor="center")

        ctk.CTkLabel(frame, text="УПРАВЛЕНИЕ ВИДЕОПОТОКОМ", font=ctk.CTkFont(size=14, weight="bold")).grid(column=0,
                                                                                                           row=0,
                                                                                                           columnspan=2,
                                                                                                           pady=20)

        self.cam_d_btn = ctk.CTkButton(frame, text='📷 Камера диагноста', command=self.cam_d, width=200, height=50)
        self.cam_d_btn.grid(column=0, row=1, padx=20, pady=20)

        self.cam_h_btn = ctk.CTkButton(frame, text='📷 Камера хирурга', command=self.cam_h, width=200, height=50)
        self.cam_h_btn.grid(column=1, row=1, padx=20, pady=20)

    def create_telemetry_tab(self):
        """Вкладка Телеметрия"""
        self.telemetry_tab.grid_columnconfigure((0, 1), weight=1)

        # Статус подключения
        conn_frame = ctk.CTkFrame(self.telemetry_tab, corner_radius=10)
        conn_frame.grid(column=0, row=0, columnspan=2, sticky="nsew", padx=10, pady=10)
        conn_frame.grid_columnconfigure(1, weight=1)

        ctk.CTkLabel(conn_frame, text="СТАТУС ПОДКЛЮЧЕНИЯ", font=ctk.CTkFont(size=14, weight="bold")).grid(column=0,
                                                                                                           row=0,
                                                                                                           columnspan=3,
                                                                                                           pady=10)

        self._create_telemetry_row(conn_frame, 1, "Диагност:", self.telemetry_vars['diag_status'],
                                   status_color="#2CC985")
        self._create_telemetry_row(conn_frame, 2, "Хирург:", self.telemetry_vars['hir_status'], status_color="#2CC985")

        # Датчики
        sensor_frame = ctk.CTkFrame(self.telemetry_tab, corner_radius=10)
        sensor_frame.grid(column=0, row=1, columnspan=2, sticky="nsew", padx=10, pady=10)
        sensor_frame.grid_columnconfigure(1, weight=1)

        ctk.CTkLabel(sensor_frame, text="ПОКАЗАНИЯ ДАТЧИКОВ", font=ctk.CTkFont(size=14, weight="bold")).grid(column=0,
                                                                                                             row=0,
                                                                                                             columnspan=3,
                                                                                                             pady=10)

        self._create_telemetry_row(sensor_frame, 1, "Лазер (Диагност):", self.telemetry_vars['diag_las'], unit="мм")
        self._create_telemetry_row(sensor_frame, 2, "Лазер (Хирург):", self.telemetry_vars['hir_las'], unit="мм")
        self._create_telemetry_row(sensor_frame, 3, "Сила (Диагност):", self.telemetry_vars['diag_force'], unit="Н")
        self._create_telemetry_row(sensor_frame, 4, "Сила (Хирург):", self.telemetry_vars['hir_force'], unit="Н")

        # Логирование
        log_frame = ctk.CTkFrame(self.telemetry_tab, corner_radius=10)
        log_frame.grid(column=0, row=2, columnspan=2, sticky="nsew", padx=10, pady=10)

        self.toggle_telemetry_log_btn = ctk.CTkCheckBox(log_frame, text="Включить запись в CSV",
                                                        variable=self.tel_logging_var,
                                                        command=self.toggle_telemetry_logging, checkbox_width=20,
                                                        checkbox_height=20)
        self.toggle_telemetry_log_btn.grid(column=0, row=0, padx=20, pady=15, sticky="w")

        self.telemetry_logging_status_label = ctk.CTkLabel(log_frame, text="⏺ Статус: ОТКЛЮЧЕНО",
                                                           font=ctk.CTkFont(weight="bold"), text_color="#E74C3C")
        self.telemetry_logging_status_label.grid(column=1, row=0, padx=20, pady=15, sticky="w")

    def _create_telemetry_row(self, parent, row, label_text, variable, unit="", status_color="#3498DB"):
        """Хелпер для создания строк телеметрии"""
        lbl = ctk.CTkLabel(parent, text=label_text, font=ctk.CTkFont(weight="bold"))
        lbl.grid(column=0, row=row, sticky="e", padx=15, pady=8)

        # Поле данных с выделением
        data_lbl = ctk.CTkLabel(parent, textvariable=variable, font=ctk.CTkFont(family="Consolas", size=13),
                                corner_radius=5, fg_color="#2B2B2B", width=120, height=30)
        data_lbl.grid(column=1, row=row, sticky="w", padx=10, pady=8)

        if unit:
            unit_lbl = ctk.CTkLabel(parent, text=unit, text_color="#888888", font=ctk.CTkFont(size=11))
            unit_lbl.grid(column=2, row=row, sticky="w", padx=5)

    def toggle_telemetry_logging(self):
        if self.tel_logging_var.get():
            self.telemetry_logger.enable()
            self.telemetry_logging_status_label.config(text="Логирование: ВКЛЮЧЕНО", foreground="green")
        else:
            self.telemetry_logger.disable()
            self.telemetry_logging_status_label.config(text="Логирование: ВЫКЛЮЧЕНО", foreground="red")

    def system_launch(self):
        self.system_launch_btn.configure(state=ctk.DISABLED)
        self.system_stop_btn.configure(state=ctk.NORMAL)
        self.control_launch_btn.configure(state=ctk.NORMAL)
        self.align_btn.configure(state=ctk.NORMAL)
        params = (self.heartbeat,)

        self.processes['power_on_surgeon'] = {
            'process': mp.Process(target=Power_On_H.main, daemon=True, name='power_on_surgeon', args=params),
            'last_heartbeat': time.time(),
            'start_time': time.time(),
            'params': params,
            'state': 'AWAITING',
            'pid': None,
            'target': Power_On_H.main
        }
        self.heartbeat.put(('power_on_surgeon', "AWAITING"))

        self.processes['power_on_diagnost'] = {
            'process': mp.Process(target=Power_On_D.main, daemon=True, name='power_on_diagnost', args=params),
            'last_heartbeat': time.time(),
            'start_time': time.time(),
            'params': params,
            'state': 'AWAITING',
            'pid': None,
            'target': Power_On_D.main
        }
        self.heartbeat.put(('power_on_diagnost', "AWAITING"))
        self.monitor.start()

    def system_stop(self):
        print('system_stopped')
        self.system_launch_btn.configure(state=ctk.NORMAL)
        self.system_stop_btn.configure(state=ctk.DISABLED)
        self.control_stop_btn.configure(state=ctk.DISABLED)
        self.control_launch_btn.configure(state=ctk.DISABLED)
        self.align_btn.configure(state=ctk.DISABLED)
        params = (self.heartbeat, )

        self.processes['power_off_surgeon'] = {
            'process': mp.Process(target=Power_Off_H.main, daemon=True, name='power_off_surgeon', args=params),
            'last_heartbeat': time.time(),
            'start_time': time.time(),
            'params': params,
            'state': 'AWAITING',
            'pid': None,
            'target': Power_Off_H.main
        }
        self.heartbeat.put(('power_off_surgeon', 'AWAITING'))

        self.processes['power_off_diagnost'] = {
            'process': mp.Process(target=Power_Off_D.main, daemon=True, name='power_off_diagnost', args=params),
            'last_heartbeat': time.time(),
            'start_time': time.time(),
            'params': params,
            'state': 'AWAITING',
            'pid': None,
            'target': Power_Off_D.main
        }
        self.heartbeat.put(('power_off_diagnost', 'AWAITING'))

    def control_launch(self):
        sleep(1)
        if self.us_lock:
            program_lock.value = 0
            self.control_stop_btn.configure(state=tk.NORMAL)
            self.control_launch_btn.configure(state=tk.DISABLED)
        else:
            self.show_error("ППУИ запущено!")

    def control_initiate(self):
        surgeon_params = (self.surgeon_ip, True, auto, program_lock, shared_path, self.heartbeat, self.diagnost_ip)
        diagnost_params = (self.diagnost_ip, False, auto, program_lock, shared_path, self.heartbeat)

        self.processes['surgeon_control'] = {
            'process': mp.Process(target=robot_control.main, daemon=True, name='surgeon_control', args=surgeon_params),
            'last_heartbeat': time.time(),
            'start_time': time.time(),
            'params': surgeon_params,
            'state': 'AWAITING',
            'pid': None,
            'target': robot_control.main
        }
        self.heartbeat.put(("surgeon_control", "AWAITING"))
        self.processes['diagnost_control'] = {
            'process': mp.Process(target=robot_control.main, daemon=True, name='diagnost_control',
                                  args=diagnost_params),
            'last_heartbeat': time.time(),
            'start_time': time.time(),
            'params': diagnost_params,
            'state': 'AWAITING',
            'pid': None,
            'target': robot_control.main
        }
        self.heartbeat.put(("diagnost_control", "AWAITING"))

    def control_disable(self):
        self.heartbeat.put(("diagnost_control","FINISHED"))
        self.heartbeat.put(("surgeon_control", "FINISHED"))

    def control_stop(self):
        print('control stopped')
        self.control_stop_btn.configure(state=tk.DISABLED)
        self.control_launch_btn.configure(state=tk.NORMAL)
        program_lock.value = 1

    def align(self):
        print('aligned')
        self.align_d_process = subprocess.Popen(['python', 'Align_D.py'])

    def unlock(self):
        print('unlocked')
        self.unlock_h_process = subprocess.Popen(['python', 'ESTOP_RESET_D.py'])
        self.unlock_d_process = subprocess.Popen(['python', 'ESTOP_RESET_H.py'])

    def cam_d(self):
        # if not self.cam_d_state:
        #     print('Diagnost cam on')
        #     self.cam_d_state = True
        #     self.cam_d_process = subprocess.Popen(['python', 'camDiagn.py'])
        # else:
        #     self.cam_d_process.kill()
        #     print('Diagnost cam off')
        #     self.cam_d_state = False
        self.show_error("Превышен порог силового давления!")
        self.show_error("Начальная точка маршрута отсутствует")
        self.show_warning('Управление диагноста отключено')
        self.show_error("Робот в движении")
        self.show_error("Данные отсутствуют")
        self.show_error(
            "Не удаётся запустить управление хирургом\nПроверьте состояние робота, он должен быть включен и разблокирован.")
        self.show_error("ППУИ запущено!")

    def cam_h(self):
        if not self.cam_h_state:
            print('Hirurg cam on')
            self.cam_d_state = True
            self.cam_h_process = subprocess.Popen(['python', 'camHirurg.py'])
        else:
            self.cam_h_process.kill()
            print('Hirurg cam off')
            self.cam_h_state = False

    def start_route(self):
        if (program_lock.value == 0):
            program_lock.value = 1
            self.show_warning('Управление диагноста отключено')
            sleep(5)
        nsteps = int(self.num_steps_entry.get())
        SX = float(self.step_x_entry.get())
        SY = float(self.step_y_entry.get())
        SZ = float(self.step_z_entry.get())
        pause = int(self.pause_time_entry.get())
        vel = float(self.velocity_entry.get())
        us_thread = Thread(target=route_follow, args=(nsteps, [-SX, -SY, - SZ], pause, vel))
        us_thread.start()

    def start_aphi(self):
        if program_lock.value == 0:
            program_lock.value = 1
            self.show_warning('Управление диагноста отключено')
            sleep(5)
        angle = float(self.move_angle_entry.get())
        nsteps = int(self.aphi_num_steps_entry.get())
        step = float(self.aphi_step_val_entry.get())
        pause = int(self.aphi_pause_time_entry.get())
        vel = float(self.aphi_velocity_entry.get())
        aphi_thread = Thread(target=aphi, args=(angle, nsteps, step, pause, vel))
        aphi_thread.start()

    def stop_aphi(self):
        pass

    def fuck_go_back(self):
        pass

    def save_aphi(self):
        pass

    def ashido_pos(self):
        if program_lock.value == 0:
            program_lock.value = 1
            self.show_warning('Управление диагноста отключено')
            sleep(5)
        angle = float(self.move_angle_entry.get()) if self.aphi_velocity_entry.get() != "" else 0
        nsteps = int(self.aphi_num_steps_entry.get())
        step = float(self.aphi_step_val_entry.get())
        pause = int(self.aphi_pause_time_entry.get())
        vel = float(self.aphi_velocity_entry.get())
        ashido_thread = Thread(target=ashido_init, args=(nsteps, step, pause, angle))
        ashido_thread.start()

    def ashido_start(self):
        if program_lock.value == 0:
            program_lock.value = 1
            self.show_warning('Управление диагноста отключено')
            sleep(5)
        angle = float(self.move_angle_entry.get()) if self.aphi_velocity_entry.get() != "" else 0
        nsteps = int(self.aphi_num_steps_entry.get())
        step = float(self.aphi_step_val_entry.get())
        pause = int(self.aphi_pause_time_entry.get())
        vel = float(self.aphi_velocity_entry.get())
        ashido_thread = Thread(target=ashido, args=(nsteps, step, pause, vel, angle))
        ashido_thread.start()

    def save_route(self):
        if us_lock and False:
            self.show_error("Робот в движении")
            return 228
        else:
            file_path = filedialog.asksaveasfilename(defaultextension=".txt")
            if file_path:
                with open(file_path, "w") as file:
                    file.write("Step | Time,s | Coordinate | Force, N\n")
                    try:
                        for x in range(len(rob_us_data[0])):
                            file.write(f'{x} {rob_us_data[0][x]} {rob_us_data[1][x]} {rob_us_data[2][x]}\n')
                    except IndexError:
                        self.show_error("Данные отсутствуют")

    def save_d_route(self):
        print(shared_path)
        if (len(shared_path[0])):
            file_path = filedialog.asksaveasfilename(defaultextension=".txt")
            if file_path:
                with open(file_path, "w") as file:
                    for pos in shared_path[0]:
                        file.write(str(pos[0]) + "|" + str(pos[1]) + "|" + str(pos[2]) + "\n")
                shared_path[0] = []
        else:
            self.show_error("Путь отсутствует")

    def save_h_route(self):
        if (len(hirurg_path[0])):
            file_path = filedialog.asksaveasfilename(defaultextension=".txt")
            if file_path:
                with open(file_path, "w") as file:
                    for pos in hirurg_path[0]:
                        file.write(str(pos[0]) + "|" + str(pos[1]) + "|" + str(pos[2]) + "\n")
                hirurg_path[0] = []
        else:
            self.show_error("Путь отсутствует")

    def launch_h_route(self):
        file_path = filedialog.askopenfile(title="Выберите файл маршрута хирурга", filetypes=[("Text files", "*.txt")])

        pass

    def stop_routef(self):
        global stop_route
        stop_route.set()

    def return_to_start(self):
        global starting_pose
        if starting_pose:
            ctrl = RTDEControlInterface("192.168.8.3")
            recv = RTDEReceiveInterface("192.168.8.3")
            ctrl.moveL(starting_pose, velocity=0.2, acceleration=0.2)
            while ctrl.isProgramRunning():
                time.sleep(0.01)
            ctrl.disconnect()
            recv.disconnect()
        else:
            self.show_error("Начальная точка маршрута отсутствует")

    def launch_d_route(self):
        pass

    def system_monitor(self):
        diag_ctrl = RTDEControlInterface("192.168.8.3")
        diag_recv = RTDEReceiveInterface("192.168.8.3")
        hirurg_ctrl = RTDEControlInterface("192.168.8.4")
        hirurg_recv = RTDEReceiveInterface("192.168.8.4")
        
        while not self.monitor_stop_event.is_set():
            diagnost_force = diag_recv.getActualTCPForce()
            hirurg_force = hirurg_recv.getActualTCPForce()
            diagnost_pose = diag_recv.getActualTCPPose()
            hirurg_pose = hirurg_recv.getActualTCPPose()
            diagnost_joints = diag_recv.getActualQ()
            hirurg_joints = hirurg_recv.getActualQ()
            
            # Безопасное обновление UI, если элемент существует
            if hasattr(self, 'diagnost_force_data_label'):
                self.diagnost_force_data_label.config(
                    text=f'X:{diagnost_force[0]:.2f}, Y: {diagnost_force[1]:.2f}, Z: {diagnost_force[2]:.2f}')
            
            if self.telemetry_logger and getattr(self.telemetry_logger, 'enabled', False):
                self.telemetry_logger.log_data(
                    ["диагност"] + list(diagnost_pose[:3]) + list(diagnost_joints) + list(diagnost_force))
                self.telemetry_logger.log_data(
                    ["хирург"] + list(hirurg_pose[:3]) + list(hirurg_joints) + list(hirurg_force))
            time.sleep(1)
            
        diag_ctrl.disconnect()
        diag_recv.disconnect()
        hirurg_ctrl.disconnect()
        hirurg_recv.disconnect()

    def watchdog(self):
        logger.debug("Запущен цикл мониторинга сердцебиения")
        while not self.watchdog_stop_event.is_set():
            current_time = time.time()
            while not self.heartbeat.empty():
                msg = self.heartbeat.get()
                name = msg[0]
                if name not in self.processes:
                    logger.debug(f"Получено сообщение от неизвестного процесса {name}: {msg}")
                    continue

                proc = self.processes[name]
                msg_type = msg[1]

                if msg_type == "READY":
                    proc['state'] = "READY"
                    proc['last_heartbeat'] = msg[2]
                    logger.info(f"Процесс {name} (PID: {proc['pid']}) готов к работе")
                    self._update_status(name, "READY", "lightgreen")

                elif msg_type == "ALIVE":
                    prev_state = proc["state"]
                    proc["state"] = "RUNNING"
                    proc["last_heartbeat"] = msg[2]

                    if prev_state != "RUNNING" or (current_time - getattr(proc, "last_log_time", 0)) > 30:
                        uptime = current_time - proc["start_time"]
                        logger.debug(
                            f"Процесс {name} активен | PID: {proc['pid']} | Аптайм: {uptime:.1f}s | Последнее сердцебиение: {current_time - proc['last_heartbeat']:.2f}s назад")
                        proc["last_log_time"] = current_time

                    self._update_status(name, "RUNNING", "green")

                elif msg_type == "ERROR":
                    error_msg = msg[2]
                    logger.error(f"Процесс {name} (PID: {proc['pid']}) ошибка: {error_msg}")
                    self._force_restart(name, reason=f"Ошибка: {error_msg}")

                elif msg_type == "CRASH":
                    error_msg = msg[2]
                    logger.critical(f"Процесс {name} (PID: {proc['pid']}) аварийно завершился: {error_msg}")
                    self._force_restart(name, reason=f"Аварийное завершение: {error_msg}")
                elif msg_type == "AWAITING":
                    logger.info(f"Процесс {name} ожидает запуска")
                    self._start_process(name)
                elif msg_type == "FINISHED":
                    logger.info(f"Процесс {name} завершил работу без ошибок")
                    self._close_process(name)

            for name in list(self.processes.keys()):
                proc = self.processes[name]
                time_since_hb = current_time - proc["last_heartbeat"]

                if time_since_hb > self.heartbeat_timeout:
                    if not proc['process'].is_alive():
                        logger.warning(
                            f"Процесс {name} (PID: {proc['pid']}) завершился без уведомления. Последнее сердцебиение: {time_since_hb:.1f}s назад")
                        self._force_restart(name, reason="Неожиданное завершение процесса")
                    else:
                        logger.critical(
                            f"ОБНАРУЖЕНО ЗАВИСАНИЕ: процесс {name} (PID: {proc.get('pid', 'N/A')}) не отвечает "
                            f"в течение {time_since_hb:.1f}s (тайм: {self.heartbeat_timeout}s) "
                            f"Состояние: {proc['state']}"
                        )
                        self._force_restart(name, reason="Зависание процесса (таймаут сердцебиения)")

            time.sleep(1.0)

    def _close_process(self, name):
        self.processes[name]['process'].terminate()
        self.processes[name]['process'].join(timeout=3.0)
        self.processes[name] = None

    def _start_process(self, name):
        logger.info(f"Запуск процесса {name}")
        target_func = self.processes[name]['target']
        params = self.processes[name]['params']

        p = mp.Process(target=target_func, args=params, daemon=True, name=name)
        p.start()

        self.processes[name] = {
            'process': p,
            'last_heartbeat': time.time(),
            'start_time': time.time(),
            'params': params,
            'state': 'STARTING',
            'pid': p.pid,
            'target': target_func
        }

    def _update_status(self, name, state, color):
        pass

    def _schedule_restart(self, name, reason="Плановый перезапуск"):
        pass

    def _actual_restart(self, name, target_func, params, reason=""):
        self.restart_delays[name] = 1.0

        logger.info(f"Перезапуск процесса {name} | Причина: {reason}")

        p = mp.Process(target=target_func, args=params, daemon=True, name=name)
        p.start()

        self.processes[name] = {
            'process': p,
            'last_heartbeat': time.time(),
            'start_time': time.time(),
            'params': params,
            'state': 'STARTING',
            'pid': p.pid,
            'target': target_func
        }
        self._update_status(name, "RESTARTING", "orange")

    def _force_restart(self, name, reason="Неизвестная причина"):
        proc = self.processes[name]
        if not proc:
            logger.warning(f"Попытка перезапуска несуществующего процесса {name}")
            return

        p = proc["process"]
        pid = proc.get('pid', 'N/A')

        logger.warning(f"Начало принудительного перезапуска {name} (PID: {pid}) | Причина: {reason}")

        if p.is_alive():
            logger.debug(f"Отправка SIGTERM процессу {name} (PID: {pid})")
            p.terminate()
            p.join(timeout=3.0)

        if p.is_alive():
            logger.error(f"Процесс {name} (PID: {pid}) не отвечает на SIGTERM, отправка SIGKILL")
            p.kill()
            p.join(timeout=2.0)

        if p.is_alive():
            logger.critical(f"НЕВОЗМОЖНО ЗАВЕРШИТЬ процесс {name} даже после SIGKILL!")
        else:
            logger.info(f"Процесс {name} (PID: {pid}) успешно завершён")

        if name in self.processes:
            del self.processes[name]

        delay = self.restart_delays.get(name, 1.0)
        self.restart_delays[name] = min(delay * 2, 30.0)

        logger.info(f"Планирование перезапуска {name} через {delay:.1f}с (экспоненциальная задержка)")

        threading.Timer(delay, self._actual_restart, args=(name, proc['target'], proc['params'], reason)).start()

    def kill_all(self):
        pass

    def show_error(self, msg):
        mb(title="ОШИБКА!", message=msg, icon="cancel")

    def show_warning(self, msg):
        mb(title="ВНИМАНИЕ!", message=msg, icon="warning")


def on_closing():
    logger.info(f"Получен сигнал закрытия приложения")
    for name in list(app.processes.keys()):
        app.processes[name]["process"].kill()
        app.processes[name]["process"].join(timeout=2.0)
    logger.info("Все процессы остановлены. Завершение работы.")
    app.destroy()


if __name__ == '__main__':
    logger = setup_status_logging()
    mp.freeze_support()
    proxy = mp.Manager()
    shared_path = proxy.list()
    shared_path.append([])
    hirurg_path = proxy.list()
    hirurg_path.append([])
    # root = tk.Tk()
    app = RobotControlUI()
    app.protocol("WM_DELETE_WINDOW", on_closing)
    app.mainloop()
