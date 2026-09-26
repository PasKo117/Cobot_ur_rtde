"""GUI for the two UR robots. RTDE access is owned by robot_control.py."""
import csv
import math
import multiprocessing as mp
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from queue import Empty
from threading import Thread
from tkinter import filedialog

import customtkinter as ctk
from CTkMessagebox import CTkMessagebox as mb

import robot_control
import route_io
from laser_sensor_lib import LaserSensorClient

ctk.set_appearance_mode('Dark')
ctk.set_default_color_theme('blue')


class CSVLogger:
    def __init__(self, filename='logs/telemetry_log.csv'):
        self.filename = Path(filename)
        self.filename.parent.mkdir(parents=True, exist_ok=True)
        self.enabled = False
        if not self.filename.exists():
            with self.filename.open('w', newline='', encoding='utf-8') as f:
                csv.writer(f).writerow(['timestamp', 'robot', 'X', 'Y', 'Z',
                                        'J1', 'J2', 'J3', 'J4', 'J5', 'J6',
                                        'FX', 'FY', 'FZ', 'MX', 'MY', 'MZ'])

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def log_data(self, name, sample):
        if self.enabled:
            with self.filename.open('a', newline='', encoding='utf-8') as f:
                csv.writer(f).writerow([datetime.now().isoformat(timespec='milliseconds'), name,
                                        *sample['pose'][:3], *sample['joints'], *sample['force']])


def dashboard_command(ip, command):
    with socket.create_connection((ip, 29999), timeout=5) as connection:
        connection.settimeout(5)
        connection.recv(4096)  # Dashboard sends a greeting before receiving commands.
        connection.sendall((command + '\n').encode('ascii'))
        answer = connection.recv(4096).decode(errors='replace').strip()
        if not answer or 'failed' in answer.lower() or 'not allowed' in answer.lower():
            raise RuntimeError(f'{ip}: {command}: {answer}')
        return answer


class RobotControlUI(ctk.CTk):
    def __init__(self):
        super().__init__()

        # Конфигурация главного окна
        self.title('ЕППУИХ - Система управления манипуляторами')
        self.geometry("1100x750")
        self.minsize(900, 650)

        # Переменные системы
        self.heartbeat = mp.Queue()
        self.tel_logging_var = ctk.BooleanVar(value=False)
        self.mode_var = ctk.StringVar(value='Режим: асинхронный')
        self.path_var = ctk.StringVar(value='Точек: хирург 0 · диагност 0')
        self.action_status_var = ctk.StringVar(value='Ожидание подключения')

        self.monitor_stop_event = threading.Event()

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

        # Инициализация лазерных датчиков
        self.laser_client = LaserSensorClient(
            pi_ip='192.168.8.37',
            pi_user='pi',
            pi_pass='rasprobot1'
        )
        self.laser_connected = False

        # Создание интерфейса
        self.create_tabs()

        self.commands = mp.Queue()
        self.messages = mp.Queue()
        self.stop_motion = mp.Event()
        self.shutdown = mp.Event()
        self.worker = None
        self.busy = False
        self.manual = False
        self.route_data = []
        self.aphi_data = []
        self.paths = {'diagnost': [], 'surgeon': []}
        self.pending_path_save = None
        self.start_pose = None
        self.camera_processes = {}
        self.telemetry_logger = CSVLogger()
        self.after(100, self.poll_worker)
        self.monitor = Thread(target=self.system_monitor, daemon=True)
        self.monitor.start()

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
                                                 command=self.control_disable, state="disabled", fg_color="#E74C3C",
                                                 hover_color="#C0392B")
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
        ctk.CTkLabel(left_frame, textvariable=self.mode_var).grid(column=0, row=7, columnspan=2, pady=3)
        ctk.CTkLabel(left_frame, textvariable=self.action_status_var, wraplength=370).grid(
            column=0, row=8, columnspan=2, pady=3)

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
        ctk.CTkLabel(right_frame, textvariable=self.path_var).grid(column=0, row=5, pady=5)

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
        self.num_steps_entry = ctk.CTkEntry(params_frame, width=100, placeholder_text='30')
        self.num_steps_entry.grid(column=1, row=1, sticky="w", padx=5, pady=5)

        # Величина шага
        ctk.CTkLabel(params_frame, text="Величина шага (м):").grid(column=0, row=2, sticky="e", padx=10, pady=5)

        ctk.CTkLabel(params_frame, text="X:").grid(column=2, row=2, padx=5)
        self.step_x_entry = ctk.CTkEntry(params_frame, width=60, placeholder_text='0.005')
        self.step_x_entry.grid(column=3, row=2, padx=2, pady=5)

        ctk.CTkLabel(params_frame, text="Y:").grid(column=4, row=2, padx=5)
        self.step_y_entry = ctk.CTkEntry(params_frame, width=60, placeholder_text='0.005')
        self.step_y_entry.grid(column=5, row=2, padx=2, pady=5)

        ctk.CTkLabel(params_frame, text="Z:").grid(column=6, row=2, padx=5)
        self.step_z_entry = ctk.CTkEntry(params_frame, width=60, placeholder_text='0.005')
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
        ctk.CTkLabel(frame, text='Используются параметры вкладки «Воздействие».\n'
                     'Доступно только осевое продвижение хирурга; угол должен быть 0.').grid(column=0, row=3, pady=10)

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
        else:
            self.telemetry_logger.disable()
        self.telemetry_logging_status_label.configure(
            text='Логирование: ВКЛЮЧЕНО' if self.tel_logging_var.get() else 'Логирование: ВЫКЛЮЧЕНО',
            text_color='green' if self.tel_logging_var.get() else 'red')

    def system_launch(self):
        self.system_launch_btn.configure(state='disabled')
        def power_on():
            try:
                for ip in (self.surgeon_ip, self.diagnost_ip):
                    dashboard_command(ip, 'power on')
                time.sleep(5)
                for ip in (self.surgeon_ip, self.diagnost_ip):
                    dashboard_command(ip, 'brake release')
                self.messages.put(('INFO', {'text': 'Роботы включены; проверьте состояние на пультах'}))
            except Exception as exc:
                self.messages.put(('ERROR', {'text': f'Включение питания: {exc}'}))
            finally:
                self.messages.put(('POWER_ON_DONE', {}))
        threading.Thread(target=power_on, daemon=True).start()

    def system_stop(self):
        self.control_disable()
        def power_off():
            for ip in (self.surgeon_ip, self.diagnost_ip):
                try:
                    dashboard_command(ip, 'power off')
                except Exception as exc:
                    self.messages.put(('ERROR', {'text': f'Выключение: {exc}'}))
            self.messages.put(('POWER_OFF_DONE', {}))
        threading.Thread(target=power_off, daemon=True).start()

    def control_initiate(self):
        if self.worker and self.worker.is_alive():
            return
        self.shutdown.clear()
        self.stop_motion.clear()
        self.worker = mp.Process(name='robot_worker', target=robot_control.main,
                                 args=(self.commands, self.messages, self.stop_motion, self.shutdown,
                                       self.heartbeat, self.diagnost_ip, self.surgeon_ip))
        self.worker.start()
        self.control_initiate_btn.configure(state='disabled')

    def control_disable(self):
        self.stop_motion.set()
        self.shutdown.set()
        self.manual = False
        self.busy = False
        self.pending_path_save = None
        self.control_launch_btn.configure(state='disabled')
        self.control_stop_btn.configure(state='disabled')
        self.control_disable_btn.configure(state='disabled')
        if self.worker:
            self.worker.join(timeout=2)
            if self.worker.is_alive():
                self.worker.terminate()
                self.worker.join(timeout=2)
            if self.worker.is_alive():
                self.worker.kill()
                self.worker.join(timeout=2)
            if self.worker.is_alive():
                self.show_error('Процесс управления не остановился. Повторная инициализация запрещена.')
                return
            self.worker = None
        # Discard commands left over from a stopped worker.
        while True:
            try:
                self.commands.get_nowait()
            except Empty:
                break
        self.control_initiate_btn.configure(state='normal')

    def control_launch(self):
        self.submit('manual_start')

    def control_stop(self):
        self.stop_motion.set()
        self.commands.put(('manual_stop', {}))
        self.manual = False
        self.control_launch_btn.configure(state='normal')
        self.control_stop_btn.configure(state='disabled')
        self.mode_var.set('Режим: асинхронный')

    def align(self):
        self.submit('align', robot='diagnost')

    def unlock(self):
        def task():
            for ip in (self.diagnost_ip, self.surgeon_ip):
                try:
                    dashboard_command(ip, 'unlock protective stop')
                except Exception as exc:
                    self.messages.put(('ERROR', {'text': f'Разблокировка: {exc}'}))
        threading.Thread(target=task, daemon=True).start()

    def camera(self, name, script):
        process = self.camera_processes.get(name)
        if process and process.poll() is None:
            process.terminate()
            process.wait(timeout=2)
            del self.camera_processes[name]
        else:
            self.camera_processes[name] = subprocess.Popen([sys.executable, str(Path(__file__).with_name(script))])

    def cam_d(self):
        self.camera('diagnost', 'camDiagn.py')

    def cam_h(self):
        self.camera('surgeon', 'camHirurg.py')

    def submit(self, action, **args):
        if not self.worker or not self.worker.is_alive():
            self.show_error('Сначала инициализируйте контроллеры')
            return False
        if self.busy or self.manual and action != 'manual_stop':
            self.show_error('Сначала остановите текущее движение')
            return False
        self.stop_motion.clear()
        self.busy = True
        self.commands.put((action, args))
        return True

    def _parameters(self, operation=False):
        if operation:
            angle = robot_control.valid_float(self.move_angle_entry.get(), 'Угол', -180, 180, True)
            if angle != 0:
                raise ValueError('Движение под углом требует проверенной калибровки; используйте угол 0')
            count = robot_control.valid_int(self.aphi_num_steps_entry.get(), 'Количество шагов')
            step = robot_control.valid_float(self.aphi_step_val_entry.get(), 'Шаг', 0, 0.05)
            pause = robot_control.valid_float(self.aphi_pause_time_entry.get(), 'Пауза', 0, 600, True)
            speed = robot_control.valid_float(self.aphi_velocity_entry.get(), 'Скорость', 0, 0.1)
            return {'count': count, 'step': step, 'pause': pause, 'speed': speed}
        count = robot_control.valid_int(self.num_steps_entry.get(), 'Количество шагов')
        steps = [-robot_control.valid_float(entry.get(), axis, -0.05, 0.05, True)
                 for entry, axis in ((self.step_x_entry, 'X'), (self.step_y_entry, 'Y'),
                                     (self.step_z_entry, 'Z'))]
        if not any(steps):
            raise ValueError('Укажите хотя бы один ненулевой шаг')
        pause = robot_control.valid_float(self.pause_time_entry.get(), 'Пауза', 0, 600, True)
        speed = robot_control.valid_float(self.velocity_entry.get(), 'Скорость', 0, 0.1)
        return {'count': count, 'steps': steps, 'pause': pause, 'speed': speed}

    def start_route(self):
        try:
            self.submit('route', **self._parameters())
        except ValueError as exc:
            self.show_error(str(exc))

    def start_aphi(self):
        try:
            self.submit('aphi', **self._parameters(True))
        except ValueError as exc:
            self.show_error(str(exc))

    def stop_aphi(self):
        self.stop_routef()

    def fuck_go_back(self):
        self.submit('aphi_return')

    def save_aphi(self):
        self._save_poses(self.aphi_data)

    def ashido_pos(self):
        self.submit('ashido_pos')

    def ashido_start(self):
        try:
            self.submit('ashido_start', **self._parameters(True))
        except ValueError as exc:
            self.show_error(str(exc))

    def save_route(self):
        if not self.route_data:
            self.show_error('Данные отсутствуют')
            return
        path = filedialog.asksaveasfilename(defaultextension='.csv', filetypes=[('CSV', '*.csv')])
        if path:
            with open(path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['time_s', 'x', 'y', 'z', 'rx', 'ry', 'rz',
                                 'fx', 'fy', 'fz', 'mx', 'my', 'mz'])
                for elapsed, pose, force in self.route_data:
                    writer.writerow([elapsed, *pose, *force])

    def _save_poses(self, poses):
        if not poses:
            self.show_error('Путь отсутствует')
            return
        path = filedialog.asksaveasfilename(defaultextension='.csv', filetypes=[('CSV', '*.csv')])
        if path:
            try:
                count = route_io.write_poses(path, poses)
            except (OSError, ValueError) as exc:
                self.show_error(f'Не удалось сохранить маршрут: {exc}')
            else:
                self.action_status_var.set(f'Сохранено {count} точек: {Path(path).name}')

    def _save_robot_route(self, name):
        poses = [pose[:] for pose in self.paths[name]]
        if not poses:
            self.show_error('Путь отсутствует')
        elif self.busy:
            self.show_error('Дождитесь остановки текущего движения')
        elif self.manual:
            self.pending_path_save = poses
            self.control_stop()
        else:
            self._save_poses(poses)

    def save_d_route(self):
        self._save_robot_route('diagnost')

    def save_h_route(self):
        self._save_robot_route('surgeon')

    def _load_route(self, name):
        path = filedialog.askopenfilename(filetypes=[('Маршруты', '*.csv *.txt'), ('Все файлы', '*.*')])
        if not path:
            return
        try:
            poses = route_io.read_poses(path)
            if self.submit('replay', robot=name, poses=poses):
                self.action_status_var.set(f'Воспроизведение: {name}, {len(poses)} точек')
        except (ValueError, OSError) as exc:
            self.show_error(str(exc))

    def launch_h_route(self):
        self._load_route('surgeon')

    def launch_d_route(self):
        self._load_route('diagnost')

    def stop_routef(self):
        self.stop_motion.set()

    def return_to_start(self):
        self.submit('return_start')

    def system_monitor(self):
        try:
            self.laser_connected = self.laser_client.connect()
            if self.laser_connected:
                self.laser_client.start_stream()
            while not self.monitor_stop_event.wait(1):
                if self.laser_connected:
                    values = self.laser_client.get_sensor_values()
                    self.messages.put(('LASER', {'values': values}))
        except Exception as exc:
            self.messages.put(('ERROR', {'text': f'Лазерные датчики: {exc}'}))
        finally:
            self.laser_client.disconnect()

    def poll_worker(self):
        try:
            while True:
                kind, data = self.messages.get_nowait()
                if kind == 'TELEMETRY':
                    for name, sample in data['robots'].items():
                        key = 'diag' if name == 'diagnost' else 'hir'
                        self.telemetry_vars[key + '_force'].set(f"{sample['force'][2]:.2f}")
                        self.telemetry_logger.log_data(name, sample)
                elif kind == 'LASER':
                    values = data['values']
                    for key, sensor in (('diag', 1), ('hir', 2)):
                        val = values.get(f'sensor_{sensor}')
                        self.telemetry_vars[key + '_las'].set(f'{val:.2f}' if val is not None else '—')
                        self.telemetry_vars[key + '_status'].set('Подкл' if values.get(f'status_{sensor}') == 'ok' else 'Откл')
                elif kind == 'PATH':
                    self.paths[data['robot']] = data['poses']
                    self.path_var.set(f"Точек: хирург {len(self.paths['surgeon'])} · "
                                      f"диагност {len(self.paths['diagnost'])}")
                elif kind == 'MODE':
                    self.mode_var.set('Режим: синхронный' if data['mode'] == 'sync'
                                      else 'Режим: асинхронный')
                elif kind == 'MANUAL_PAUSED':
                    self.manual = False
                    self.control_launch_btn.configure(state='normal')
                    self.control_stop_btn.configure(state='disabled')
                    self.mode_var.set('Режим: асинхронный')
                elif kind == 'SAVE_REQUEST':
                    self.after(0, lambda poses=data['poses']: self._save_poses(poses))
                elif kind == 'ROUTE':
                    self.route_data, self.start_pose = data['rows'], data['start']
                elif kind == 'APHI':
                    self.aphi_data = data['poses']
                elif kind == 'POWER_ON_DONE':
                    self.system_stop_btn.configure(state='normal')
                    self.system_launch_btn.configure(state='normal')
                elif kind == 'POWER_OFF_DONE':
                    self.system_stop_btn.configure(state='disabled')
                elif kind in ('DONE', 'STOPPED'):
                    self.busy = False
                    if data.get('action') == 'manual_start' and kind == 'DONE':
                        self.manual = True
                        self.control_stop_btn.configure(state='normal')
                        self.control_launch_btn.configure(state='disabled')
                    elif data.get('action') == 'manual_stop':
                        self.manual = False
                        self.control_launch_btn.configure(state='normal')
                        self.control_stop_btn.configure(state='disabled')
                        if self.pending_path_save is not None:
                            poses = self.pending_path_save
                            self.pending_path_save = None
                            self.after(0, lambda poses=poses: self._save_poses(poses))
                elif kind == 'ERROR':
                    self.busy = False
                    self.action_status_var.set(data['text'])
                    self.show_error(data['text'])
                elif kind == 'INFO':
                    self.action_status_var.set(data['text'])
                    print(data['text'])
        except Empty:
            pass
        if self.worker and not self.worker.is_alive():
            self.control_disable()
            self.show_error('Процесс управления завершился. Проверьте связь и состояние роботов.')
        try:
            while True:
                name, state, *_ = self.heartbeat.get_nowait()
                if name == 'robot_worker' and state == 'READY':
                    self.control_disable_btn.configure(state='normal')
                    self.control_launch_btn.configure(state='normal')
                    self.align_btn.configure(state='normal')
        except Empty:
            pass
        self.after(100, self.poll_worker)

    def show_error(self, msg):
        mb(title='ОШИБКА!', message=msg, icon='cancel')

    def show_warning(self, msg):
        mb(title='ВНИМАНИЕ!', message=msg, icon='warning')

    def close(self):
        self.monitor_stop_event.set()
        self.control_disable()
        for proc in self.camera_processes.values():
            if proc.poll() is None:
                proc.terminate()
        self.destroy()


if __name__ == '__main__':
    mp.freeze_support()
    app = RobotControlUI()
    app.protocol('WM_DELETE_WINDOW', app.close)
    app.mainloop()
