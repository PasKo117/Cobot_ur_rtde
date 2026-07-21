"""
Библиотека для работы с лазерными датчиками SICK OD Mini Pro
Без вывода в терминал - только возврат данных
"""
import socket
import json
import threading
import time
from datetime import datetime


class LaserSensorClient:
    """Клиент для подключения к лазерным датчикам через Raspberry Pi"""

    def __init__(self, pi_ip='192.168.8.37', pi_user='pi', pi_pass='rasprobot1', tcp_port=5000):
        self.pi_ip = pi_ip
        self.tcp_port = tcp_port
        self.sock = None
        self.connected = False
        self.streaming = False
        self.last_stream_data = {
            1: {'value': None, 'ts': 0, 'status': 'disconnected'},
            2: {'value': None, 'ts': 0, 'status': 'disconnected'}
        }
        self.stream_lock = threading.Lock()
        self.stream_thread = None
        self.running = True

    def connect(self, max_attempts=3):
        """Подключение к серверу на Raspberry Pi"""
        for attempt in range(max_attempts):
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(2)
                self.sock.connect((self.pi_ip, self.tcp_port))
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock.settimeout(None)
                self.connected = True

                # Запуск потока для приёма стрим-данных
                self.stream_thread = threading.Thread(target=self._stream_listener, daemon=True)
                self.stream_thread.start()
                return True
            except Exception as e:
                if attempt < max_attempts - 1:
                    time.sleep(1)
                if self.sock:
                    try:
                        self.sock.close()
                    except:
                        pass
        return False

    def _stream_listener(self):
        """Фоновый поток для получения стрим-данных"""
        buffer = ""
        while self.running and self.connected:
            try:
                self.sock.settimeout(0.4)
                chunk = self.sock.recv(4096).decode('utf-8', errors='ignore')
                if not chunk:
                    time.sleep(0.05)
                    continue
                buffer += chunk
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    if not line.strip():
                        continue
                    try:
                        msg = json.loads(line.strip())
                        if msg.get('type') == 'stream':
                            sid = msg.get('sensor_id', 1)
                            with self.stream_lock:
                                self.last_stream_data[sid] = {
                                    'value': msg.get('value'),
                                    'ts': time.time(),
                                    'status': 'ok'
                                }
                    except json.JSONDecodeError:
                        pass
            except socket.timeout:
                pass
            except (ConnectionResetError, BrokenPipeError, OSError):
                break
            except Exception as e:
                break
            time.sleep(0.01)

    def _send_command(self, action, sensor_id=None, **kw):
        """Отправка команды серверу"""
        if not self.sock:
            return None
        try:
            payload = {"action": action, **kw}
            if sensor_id is not None:
                payload['sensor_id'] = sensor_id
            req = json.dumps(payload).encode() + b"\n"
            self.sock.sendall(req)

            buffer = ""
            self.sock.settimeout(1.5)
            while True:
                chunk = self.sock.recv(4096).decode('utf-8', errors='ignore')
                if not chunk:
                    break
                buffer += chunk
                if "\n" in buffer:
                    line = buffer.split("\n")[0]
                    self.sock.settimeout(None)
                    return json.loads(line.strip())
        except:
            return None
        return None

    def get_sensor_values(self):
        """
        Получить текущие значения с обоих датчиков
        Возвращает: dict {sensor_1: value_mm, sensor_2: value_mm}
        """
        with self.stream_lock:
            val1 = self.last_stream_data[1]['value']
            val2 = self.last_stream_data[2]['value']
            ts1 = self.last_stream_data[1]['ts']
            ts2 = self.last_stream_data[2]['ts']

        # Если данные старые (>1 сек), запрашиваем напрямую
        current_time = time.time()
        if val1 is None or (current_time - ts1) > 1.0:
            r = self._send_command('measure', sensor_id=1)
            if r and r.get('status') == 'ok':
                val1 = r.get('data')
                with self.stream_lock:
                    self.last_stream_data[1] = {'value': val1, 'ts': current_time, 'status': 'ok'}
            else:
                with self.stream_lock:
                    self.last_stream_data[1]['status'] = 'error'

        if val2 is None or (current_time - ts2) > 1.0:
            r = self._send_command('measure', sensor_id=2)
            if r and r.get('status') == 'ok':
                val2 = r.get('data')
                with self.stream_lock:
                    self.last_stream_data[2] = {'value': val2, 'ts': current_time, 'status': 'ok'}
            else:
                with self.stream_lock:
                    self.last_stream_data[2]['status'] = 'error'

        return {
            'sensor_1': val1 if val1 is not None else 0.0,
            'sensor_2': val2 if val2 is not None else 0.0,
            'status_1': self.last_stream_data[1]['status'],
            'status_2': self.last_stream_data[2]['status']
        }

    def start_stream(self):
        """Запустить стриминг данных"""
        if not self.streaming:
            r = self._send_command('stream_start')
            if r and r.get('status') == 'ok':
                self.streaming = True
                return True
        return False

    def stop_stream(self):
        """Остановить стриминг"""
        if self.streaming:
            r = self._send_command('stream_stop')
            self.streaming = False
            return r and r.get('status') == 'ok'
        return True

    def disconnect(self):
        """Отключиться"""
        self.running = False
        self.stop_stream()
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None
        self.connected = False