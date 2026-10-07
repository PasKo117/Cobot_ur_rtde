"""Independent RTSP readers and serialized, time-limited Hikvision PTZ commands.

No robot imports and no network activity at import time.
"""
import os
import threading
import time
import xml.etree.ElementTree as ET

STALE_SECONDS = 3.0
PULSE_MS = 350
LEASE_SECONDS = 0.45
REPEAT_SECONDS = 0.22
ZERO = (0, 0, 0)


class VideoReader:
    def __init__(self, config, cv2=None, retry_seconds=5):
        if cv2 is None:
            os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;tcp'
            os.environ.setdefault('OPENCV_LOG_LEVEL', 'SILENT')
            import cv2
        self.cv2 = cv2
        self.config = config
        self.retry_seconds = retry_seconds
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.frame = None
        self.stamp = 0.0
        self.sequence = 0
        self.status = 'Подключение…'
        self.thread = threading.Thread(target=self._run, daemon=True, name=f'video-{config.host}')

    def start(self):
        self.thread.start()

    def close(self):
        self.stop_event.set()

    def snapshot(self):
        with self.lock:
            return self.frame, self.stamp, self.sequence, self.status

    def _status(self, value):
        with self.lock:
            self.frame = None
            self.status = value

    def _run(self):
        if not self.config.password:
            self._status('не удалось подключить — не задан пароль')
            return
        cv2 = self.cv2
        streams = list(self.config.streams)
        while not self.stop_event.is_set():
            for stream in streams[:]:
                if self.stop_event.is_set():
                    return
                cap = None
                try:
                    self._status(f'Подключение… поток {stream}')
                    cap = cv2.VideoCapture(self.config.rtsp_url(stream), cv2.CAP_FFMPEG,
                                          [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000,
                                           cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2500])
                    if not cap.isOpened():
                        self._status('не удалось подключить')
                        continue
                    while not self.stop_event.is_set():
                        ok, frame = cap.read()
                        if not ok or frame is None or not frame.size:
                            self._status('не удалось подключить — поток прерван')
                            break
                        with self.lock:
                            self.frame = frame
                            self.stamp = time.monotonic()
                            self.sequence += 1
                            self.status = f'В эфире • поток {stream}'
                        streams = [stream] + [s for s in self.config.streams if s != stream]
                except Exception as exc:
                    self._status(f'не удалось подключить — {type(exc).__name__}')
                finally:
                    if cap is not None:
                        try:
                            cap.release()
                        except Exception:
                            pass
            if not self.stop_event.is_set():
                self._status(f'не удалось подключить — повтор через {self.retry_seconds} с')
                self.stop_event.wait(self.retry_seconds)


class PTZError(Exception):
    """An intentionally credential-free message suitable for the UI."""


class HikvisionPTZ:
    def __init__(self, config, session=None):
        import requests
        from requests.auth import HTTPDigestAuth
        self.requests = requests
        self.config = config
        self.session = session or requests.Session()
        self.session.trust_env = False  # Never send LAN camera commands through an HTTP proxy.
        self.session.auth = HTTPDigestAuth(config.username, config.password)

    def _put(self, action, vector, duration=None):
        if not self.config.password:
            raise PTZError('PTZ: не задан пароль')
        pan, tilt, zoom = vector
        if not all(type(v) is int and -100 <= v <= 100 for v in vector):
            raise ValueError('PTZ speed must be an integer in -100..100')
        pan *= -1 if self.config.invert_pan else 1
        tilt *= -1 if self.config.invert_tilt else 1
        root = ET.Element('PTZData', version='2.0', xmlns='http://www.hikvision.com/ver20/XMLSchema')
        for key, value in [('pan', pan), ('tilt', tilt), ('zoom', zoom)]:
            ET.SubElement(root, key).text = str(value)
        if duration is not None:
            ET.SubElement(ET.SubElement(root, 'Momentary'), 'duration').text = str(duration)
        try:
            response = self.session.put(self.config.ptz_url + '/' + action,
                                        data=ET.tostring(root, encoding='utf-8'),
                                        headers={'Content-Type': 'application/xml'},
                                        timeout=(0.8, 0.8), allow_redirects=False)
        except self.requests.RequestException:
            raise PTZError('PTZ: нет связи или таймаут') from None
        try:
            if response.status_code in (401, 403):
                raise PTZError(f'PTZ: HTTP {response.status_code} — проверьте пароль и права PTZ')
            if response.status_code in (404, 405, 501):
                raise PTZError(f'PTZ: HTTP {response.status_code} — ISAPI/{action} не поддерживается')
            if not 200 <= response.status_code < 300:
                raise PTZError(f'PTZ: HTTP {response.status_code}')
            if response.content:
                try:
                    body = ET.fromstring(response.content)
                except ET.ParseError:
                    raise PTZError('PTZ: вместо XML получен неизвестный ответ') from None
                codes = [(n.text or '').strip() for n in body.iter() if n.tag.split('}')[-1] == 'statusCode']
                if not codes or any(c not in ('0', '1') for c in codes):
                    code = codes[0] if codes and codes[0].isdigit() else '?'
                    raise PTZError(f'PTZ: камера отклонила {action} (код {code})')
        finally:
            response.close()

    def move(self, vector):
        # Each command expires on the camera. No fallback to unbounded continuous movement.
        self._put('momentary', vector, PULSE_MS)

    def stop(self):
        self._put('continuous', ZERO)

    def close(self):
        self.session.close()


class PTZWorker:
    """Latest intent only, no movement queue. A GUI heartbeat renews a short lease.

    In-flight moves cannot be cancelled at HTTP level. A stop is serialized after
    them, and the camera-side pulse limit also bounds that movement.
    """
    def __init__(self, client):
        self.client = client
        self.condition = threading.Condition()
        self.vector = ZERO
        self.deadline = 0.0
        self.generation = 0
        self.closed = False
        self.stop_pending = False
        self.status = 'PTZ: готово к команде'
        self.failed = False
        self.thread = threading.Thread(target=self._run, daemon=True, name='camera-ptz')
        self.thread.start()

    def begin(self, vector):
        with self.condition:
            if self.closed:
                return None
            self.generation += 1
            self.vector = vector
            self.failed = False
            self.deadline = time.monotonic() + LEASE_SECONDS
            self.condition.notify()
            return self.generation

    def renew(self, token):
        with self.condition:
            if not self.closed and token == self.generation and self.vector != ZERO:
                self.deadline = time.monotonic() + LEASE_SECONDS

    def stop(self):
        with self.condition:
            self.generation += 1
            self.vector = ZERO
            self.stop_pending = True
            self.condition.notify()

    def close(self):
        with self.condition:
            self.closed = True
            self.vector = ZERO
            self.stop_pending = True
            self.generation += 1
            self.condition.notify()

    def get_status(self):
        with self.condition:
            return self.status

    def _run(self):
        last_move = 0.0
        moving = False
        try:
            while True:
                with self.condition:
                    now = time.monotonic()
                    if self.vector != ZERO and now >= self.deadline:
                        self.vector = ZERO
                        self.generation += 1
                        self.stop_pending = True
                    if self.stop_pending:
                        action, vector = 'stop', ZERO
                        self.stop_pending = False
                    elif self.closed:
                        break
                    elif self.vector != ZERO and now - last_move >= REPEAT_SECONDS:
                        action, vector = 'move', self.vector
                    else:
                        self.condition.wait(0.04)
                        continue
                    generation = self.generation
                try:
                    if action == 'stop':
                        self.client.stop()
                        moving = False
                    else:
                        # Treat even a timed-out request as potentially accepted by the camera.
                        moving = True
                        self.client.move(vector)
                        last_move = time.monotonic()
                    with self.condition:
                        if generation == self.generation and not self.failed:
                            self.status = 'PTZ: остановлено' if action == 'stop' else 'PTZ: движение'
                except Exception as exc:
                    with self.condition:
                        self.status = str(exc) if isinstance(exc, PTZError) else f'PTZ: {type(exc).__name__}'
                        self.failed = True
                        # Reject heartbeat renewal of a failed command; a new press is required.
                        self.vector = ZERO
                        self.generation += 1
                        if action == 'move':
                            self.stop_pending = True
                    if action == 'stop':
                        moving = False  # Pulse expiry, not an acknowledged stop, bounds movement.
        finally:
            if moving:
                try:
                    self.client.stop()
                except Exception:
                    pass
            self.client.close()
