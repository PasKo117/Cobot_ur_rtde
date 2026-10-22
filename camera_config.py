"""Camera settings. Local credentials are excluded from git."""
from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import sys
from urllib.parse import quote


@dataclass(frozen=True)
class CameraConfig:
    name: str
    host: str
    username: str = 'admin'
    password: str = field(default='', repr=False)
    rtsp_port: int = 554
    http_port: int = 80
    scheme: str = 'http'
    channel: int = 1
    streams: tuple = (102, 101)
    invert_pan: bool = False
    invert_tilt: bool = False

    def rtsp_url(self, stream):
        return (f'rtsp://{quote(self.username, safe="")}:{quote(self.password, safe="")}@'
                f'{self.host}:{self.rtsp_port}/Streaming/Channels/{stream}')

    @property
    def ptz_url(self):
        return f'{self.scheme}://{self.host}:{self.http_port}/ISAPI/PTZCtrl/channels/{self.channel}'


def config_path():
    folder = Path(sys.executable).parent if getattr(sys, 'frozen', False) else Path(__file__).parent
    return folder / 'cameras.local.json'


def load_cameras(path=None):
    path = Path(path) if path else config_path()
    raw = json.loads(path.read_text(encoding='utf-8-sig')) if path.exists() else {}
    defaults = raw.get('defaults', {})
    entries = raw.get('cameras', [{'name': f'Камера {i + 1}', 'host': f'192.168.8.{64 + i}'}
                                for i in range(3)])
    if not isinstance(entries, list) or not 1 <= len(entries) <= 3:
        raise ValueError('В cameras.local.json требуется от 1 до 3 камер.')
    result = []
    for entry in entries:
        values = {**defaults, **entry}
        for env, key in [('COBOT_CAMERA_USER', 'username'), ('COBOT_CAMERA_PASSWORD', 'password'),
                         ('HIKVISION_USERNAME', 'username'), ('HIKVISION_PASSWORD', 'password')]:
            if env in os.environ:
                values[key] = os.environ[env]
        cam = CameraConfig(**values)
        # Restrict hosts to IP/host names, so credentials are never sent to a URL supplied as a host.
        if not cam.host or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.-' for c in cam.host):
            raise ValueError('host должен содержать только IP-адрес или имя камеры.')
        if cam.scheme not in ('http', 'https'):
            raise ValueError('scheme: только http или https.')
        if not all(type(p) is int and 1 <= p <= 65535 for p in (cam.rtsp_port, cam.http_port)):
            raise ValueError('Порт камеры должен быть целым числом от 1 до 65535.')
        if type(cam.channel) is not int or cam.channel < 1:
            raise ValueError('Некорректный PTZ-канал.')
        if not cam.streams or not all(type(s) is int and s > 0 for s in cam.streams):
            raise ValueError('Некорректные номера видеопотоков.')
        if not isinstance(cam.password, str) or not isinstance(cam.username, str):
            raise ValueError('Имя пользователя и пароль должны быть строками.')
        result.append(cam)
    return result
