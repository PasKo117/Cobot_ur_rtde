"""Offline tests. No connection to cameras, robots, or laser sensors."""
import ast
import contextlib
from dataclasses import replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

from camera_config import CameraConfig, load_cameras
from camera_backend import HikvisionPTZ, PTZError, PTZWorker, VideoReader
from camera_window import CameraPanel, CameraWindow

CONFIG = CameraConfig('Test', '127.0.0.1', password='test-secret')


def eventually(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError('Condition was not satisfied before timeout')


class CameraTests(unittest.TestCase):
    def test_config_relative_to_module_and_credentials_encoding(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'cameras.local.json'
            path.write_text(json.dumps({'defaults': {'password': 'a:@/# ?'}, 'cameras': [
                {'name': 'One', 'host': '192.168.8.64'}]}))
            with patch.dict('os.environ', {}, clear=True):
                camera = load_cameras(path)[0]
            self.assertIn('a%3A%40%2F%23%20%3F', camera.rtsp_url(102))
            self.assertNotIn('a:@/# ?', repr(camera))
            with patch.dict('os.environ', {'HIKVISION_PASSWORD': 'override'}):
                self.assertEqual(load_cameras(path)[0].password, 'override')

    def test_digest_and_bounded_xml_over_real_local_http(self):
        received = []
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass
            def do_PUT(self):
                body = self.rfile.read(int(self.headers['Content-Length']))
                auth = self.headers.get('Authorization', '')
                if not auth.startswith('Digest '):
                    self.send_response(401)
                    self.send_header('WWW-Authenticate', 'Digest realm="camera", nonce="12345", qop="auth"')
                    self.end_headers()
                    return
                received.append((self.path, ET.fromstring(body), auth))
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b'<ResponseStatus xmlns="http://www.hikvision.com/ver20/XMLSchema"><statusCode>1</statusCode></ResponseStatus>')
        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        client = HikvisionPTZ(replace(CONFIG, http_port=server.server_port, invert_pan=True))
        try:
            client.move((25, -30, 0))
            client.stop()
        finally:
            client.close()
            server.shutdown()
            server.server_close()
            thread.join(1)
        self.assertEqual(len(received), 2)
        path, root, auth = received[0]
        self.assertTrue(path.endswith('/1/momentary'))
        self.assertNotIn('test-secret', auth)
        self.assertEqual(root.find('{*}pan').text, '-25')
        self.assertEqual(root.find('{*}Momentary/{*}duration').text, '350')
        self.assertTrue(received[1][0].endswith('/continuous'))
        self.assertTrue(all(n.text == '0' for n in received[1][1]))

    def test_http_and_xml_errors_do_not_disclose_credentials(self):
        for code, body in [(401, b''), (403, b''), (404, b''), (500, b''),
                           (200, b'<ResponseStatus><statusCode>4</statusCode></ResponseStatus>'),
                           (200, b'<html>test-secret</html>'), (200, b'not xml')]:
            with self.subTest(code=code, body=body):
                response = SimpleNamespace(status_code=code, content=body, close=Mock())
                session = Mock(put=Mock(return_value=response))
                client = HikvisionPTZ(CONFIG, session=session)
                with self.assertRaises(PTZError) as caught:
                    client.move((0, 0, 25))
                self.assertNotIn('test-secret', str(caught.exception))
                self.assertFalse(session.put.call_args.kwargs['allow_redirects'])
                self.assertEqual(session.put.call_args.kwargs['timeout'], (0.8, 0.8))
                response.close.assert_called_once()

    def test_timeout_becomes_readable_error(self):
        import requests
        client = HikvisionPTZ(CONFIG, session=Mock(put=Mock(side_effect=requests.Timeout('test-secret'))))
        with self.assertRaisesRegex(PTZError, 'таймаут') as caught:
            client.move((1, 0, 0))
        self.assertNotIn('test-secret', str(caught.exception))

    @contextlib.contextmanager
    def worker(self, client):
        worker = PTZWorker(client)
        try:
            yield worker
        finally:
            worker.close()
            worker.thread.join(2)
            self.assertFalse(worker.thread.is_alive())

    def test_release_during_inflight_move_orders_stop_last(self):
        entered, release = threading.Event(), threading.Event()
        calls = []
        def move(vector):
            calls.append(('move', vector))
            entered.set()
            release.wait(1)
        client = SimpleNamespace(move=move, stop=lambda: calls.append(('stop',)), close=lambda: None)
        with self.worker(client) as worker:
            token = worker.begin((20, 0, 0))
            self.assertTrue(entered.wait(1))
            worker.stop()
            worker.renew(token)  # Old button heartbeat must not restart movement.
            release.set()
            eventually(lambda: len(calls) >= 2)
            time.sleep(.26)
            self.assertEqual(calls, [('move', (20, 0, 0)), ('stop',)])

    def test_missing_gui_heartbeat_stops(self):
        client = Mock()
        with self.worker(client) as worker:
            worker.begin((0, 25, 0))
            eventually(lambda: client.stop.call_count > 0)
            moves = client.move.call_count
            time.sleep(.26)
            self.assertEqual(client.move.call_count, moves)

    def test_failed_command_is_latched_until_new_press(self):
        client = Mock()
        client.move.side_effect = PTZError('PTZ: HTTP 403')
        with self.worker(client) as worker:
            token = worker.begin((0, 0, 25))
            eventually(lambda: client.stop.call_count > 0)
            for _ in range(6):
                worker.renew(token)
                time.sleep(.05)
            self.assertEqual(client.move.call_count, 1)
            self.assertIn('403', worker.get_status())
            client.move.side_effect = None
            worker.begin((0, 0, -20))
            eventually(lambda: client.move.call_count == 2)

    def test_one_blocked_ptz_does_not_block_another(self):
        entered, release = threading.Event(), threading.Event()
        slow = Mock()
        slow.move.side_effect = lambda _: (entered.set(), release.wait(1))
        fast = Mock()
        with self.worker(slow) as one, self.worker(fast) as two:
            one.begin((20, 0, 0))
            self.assertTrue(entered.wait(1))
            two.begin((0, 0, 20))
            eventually(lambda: fast.move.call_count == 1)
            release.set()

    def test_rtsp_failure_fallback_disconnect_reconnect_and_shutdown(self):
        released, paths = [], []
        class Capture:
            def __init__(self, url, *args):
                self.url, self.count = url, 0
                paths.append(url)
            def isOpened(self):
                return 'bad-host' not in self.url and self.url.endswith('/101')
            def read(self):
                time.sleep(.005)
                self.count += 1
                if self.count == 4:
                    raise RuntimeError('decoder disconnected')
                return True, SimpleNamespace(size=1)
            def release(self):
                released.append(self.url)
        cv = SimpleNamespace(VideoCapture=Capture, CAP_FFMPEG=1,
                             CAP_PROP_OPEN_TIMEOUT_MSEC=2, CAP_PROP_READ_TIMEOUT_MSEC=3)
        readers = [VideoReader(replace(CONFIG, host=host), cv2=cv, retry_seconds=.02)
                   for host in ['bad-host', 'good-one', 'good-two']]
        try:
            for reader in readers:
                reader.start()
            eventually(lambda: readers[1].sequence > 5 and readers[2].sequence > 5)
            self.assertEqual(readers[0].sequence, 0)
            self.assertTrue(any(p.endswith('/102') for p in paths))
            self.assertTrue(any(p.endswith('/101') for p in paths))
        finally:
            for reader in readers:
                reader.close()
            for reader in readers:
                reader.thread.join(1)
                self.assertFalse(reader.thread.is_alive())
        self.assertEqual(len(released), len(paths))

    def test_stale_video_cancels_held_motion(self):
        panel = SimpleNamespace(reader=Mock(), stop_motion=Mock(), ptz=Mock(), held=object(),
                                video_status=Mock(), ptz_status=Mock(), canvas=Mock(),
                                winfo_width=lambda: 400, draw_key=None, photo=None)
        panel.reader.snapshot.return_value = (object(), time.monotonic() - 10, 1, 'old')
        panel.canvas.winfo_width.return_value = 400
        panel.canvas.winfo_height.return_value = 250
        panel.ptz.get_status.return_value = 'PTZ: готово'
        CameraPanel.update_view(panel)
        panel.stop_motion.assert_called_once()
        panel.ptz.renew.assert_not_called()
        panel.canvas.create_image.assert_not_called()

    def test_focus_loss_and_minimize_stop_motion(self):
        window = SimpleNamespace(focus_displayof=lambda: None, stop_all=Mock())
        CameraWindow._check_focus(window)
        window.stop_all.assert_called_once()
        window.stop_all.reset_mock()
        CameraWindow._unmap(window, SimpleNamespace(widget=window))
        window.stop_all.assert_called_once()

    def test_window_close_requests_all_stops_before_destroy(self):
        panels = [Mock(), Mock(), Mock()]
        window = SimpleNamespace(on_closed=None, closing=False, _timer='tick', _focus_timer=None,
                                 panels=panels, after_cancel=Mock(), title=Mock(), _finish_close=Mock())
        CameraWindow.close(window)
        self.assertTrue(window.closing)
        for panel in panels:
            panel.shutdown.assert_called_once()
        window._finish_close.assert_called_once()

    def test_app_reuses_window_without_importing_robot_dependencies(self):
        source = ast.parse((Path(__file__).parents[1] / 'app.py').read_text())
        cls = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'RobotControlUI')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'open_cameras')
        namespace = {}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])), 'app.py', 'exec'), namespace)
        camera = Mock()
        camera.winfo_exists.return_value = True
        app = SimpleNamespace(camera_window=camera)
        namespace['open_cameras'](app)
        camera.present.assert_called_once()


if __name__ == '__main__':
    unittest.main()
