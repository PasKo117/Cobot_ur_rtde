"""Movable three-camera window: python camera_window.py (no robot connections)."""
import argparse
import time
import tkinter as tk
from tkinter import messagebox

from camera_backend import HikvisionPTZ, PTZWorker, STALE_SECONDS, VideoReader
from camera_config import load_cameras

BG = '#111923'
CARD = '#1b2938'
TEXT = '#e6eef6'
MUTED = '#a0b4c8'
BLUE = '#285a82'
GREEN = '#83dfba'
AMBER = '#ffd084'


def label(parent, text='', **kwargs):
    return tk.Label(parent, text=text, bg=CARD, fg=TEXT, font=('Segoe UI', 10), **kwargs)


def button(parent, text, command=None, **kwargs):
    return tk.Button(parent, text=text, command=command, bg=BLUE, fg=TEXT,
                     activebackground='#3c7eaa', activeforeground='white',
                     relief='flat', borderwidth=0, font=('Segoe UI', 11),
                     cursor='hand2', padx=8, pady=5, **kwargs)


class CameraPanel(tk.Frame):
    def __init__(self, parent, config, cv2, image, image_tk):
        super().__init__(parent, bg=CARD, padx=8, pady=8)
        self.config = config
        self.cv2, self.Image, self.ImageTk = cv2, image, image_tk
        self.reader = VideoReader(config, cv2)
        self.ptz = PTZWorker(HikvisionPTZ(config))
        self.held = None
        self.token = None
        self.photo = None
        self.draw_key = None
        self.speed = tk.IntVar(value=25)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)
        label(self, f'{config.name}  •  {config.host}').grid(row=0, column=0, sticky='w')
        self.video_status = label(self, 'Подключение…')
        self.video_status.grid(row=1, column=0, sticky='w', pady=(3, 6))
        self.canvas = tk.Canvas(self, bg='#080e16', highlightthickness=0, width=320, height=200)
        self.canvas.grid(row=2, column=0, sticky='nsew')
        controls = tk.Frame(self, bg=CARD)
        controls.grid(row=3, column=0, sticky='ew', pady=(8, 0))
        controls.columnconfigure(1, weight=1)
        pad = tk.Frame(controls, bg=CARD)
        pad.grid(row=0, column=0, rowspan=2, sticky='w')
        directions = [('↖', -1, 1), ('↑', 0, 1), ('↗', 1, 1),
                      ('←', -1, 0), ('■', 0, 0), ('→', 1, 0),
                      ('↙', -1, -1), ('↓', 0, -1), ('↘', 1, -1)]
        for index, (text, pan, tilt) in enumerate(directions):
            control = button(pad, text, width=2, takefocus=False)
            control.grid(row=index // 3, column=index % 3, padx=2, pady=2)
            if pan == tilt == 0:
                control.configure(command=lambda: self.stop_motion(force=True), bg='#943f4b')
            else:
                self._bind_hold(control, (pan, tilt, 0))
        zoom = tk.Frame(controls, bg=CARD)
        zoom.grid(row=0, column=1, sticky='n', padx=(8, 0))
        label(zoom, 'Зум камеры').pack()
        zoom_buttons = tk.Frame(zoom, bg=CARD)
        zoom_buttons.pack(pady=3)
        for text, sign in [('−', -1), ('+', 1)]:
            control = button(zoom_buttons, text, width=3, takefocus=False)
            control.pack(side='left', padx=3)
            self._bind_hold(control, (0, 0, sign))
        speed_box = tk.Frame(controls, bg=CARD)
        speed_box.grid(row=1, column=1, sticky='ew', padx=(10, 0))
        label(speed_box, 'Скорость PTZ, %').pack(anchor='w')
        tk.Scale(speed_box, from_=1, to=100, orient='horizontal', variable=self.speed,
                 bg=CARD, fg=TEXT, troughcolor=BG, highlightthickness=0,
                 activebackground=BLUE, length=130, command=lambda _: self.stop_motion()).pack(fill='x')
        self.ptz_status = label(self, 'PTZ: готово к команде', anchor='w')
        self.ptz_status.grid(row=4, column=0, sticky='ew', pady=(6, 0))
        self.reader.start()

    def _bind_hold(self, control, vector):
        control.bind('<ButtonPress-1>', lambda event: self._begin(control, vector))
        control.bind('<ButtonRelease-1>', lambda event: self.stop_motion())
        control.bind('<Leave>', lambda event: self.stop_motion() if self.held is control else None)

    def _fresh(self):
        frame, stamp, _, _ = self.reader.snapshot()
        return frame is not None and time.monotonic() - stamp < STALE_SECONDS

    def _begin(self, control, vector):
        self.stop_motion()
        if not self._fresh():
            self.ptz_status.configure(text='PTZ: дождитесь живого видео', fg=AMBER)
            return 'break'
        self.winfo_toplevel().focus_set()
        self.held = control
        control.configure(bg='#368daa')
        self.token = self.ptz.begin(tuple(int(self.speed.get()) * axis for axis in vector))
        return 'break'

    def stop_motion(self, force=False):
        if self.held is not None:
            self.held.configure(bg=BLUE)
            self.held = None
            self.token = None
            self.ptz.stop()
        elif force:
            self.ptz.stop()

    def update_view(self):
        frame, stamp, sequence, status = self.reader.snapshot()
        fresh = frame is not None and time.monotonic() - stamp < STALE_SECONDS
        if not fresh:
            self.stop_motion()
            if frame is not None:
                status = 'не удалось подключить — нет свежих кадров'
        elif self.held is not None:
            self.ptz.renew(self.token)
        self.video_status.configure(text=status, fg=GREEN if fresh else AMBER)
        ptz_status = self.ptz.get_status()
        self.ptz_status.configure(text=ptz_status, fg=AMBER if self.ptz.failed else MUTED,
                                  wraplength=max(200, self.winfo_width() - 20))
        width, height = max(1, self.canvas.winfo_width()), max(1, self.canvas.winfo_height())
        key = (sequence if fresh else status, width, height, fresh)
        if key == self.draw_key:
            return
        self.draw_key = key
        self.canvas.delete('all')
        if fresh:
            try:
                picture = self.Image.fromarray(self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB))
                picture.thumbnail((width, height), self.Image.Resampling.BILINEAR)
                self.photo = self.ImageTk.PhotoImage(picture, master=self.canvas)
                self.canvas.create_image(width // 2, height // 2, image=self.photo)
            except Exception:
                self.photo = None
                self.canvas.create_text(width // 2, height // 2, text='Ошибка отображения кадра', fill=AMBER)
        else:
            self.photo = None
            self.canvas.create_text(width // 2, height // 2, text=status, fill=AMBER,
                                    width=max(1, width - 20), justify='center', font=('Segoe UI', 12))

    def shutdown(self):
        self.stop_motion()
        self.reader.close()
        self.ptz.close()


class CameraWindow(tk.Toplevel):
    """A normal nonmodal OS window. No transient/grab/topmost coupling to app.py."""
    def __init__(self, master, configs=None):
        # Dependencies are optional until the camera window is requested.
        import os
        os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;tcp'
        os.environ.setdefault('OPENCV_LOG_LEVEL', 'SILENT')
        import cv2
        from PIL import Image, ImageTk
        import requests  # noqa: F401 — fail before constructing a partial window
        configs = configs if configs is not None else load_cameras()
        if not cv2.videoio_registry.hasBackend(cv2.CAP_FFMPEG):
            raise RuntimeError('В OpenCV отсутствует FFmpeg. Установите opencv-python.')
        super().__init__(master)
        self.title('Камеры Hikvision • просмотр и PTZ')
        self.configure(bg=BG)
        self.minsize(1000, 560)
        self.geometry(f'{min(1500, self.winfo_screenwidth() - 60)}x'
                      f'{min(850, self.winfo_screenheight() - 100)}')
        self.panels = []
        self.closing = False
        self.done = False
        self.on_closed = None
        self._timer = None
        self._close_timer = None
        self._focus_timer = None
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        bar = tk.Frame(self, bg=BG, padx=10, pady=8)
        bar.grid(row=0, column=0, sticky='ew')
        button(bar, '■ Стоп всех камер', lambda: self.stop_all(force=True)).pack(side='left', padx=(0, 10))
        self.layout_button = button(bar, '▦ Мозаика', self.toggle_layout)
        self.layout_button.pack(side='left')
        tk.Label(bar, text='Удерживайте стрелку или + / −.  Esc — стоп.',
                 bg=BG, fg=MUTED, font=('Segoe UI', 10)).pack(side='right', padx=8)
        self.container = tk.Frame(self, bg=BG)
        self.container.grid(row=1, column=0, sticky='nsew', padx=4)
        footer = tk.Label(self, text='Перенесите окно за заголовок на второй монитор.  '
                         'Windows: Win + Shift + ← / →.  PTZ не управляет роботами.',
                         bg=BG, fg=MUTED, font=('Segoe UI', 10), pady=8)
        footer.grid(row=2, column=0, sticky='ew')
        self.mosaic = False
        try:
            for config in configs:
                self.panels.append(CameraPanel(self.container, config, cv2, Image, ImageTk))
        except Exception:
            for panel in self.panels:
                panel.shutdown()
            self.destroy()
            raise
        self.arrange()
        self.protocol('WM_DELETE_WINDOW', self.close)
        self.bind('<ButtonRelease-1>', lambda _: self.stop_all(), add='+')
        self.bind('<Escape>', lambda _: self.stop_all(force=True))
        self.bind('<FocusOut>', self._focus_out, add='+')
        self.bind('<Unmap>', self._unmap, add='+')
        self._tick()

    def arrange(self):
        for index in range(3):
            self.container.columnconfigure(index, weight=0, uniform='')
            self.container.rowconfigure(index, weight=0, uniform='')
        cols = 2 if self.mosaic else len(self.panels)
        for index in range(cols):
            self.container.columnconfigure(index, weight=1, uniform='camera-columns')
        for index, panel in enumerate(self.panels):
            row, col = divmod(index, cols)
            self.container.rowconfigure(row, weight=1, uniform='camera-rows')
            panel.grid(row=row, column=col, sticky='nsew', padx=5, pady=5)

    def toggle_layout(self):
        self.stop_all()
        self.mosaic = not self.mosaic
        self.minsize(1000, 700 if self.mosaic else 560)
        self.layout_button.configure(text='▤ В один ряд' if self.mosaic else '▦ Мозаика')
        self.arrange()

    def _focus_out(self, event):
        if self.closing:
            return
        if self._focus_timer:
            self.after_cancel(self._focus_timer)
        self._focus_timer = self.after(30, self._check_focus)

    def _check_focus(self):
        self._focus_timer = None
        focused = self.focus_displayof()
        if focused is None or focused.winfo_toplevel() != self:
            self.stop_all()

    def _unmap(self, event):
        if event.widget is self:
            self.stop_all()

    def stop_all(self, force=False):
        for panel in self.panels:
            panel.stop_motion(force=force)

    def _tick(self):
        if self.closing:
            return
        for panel in self.panels:
            # A failure rendering one panel must not interrupt the update loop of the others.
            try:
                panel.update_view()
            except Exception as exc:
                panel.stop_motion()
                panel.video_status.configure(text=f'Ошибка камеры: {type(exc).__name__}', fg=AMBER)
        self._timer = self.after(50, self._tick)

    def present(self):
        if not self.closing:
            self.deiconify()
            self.lift()
            self.focus_set()

    def close(self, on_closed=None):
        if on_closed is not None:
            self.on_closed = on_closed
        if self.closing:
            return
        self.closing = True
        if self._timer:
            self.after_cancel(self._timer)
        if self._focus_timer:
            self.after_cancel(self._focus_timer)
        for panel in self.panels:
            panel.shutdown()
        self.title('Камеры • остановка PTZ и закрытие…')
        self._close_deadline = time.monotonic() + 4.0
        self._finish_close()

    def _finish_close(self):
        workers_alive = any(panel.ptz.thread.is_alive() for panel in self.panels)
        if workers_alive and time.monotonic() < self._close_deadline:
            self._close_timer = self.after(50, self._finish_close)
            return
        self.done = True
        callback = self.on_closed
        self.destroy()
        if callback:
            callback()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', help='Путь к cameras.local.json')
    args = parser.parse_args(argv)
    root = tk.Tk()
    root.withdraw()
    try:
        window = CameraWindow(root, load_cameras(args.config))
    except Exception as exc:
        # No raw configuration/URL in error dialogs.
        text = (f'Не удалось открыть окно камер: {type(exc).__name__}.\n'
                'Проверьте cameras.local.json и установите зависимости:\n'
                'python -m pip install -r requirements-cameras.txt')
        print(text)
        messagebox.showerror('Камеры', text, parent=root)
        root.destroy()
        return 1
    window.on_closed = root.destroy
    try:
        root.mainloop()
    except KeyboardInterrupt:
        window.close(root.destroy)
        root.mainloop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
