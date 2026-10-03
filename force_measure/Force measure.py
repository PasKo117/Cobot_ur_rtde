"""Standalone diagnostic force display; uses RTDEReceiveInterface."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import tkinter as tk
from standalone_rtde import RobotSession


class App(tk.Tk):
    def __init__(self, robot):
        super().__init__()
        self.robot = robot
        self.title('Сила TCP диагноста')
        self.label = tk.Label(self, font=('Arial', 17), padx=20, pady=20)
        self.label.pack()
        self.protocol('WM_DELETE_WINDOW', self.destroy)
        self.after(100, self.refresh)

    def refresh(self):
        try:
            self.label.configure(text=f'Сила диагноста Z: {self.robot.force()[2]:.2f} Н')
        except Exception as exc:
            self.label.configure(text=f'Ошибка RTDE: {exc}')
            return
        self.after(100, self.refresh)


def main():
    with RobotSession('diagnost', control=False) as robot:
        App(robot).mainloop()


if __name__ == '__main__':
    main()
