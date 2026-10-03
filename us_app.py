"""Legacy ultrasound GUI entry point; it now launches the maintained app.py.

The optional --route demonstration makes a bounded Y out-and-back scan on
"diagnost" and writes sampled force and pose to data.txt.
"""
import argparse
import csv
from pathlib import Path
import time
from standalone_rtde import RobotSession


def route_follow(nsteps, step_val, pause_time, vel, filename='data.txt'):
    if not 1 <= nsteps <= 100 or not 0 < abs(step_val) <= 0.02:
        raise ValueError('Требуются 1..100 шагов по оси Y, каждый не более 20 мм')
    if not 0 <= pause_time <= 60 or not 0 < vel <= 0.1:
        raise ValueError('Пауза 0..60 с; скорость 0..0.1 м/с')
    rows = []
    with RobotSession('diagnost') as robot:
        start = time.monotonic()
        for delta in [step_val] * nsteps + [-step_val] * nsteps:
            target = robot.pose()
            target[1] += delta
            robot.move(target, vel)
            rows.append([time.monotonic() - start, *robot.pose(), *robot.force()])
            time.sleep(pause_time)
    with Path(filename).open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s', 'x', 'y', 'z', 'rx', 'ry', 'rz',
                         'fx', 'fy', 'fz', 'mx', 'my', 'mz'])
        writer.writerows(rows)
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', action='store_true', help='Выполнить пробный маршрут без GUI')
    parser.add_argument('--steps', type=int, default=2)
    parser.add_argument('--step-mm', type=float, default=1)
    parser.add_argument('--pause', type=float, default=0.5)
    parser.add_argument('--speed', type=float, default=0.01)
    args = parser.parse_args(argv)
    if args.route:
        print('Записано точек:', len(route_follow(args.steps, args.step_mm / 1000,
                                                  args.pause, args.speed)))
    else:
        import multiprocessing as mp
        from app import RobotControlUI
        mp.freeze_support()
        gui = RobotControlUI()
        gui.protocol('WM_DELETE_WINDOW', gui.close)
        gui.mainloop()


if __name__ == '__main__':
    main()
