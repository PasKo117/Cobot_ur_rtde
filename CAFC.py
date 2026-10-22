"""Bounded experimental force measurement and contact-following probe.

Default prints force only. --execute drives the diagnost for at most 3 s at
small speeds and writes force/command samples. Not a certified force controller.
"""
import argparse
import csv
from datetime import datetime
import time

from standalone_rtde import RobotSession


def FC_velocity(Fm, f_raw, angle_rad=0.0, f_target=1.0, kf=0.01,
                m_tool=0.2, v_max=0.005):
    """Legacy proportional correction; gravity must be captured in Fm."""
    error = f_target - (f_raw - Fm)
    return max(-v_max, min(v_max, kf * error))


def force_calibration(robot, vel=0.002, acc=0.1, cal_time=0.4):
    if not 0 < cal_time <= 1 or not 0 < vel <= 0.005:
        raise ValueError('Калибровочное движение ограничено 1 с и 5 мм/с')
    def sample(vector):
        forces = []
        end = time.monotonic() + cal_time
        try:
            while time.monotonic() < end:
                robot.speed(vector, acc)
                forces.append(robot.force()[2])
                time.sleep(0.05)
        finally:
            robot.ctrl.speedStop(0.5)
        return sum(forces) / len(forces)
    return (sample([0]*6), sample([0, 0, vel, 0, 0, 0]),
            sample([0, 0, -vel, 0, 0, 0]))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Выполнить пробное движение')
    parser.add_argument('--seconds', type=float, default=3, help='Не более 3 с')
    args = parser.parse_args(argv)
    if not 0 < args.seconds <= 3:
        parser.error('--seconds должен быть в диапазоне 0..3')
    with RobotSession('diagnost', control=args.execute) as robot:
        print('TCP force [N, Nm]:', robot.force())
        if not args.execute:
            return
        baseline, up, down = force_calibration(robot)
        print('Базовая / +Z / -Z силы, Н:', baseline, up, down)
        output = f'force_probe_{datetime.now():%Y%m%d_%H%M%S}.csv'
        with open(output, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['time_s', 'force_z_N', 'command_z_m_s'])
            start = time.monotonic()
            try:
                while time.monotonic() - start < args.seconds:
                    force = robot.force()[2]
                    vz = FC_velocity(baseline, force)
                    robot.speed([0, 0, vz, 0, 0, 0])
                    writer.writerow([time.monotonic()-start, force, vz])
                    time.sleep(0.05)
            finally:
                robot.ctrl.speedStop(0.5)
        print('Файл:', output)


if __name__ == '__main__':
    main()
