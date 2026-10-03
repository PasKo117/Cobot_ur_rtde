"""Read UR joints via RTDE and mirror them into an existing RoboDK robot."""
import argparse
import math
import time
from standalone_rtde import RobotSession


def joints_changed(previous, current, tolerance_deg=0.1):
    return previous is None or any(abs(a-b) > tolerance_deg for a, b in zip(previous, current))


def main(robot_name='diagnost', argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--once', action='store_true', help='Считать одно положение и выйти')
    parser.add_argument('--interval', type=float, default=0.1, help='Интервал опроса, с')
    parser.add_argument('--targets', action='store_true', help='Создавать цели через каждые 10 градусов')
    args = parser.parse_args(argv)
    if not 0.05 <= args.interval <= 10:
        parser.error('--interval должен быть 0.05..10 с')
    from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
    rdk = Robolink()
    name = 'Diagnost' if robot_name == 'diagnost' else 'Hirurg'
    virtual = rdk.Item(name, ITEM_TYPE_ROBOT)
    if not virtual.Valid():
        raise RuntimeError(f'Робот {name} не найден в RoboDK')
    last, last_target = None, None
    count = 0
    with RobotSession(robot_name, control=False) as robot:
        try:
            while True:
                degrees = [v * 180 / math.pi for v in robot.joints()]
                if joints_changed(last, degrees):
                    virtual.setJoints(degrees)
                    last = degrees
                if args.targets and joints_changed(last_target, degrees, 10):
                    count += 1
                    target = rdk.AddTarget(f'{name} {count}', 0, virtual)
                    target.setAsJointTarget()
                    target.setJoints(degrees)
                    last_target = degrees
                if args.once:
                    print(f'{name}: {degrees}')
                    return
                time.sleep(args.interval)
        except KeyboardInterrupt:
            return
