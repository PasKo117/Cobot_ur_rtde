"""Laser/robot experiment with one bounded probe per explicit --execute call.

Uses the Raspberry Pi JSON sensor protocol from laser_sensor_lib.py. Legacy
raw 192.168.8.149:9093 messages are not assumed to be compatible.
"""
import argparse
import csv
from datetime import datetime
from pathlib import Path

from laser_sensor_lib import LaserSensorClient
from standalone_rtde import RobotSession


def ones_complement_to_signed(value, bit_length=16):
    mask = (1 << bit_length) - 1
    return -((~value & mask) + 1) if value & (1 << (bit_length-1)) else value


def laser_values(client):
    values = client.get_sensor_values()
    if values['status_1'] != 'ok' or values['status_2'] != 'ok':
        raise ConnectionError(f'Лазерные показания недоступны: {values}')
    return values['sensor_1'], values['sensor_2']


def run_probe(robot, sensor, *, step_mm, speed, angle_deg=0):
    if not 0 < abs(step_mm) <= 5 or not 0 < speed <= 0.02 or not -25 <= angle_deg <= 25:
        raise ValueError('Шаг <= 5 мм, скорость <= 0.02 м/с, угол <= 25°')
    start = robot.pose()
    if angle_deg:
        from scipy.spatial.transform import Rotation
        from math import radians
        target = start[:3] + list((Rotation.from_rotvec(start[3:])
                                    * Rotation.from_euler('x', radians(angle_deg))).as_rotvec())
        robot.move(target, 0.02)
    before = laser_values(sensor)
    robot.translate(step_mm / 1000, speed)
    after = laser_values(sensor)
    return [('before', *start, *before), ('after', *robot.pose(), *after)]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--experiment', choices=('uno', 'duo', 'tres'), default='uno',
                        help='uno: диагност; duo: хирург с углом; tres: хирург без угла')
    parser.add_argument('--execute', action='store_true', help='Выполнить один пробный шаг')
    parser.add_argument('--step-mm', type=float, default=1)
    parser.add_argument('--speed', type=float, default=0.005)
    parser.add_argument('--angle-deg', type=float, default=5)
    args = parser.parse_args(argv)
    sensor = LaserSensorClient()
    if not sensor.connect(max_attempts=1):
        raise ConnectionError('Raspberry Pi лазерных датчиков недоступен')
    try:
        values = laser_values(sensor)
        print('Диагност / хирург, мм:', values)
        if not args.execute:
            return
        name = 'diagnost' if args.experiment == 'uno' else 'surgeon'
        angle = args.angle_deg if args.experiment == 'duo' else 0
        with RobotSession(name) as robot:
            rows = run_probe(robot, sensor, step_mm=args.step_mm,
                             speed=args.speed, angle_deg=angle)
        output = Path('experiments') / f'laser_{args.experiment}_{datetime.now():%Y%m%d_%H%M%S}.csv'
        output.parent.mkdir(exist_ok=True)
        with output.open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['phase','x','y','z','rx','ry','rz','diagnost_mm','surgeon_mm'])
            writer.writerows(rows)
        print('Файл:', output)
    finally:
        sensor.disconnect()


if __name__ == '__main__':
    main()
