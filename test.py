"""Bounded axial motion example; default is a read-only preview."""
import argparse
from standalone_rtde import RobotSession
from robot_control import tool_translation


def main(argv=None):
    parser = argparse.ArgumentParser(description='Проба движения инструмента диагноста')
    parser.add_argument('--execute', action='store_true', help='Выполнить один шаг 5 мм и возврат')
    args = parser.parse_args(argv)
    with RobotSession('diagnost', control=args.execute) as robot:
        start = robot.pose()
        target = tool_translation(start, -0.005)
        print('Текущая поза:', start, 'Цель:', target)
        if args.execute:
            robot.move(target, 0.005)
            robot.move(start, 0.005)
            print('Суставы:', robot.joints())


if __name__ == '__main__':
    main()
