"""Print surgeon TCP pose and joints without moving the robot."""
from standalone_rtde import RobotSession


def main():
    with RobotSession('surgeon', control=False) as robot:
        print('TCP (м/рад):', robot.pose())
        print('Суставы (рад):', robot.joints())


if __name__ == '__main__':
    main()
