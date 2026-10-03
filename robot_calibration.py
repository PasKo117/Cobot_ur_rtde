"""Calibrate diagnost base -> surgeon base from operator supplied common points.

Default: inspect saved data. --collect enables freedrive in turn. --execute
also permits a single mapped target move after an explicit console confirmation.
"""
import argparse
from pathlib import Path
from calibration_manager import CalibrationManager
from standalone_rtde import sessions


class DualRobotSystem:
    def __init__(self, diagnost, surgeon, calib_file=None):
        self.diagnost, self.surgeon = diagnost, surgeon
        self.calib = CalibrationManager(calib_file)

    def collect_points(self, n_points=5):
        if not 4 <= n_points <= 20:
            raise ValueError('Количество общих точек: 4..20')
        a, b = [], []
        for i in range(n_points):
            for robot, name, dest in ((self.diagnost, 'диагност', a),
                                      (self.surgeon, 'хирург', b)):
                with robot.freedrive():
                    input(f'Точка {i+1}/{n_points}: подведите {name} к ОДНОЙ и той же метке, Enter: ')
                pos = robot.pose()[:3]
                print(name, pos)
                dest.append(pos)
        return a, b

    def run_calibration(self, n_points=5):
        a, b = self.collect_points(n_points)
        self.calib.calculate_transform_from_points(a, b)
        if self.calib.rmse > 0.005:
            raise ValueError(f'Невязка {self.calib.rmse*1000:.1f} мм превышает 5 мм; файл не сохранён')
        print('RMS, мм:', self.calib.rmse * 1000)
        print('Сохранено:', self.calib.save({'n_points': n_points, 'direction': 'diagnost_to_surgeon'}))

    def preview(self, *, execute=False):
        self.calib.load()
        target = self.calib.transform_pose(self.diagnost.pose())
        print('Расчётная поза хирурга [м, рад]:', target)
        if execute:
            if input('Освободите рабочую область. Для движения введите MOVE: ') == 'MOVE':
                self.surgeon.move(target, 0.02, 0.05)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--collect', action='store_true', help='Собрать парные точки во freedrive')
    parser.add_argument('--points', type=int, default=5)
    parser.add_argument('--preview', action='store_true', help='Показать расчётную позу хирурга')
    parser.add_argument('--execute', action='store_true', help='Разрешить движение по расчётной позе')
    parser.add_argument('--file', type=Path, default=Path(__file__).with_name('calibration_data.json'))
    args = parser.parse_args(argv)
    if args.execute and not args.preview:
        parser.error('--execute требует --preview')
    if not (args.collect or args.preview):
        calib = CalibrationManager(args.file)
        calib.load()
        print('Калибровка корректна:', calib.verify(), 'базы, м:', calib.get_translation_distance())
        return
    with sessions('diagnost', 'surgeon') as robots:
        system = DualRobotSystem(robots['diagnost'], robots['surgeon'], args.file)
        if args.collect:
            system.run_calibration(args.points)
        if args.preview:
            system.preview(execute=args.execute)


if __name__ == '__main__':
    main()
