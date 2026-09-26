"""Read and write six-coordinate RTDE TCP waypoint files."""
import csv
import math
from pathlib import Path

HEADER = ('x', 'y', 'z', 'rx', 'ry', 'rz')


def validate_pose(values):
    if len(values) != 6:
        raise ValueError('Точка маршрута должна содержать 6 координат TCP: x,y,z,rx,ry,rz')
    try:
        pose = [float(v) for v in values]
    except (TypeError, ValueError) as error:
        raise ValueError('Координаты маршрута должны быть числами') from error
    if not all(math.isfinite(v) for v in pose):
        raise ValueError('Координаты маршрута должны быть конечными числами')
    return pose


def read_poses(filename):
    result = []
    with open(filename, newline='', encoding='utf-8-sig') as stream:
        for lineno, raw in enumerate(stream, 1):
            line = raw.strip()
            if not line:
                continue
            delimiter = '|' if '|' in line else ';' if ';' in line else ','
            fields = next(csv.reader([line], delimiter=delimiter))
            if not result and tuple(f.strip().lower() for f in fields) == HEADER:
                continue
            try:
                result.append(validate_pose(fields))
            except ValueError as error:
                raise ValueError(f'Строка {lineno}: {error}') from error
            if len(result) > 10000:
                raise ValueError('Слишком много точек в маршруте (максимум 10000)')
    if not result:
        raise ValueError('Файл маршрута пуст')
    return result


def write_poses(filename, poses):
    validated = [validate_pose(pose) for pose in poses]
    if not validated:
        raise ValueError('Не записано ни одной точки маршрута')
    if len(validated) > 10000:
        raise ValueError('Слишком много точек в маршруте (максимум 10000)')
    with Path(filename).open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(HEADER)
        writer.writerows(validated)
    return len(validated)
