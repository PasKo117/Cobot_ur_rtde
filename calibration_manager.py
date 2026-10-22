"""Rigid transformation diagnost base -> surgeon base in RTDE TCP units."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


class CalibrationManager:
    def __init__(self, filename=None):
        self.filename = Path(filename or Path(__file__).with_name('calibration_data.json'))
        self.rotation = None
        self.translation = None
        self.rmse = None

    def calculate_transform_from_points(self, points1, points2):
        a = np.asarray(points1, dtype=float)
        b = np.asarray(points2, dtype=float)
        if a.shape != b.shape or a.ndim != 2 or a.shape[1] != 3 or len(a) < 4:
            raise ValueError('Нужно как минимум 4 парные точки XYZ в метрах')
        if not (np.isfinite(a).all() and np.isfinite(b).all()):
            raise ValueError('Координаты должны быть конечными числами')
        ac, bc = a.mean(axis=0), b.mean(axis=0)
        if np.linalg.matrix_rank(a - ac, tol=1e-6) < 2:
            raise ValueError('Точки почти коллинеарны; соберите разные положения')
        u, _, vt = np.linalg.svd((a - ac).T @ (b - bc))
        d = np.eye(3)
        d[2, 2] = np.linalg.det(vt.T @ u.T)
        self.rotation = vt.T @ d @ u.T
        self.translation = bc - self.rotation @ ac
        residual = a @ self.rotation.T + self.translation - b
        self.rmse = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
        return self.rotation, self.translation

    def verify(self):
        if self.rotation is None or self.translation is None:
            return False
        r = self.rotation
        return bool(np.isfinite(r).all() and np.isfinite(self.translation).all()
                    and np.allclose(r @ r.T, np.eye(3), atol=1e-3)
                    and abs(np.linalg.det(r) - 1) < 1e-3)

    def save(self, metadata=None):
        if not self.verify():
            raise ValueError('Некорректная калибровка')
        data = {'R': self.rotation.tolist(), 't': self.translation.tolist()}
        if metadata:
            data['metadata'] = metadata
        self.filename.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding='utf-8')
        return self.filename

    def load(self):
        data = json.loads(self.filename.read_text(encoding='utf-8'))
        self.rotation = np.asarray(data['R'], dtype=float)
        self.translation = np.asarray(data['t'], dtype=float)
        if self.rotation.shape != (3, 3) or self.translation.shape != (3,) or not self.verify():
            raise ValueError('Файл калибровки: неверная матрица R или вектор t')
        return self.rotation, self.translation

    def transform_pose(self, pose_list):
        from robot_control import valid_pose
        pose = valid_pose(pose_list)
        if not self.verify():
            raise ValueError('Сначала загрузите/рассчитайте калибровку')
        position = self.rotation @ pose[:3] + self.translation
        orientation = (Rotation.from_matrix(self.rotation)
                       * Rotation.from_rotvec(pose[3:])).as_rotvec()
        return list(position) + list(orientation)

    def transform_position_only(self, pose_list, orientation_offset=None):
        from robot_control import valid_pose
        pose = valid_pose(pose_list)
        if not self.verify():
            raise ValueError('Сначала загрузите/рассчитайте калибровку')
        position = self.rotation @ pose[:3] + self.translation
        orientation = orientation_offset if orientation_offset is not None else pose[3:]
        return list(position) + list(orientation)

    def get_translation_distance(self):
        return float(np.linalg.norm(self.translation)) if self.translation is not None else 0.0
