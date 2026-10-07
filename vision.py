"""Calibrated square-marker pose; coordinates are x right, y down, z forward."""
import cv2
import numpy as np


def load_calibration(path):
    with np.load(path, allow_pickle=False) as data:
        matrix = data['camera_matrix'].copy()
        distortion = data['dist_coeffs'].copy()
        raw_size = data['image_size'].copy()
    if (matrix.shape != (3, 3) or not np.isfinite(matrix).all()
            or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.isfinite(distortion).all()
            or distortion.size not in (4, 5, 8, 12, 14)
            or raw_size.shape != (2,) or not np.isfinite(raw_size).all()
            or np.any(raw_size <= 0) or np.any(raw_size != np.floor(raw_size))):
        raise ValueError('Invalid camera calibration')
    return matrix, distortion, tuple(raw_size.astype(int))


def estimate_pose(corners, marker_size, matrix, distortion, max_error=3):
    half = marker_size / 2
    # IPPE_SQUARE requires this object-point order.
    points = np.array([[-half, half, 0], [half, half, 0],
                       [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
    image_points = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    if half <= 0 or not np.isfinite(image_points).all():
        return None
    found, rotation, translation = cv2.solvePnP(
        points, image_points, matrix, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if (not found or not np.isfinite(translation).all()
            or not np.isfinite(rotation).all() or translation[2, 0] <= 0):
        return None
    projected, _ = cv2.projectPoints(points, rotation, translation, matrix, distortion)
    error = np.linalg.norm(projected.reshape(4, 2) - image_points, axis=1).mean()
    if error > max_error:
        return None
    return tuple(float(v) for v in translation[:, 0])
