"""Synthetic vision checks independent of a webcam or Pi."""
from io import BytesIO
import unittest
import cv2
import numpy as np
from vision import estimate_pose, load_calibration


class VisionTests(unittest.TestCase):
    def test_generated_marker_detected(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        marker = cv2.aruco.generateImageMarker(dictionary, 0, 240)
        image = cv2.copyMakeBorder(marker, 60, 60, 60, 60, cv2.BORDER_CONSTANT, value=255)
        _, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(image)
        self.assertEqual(ids.flatten().tolist(), [0])

    def test_known_projected_pose_recovered(self):
        matrix = np.array([[800., 0, 320], [0, 800, 240], [0, 0, 1]])
        distortion = np.zeros(5)
        points = np.array([[-.05, .05, 0], [.05, .05, 0],
                           [.05, -.05, 0], [-.05, -.05, 0]], dtype=np.float32)
        expected = np.array([.12, -.03, 1.1])
        corners, _ = cv2.projectPoints(points, np.array([3.0, .1, .05]), expected, matrix, distortion)
        pose = estimate_pose(corners, .1, matrix, distortion)
        self.assertIsNotNone(pose)
        np.testing.assert_allclose(pose, expected, atol=.001)

    def test_bad_calibration_rejected(self):
        data = BytesIO()
        np.savez(data, camera_matrix=np.zeros((3, 3)), dist_coeffs=np.zeros(5), image_size=[640, 480])
        data.seek(0)
        with self.assertRaises(ValueError):
            load_calibration(data)
