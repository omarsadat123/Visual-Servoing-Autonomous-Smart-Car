"""Exercise the actual vision loop with generated frames and mocked hardware."""
import unittest
from unittest.mock import MagicMock, patch
import cv2
import numpy as np
import follow


class FollowTests(unittest.TestCase):
    def run_loop(self, mode):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        marker = cv2.aruco.generateImageMarker(dictionary, 1, 80)
        frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        frame[200:280, 400:480] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        camera = MagicMock()
        camera.read.side_effect = [(True, frame), (True, np.full_like(frame, 255)), (False, None)]
        motors = MagicMock()
        matrix = np.array([[800., 0, 320], [0, 800, 240], [0, 0, 1]])
        with (patch('sys.argv', ['follow.py', '--mode', mode, '--headless', '--drive']),
              patch.object(follow.cv2, 'VideoCapture', return_value=camera),
              patch.object(follow, 'Motors', return_value=motors),
              patch.object(follow, 'load_calibration', return_value=(matrix, np.zeros(5), (640, 480))),
              patch.object(follow.signal, 'signal'),
              patch.object(follow.time, 'monotonic', side_effect=[1, 1.1, 1.2]),
              patch('builtins.print')):
            with self.assertRaisesRegex(RuntimeError, 'capture failed'):
                follow.main()
        commands = motors.set.call_args_list
        self.assertGreater(commands[0].args[0], 0)
        self.assertEqual(commands[1].args, (0, 0))
        motors.close.assert_called_once()
        camera.release.assert_called_once()

    def test_basic_follow_loss_and_capture_failure(self):
        self.run_loop('basic')

    def test_pose_follow_loss_and_capture_failure(self):
        self.run_loop('pose')
