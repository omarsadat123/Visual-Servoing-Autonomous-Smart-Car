import json
import unittest
from pathlib import Path
from control import Controller


class ControlTests(unittest.TestCase):
    def setUp(self):
        with Path(__file__).with_name('config.json').open() as f:
            self.config = json.load(f)
        self.controller = Controller(self.config)

    def test_lost_target_stops_immediately(self):
        self.controller.step(0, 2, .1)
        self.assertEqual(self.controller.step(None, None, .1), (0, 0))

    def test_close_target_and_stale_loop_stop(self):
        self.assertEqual(self.controller.step(0, .2, .1), (0, 0))
        self.assertEqual(self.controller.step(0, 2, 1), (0, 0))

    def test_right_target_turns_right(self):
        for _ in range(20):
            left, right = self.controller.step(.3, 1, .1)
        self.assertGreater(left, right)
        self.assertLessEqual(left, self.config['max_pwm'])

    def test_slew_limited(self):
        left, _ = self.controller.step(0, 2, .1)
        self.assertLessEqual(left, .04 + 1e-8)


if __name__ == '__main__':
    unittest.main()
