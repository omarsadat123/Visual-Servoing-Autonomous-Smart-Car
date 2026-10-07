"""Motor routing tests without GPIO hardware."""
import threading
import unittest
from types import SimpleNamespace
from motors import Motors


class MotorTests(unittest.TestCase):
    def test_four_channel_routing(self):
        motors = Motors.__new__(Motors)
        motors.lock = threading.Lock()
        motors.devices = [SimpleNamespace(value=0) for _ in range(4)]
        motors.sides = ['left', 'left', 'right', 'right']
        motors.set(.2, .3)
        self.assertEqual([m.value for m in motors.devices], [.2, .2, .3, .3])
        motors.set(0, 0)
        self.assertEqual([m.value for m in motors.devices], [0, 0, 0, 0])

    def test_duplicate_pins_rejected(self):
        with self.assertRaises(ValueError):
            Motors({'left': [[1, 2, 3], [4, 5, 6]],
                    'right': [[7, 8, 9], [10, 11, 1]]})


if __name__ == '__main__':
    unittest.main()
