"""Pure differential-drive controller; OpenCV camera x points right, z forward."""
import math


class Controller:
    def __init__(self, config):
        self.c = config
        self.previous = (0.0, 0.0)
        if not 0 < config['max_pwm'] <= 1 or config['slew_per_second'] <= 0:
            raise ValueError('Invalid PWM limit or slew rate')

    def step(self, x, z, dt):
        if x is None or z is None or not all(map(math.isfinite, (x, z, dt))) or z <= 0 or dt <= 0 or dt > 0.5:
            self.previous = (0.0, 0.0)
            return self.previous
        # No automatic reverse. Stop inside the following distance.
        error = z - self.c['target_distance_m']
        if error <= 0.04:
            self.previous = (0.0, 0.0)
            return self.previous
        turn = self.c['steering_gain'] * math.atan2(x, z)
        speed = min(self.c['max_pwm'], self.c['distance_gain'] * error)
        target = [max(0.0, min(self.c['max_pwm'], speed + turn)),
                  max(0.0, min(self.c['max_pwm'], speed - turn))]
        delta = self.c['slew_per_second'] * dt
        self.previous = tuple(old + max(-delta, min(delta, new-old))
                              for old, new in zip(self.previous, target))
        return self.previous
