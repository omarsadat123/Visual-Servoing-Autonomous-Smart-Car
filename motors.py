"""Four motor channels on two L298Ns, with an independent command timeout."""
import threading
import time


class Motors:
    def __init__(self, pins, enabled=False):
        self.devices = []
        self.sides = []
        channels = [(side, channel) for side in ('left', 'right') for channel in pins[side]]
        if any(len(pins[side]) != 2 for side in ('left', 'right')):
            raise ValueError('Configure two motor channels per side')
        flat = [pin for _, channel in channels for pin in channel]
        if any(len(channel) != 3 for _, channel in channels) or len(set(flat)) != 12:
            raise ValueError('Each motor needs three distinct BCM pins; no pins may overlap')
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.last = time.monotonic()
        if enabled:
            from gpiozero import Motor
            from gpiozero.pins.lgpio import LGPIOFactory
            self.factory = LGPIOFactory()
            try:
                for side, channel in channels:
                    forward, backward, enable = channel
                    self.devices.append(Motor(forward, backward, enable=enable,
                                              pwm=True, pin_factory=self.factory))
                    self.sides.append(side)
            except Exception:
                for device in self.devices:
                    device.close()
                self.factory.close()
                raise
        self.thread = threading.Thread(target=self._watchdog, daemon=True)
        self.thread.start()

    def set(self, left, right):
        with self.lock:
            for device, side in zip(self.devices, self.sides):
                value = left if side == 'left' else right
                device.value = max(-1.0, min(1.0, value))
            self.last = time.monotonic()

    def _watchdog(self):
        while not self.done.wait(0.05):
            with self.lock:
                if time.monotonic() - self.last > 0.35:
                    for device in self.devices:
                        device.stop()

    def close(self):
        self.done.set()
        self.thread.join()
        with self.lock:
            for device in self.devices:
                device.stop()
                device.close()
            if self.devices:
                self.factory.close()
