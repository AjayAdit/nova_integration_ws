import numpy as np
from .base import BaseTrajectory


class CircleTrajectory(BaseTrajectory):
    """
    Constant-radius counter-clockwise circle on the ground plane.

    x(t) = amp * cos(ω t) - amp
    y(t) = amp * sin(ω t)

    Speed on the circle = amp * ω (uniform, no mean-factor needed).
    """

    def __init__(self, args):
        self._amp = args.amplitude if getattr(args, 'amplitude', None) is not None else 5.0            # radius in metres
        if getattr(args, 'target_speed', None) is not None:
            self._omega = args.target_speed / self._amp
            print(f" Circle: target_speed={args.target_speed:.2f} m/s "
                  f"→ omega={self._omega:.4f} rad/s  (period={self.period:.2f} s)")
        else:
            self._omega = 2 * np.pi / 8       # same default as lissajous
            print(f" Circle: omega={self._omega:.4f} rad/s  "
                  f"(period={self.period:.2f} s, "
                  f"speed={self._amp * self._omega:.2f} m/s)")

    def __call__(self, t: float):
        pos = np.array([
            self._amp * np.cos(self._omega * t) - self._amp,
            self._amp * np.sin(self._omega * t),
            0.0,
        ])
        vel = self._amp * self._omega * np.array([
            -np.sin(self._omega * t),
             np.cos(self._omega * t),
             0.0,
        ])
        return pos, vel

    @property
    def period(self) -> float:
        return 2 * np.pi / self._omega

    @property
    def name(self) -> str:
        return "circle"
