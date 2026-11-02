import numpy as np
from .base import BaseTrajectory


class ZigzagTrajectory(BaseTrajectory):
    """
    Sinusoidal back-and-forth along the X-axis (no Y motion).

    x(t) = amp * sin(ω t)     ← oscillates ±amp
    y(t) = 0

    Peak speed = amp * ω  (at the midpoint of each stroke).
    """

    def __init__(self, args):
        self._amp = args.amplitude if getattr(args, 'amplitude', None) is not None else 2.5
        if getattr(args, 'target_speed', None) is not None:
            self._omega = args.target_speed / self._amp   # peak speed → omega
            print(f" Zigzag: peak_speed={args.target_speed:.2f} m/s "
                  f"→ omega={self._omega:.4f} rad/s  (period={self.period:.2f} s)")
        else:
            self._omega = 2 * np.pi / 8
            print(f" Zigzag: omega={self._omega:.4f} rad/s  "
                  f"(period={self.period:.2f} s, "
                  f"peak speed={self._amp * self._omega:.2f} m/s)")

    def __call__(self, t: float):
        pos = np.array([self._amp * np.sin(self._omega * t), 0.0, 0.0])
        vel = np.array([self._amp * self._omega * np.cos(self._omega * t), 0.0, 0.0])
        return pos, vel

    @property
    def period(self) -> float:
        return 2 * np.pi / self._omega

    @property
    def name(self) -> str:
        return "zigzag"
