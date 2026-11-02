import numpy as np
from .base import BaseTrajectory

# Arc-length integral of the lemniscate (figure-8) — used to convert
# a desired mean speed into an angular frequency.
#   mean over one lap of  sqrt(cos^2 u + 4 cos^2 2u),  u = omega t
# Speed is NOT constant on this path: the same integrand peaks at sqrt(5)
# and bottoms out at 0.69597, so the instantaneous speed spans
# 0.464x to 1.490x the requested mean.
LEMNISCATE_MEAN_FACTOR = 1.5007406


class LissajousTrajectory(BaseTrajectory):
    """
    Figure-8 (Lissajous / lemniscate) ground-plane trajectory.
    Exact port of the original lissajous_target() from sim_world.py.

    x(t) = amp * sin(ω t)
    y(t) = amp * sin(2ω t)
    """

    def __init__(self, args):
        self._amp = args.amplitude if getattr(args, 'amplitude', None) is not None else 2.5
        if getattr(args, 'target_speed', None) is not None:
            self._omega = args.target_speed / (LEMNISCATE_MEAN_FACTOR * self._amp)
            print(f" Lissajous: target_speed={args.target_speed:.2f} m/s "
                  f"→ omega={self._omega:.4f} rad/s  (period={self.period:.2f} s)")
        else:
            self._omega = 2 * np.pi / 8  # default ≈ 0.785 rad/s
            print(f" Lissajous: omega={self._omega:.4f} rad/s  "
                  f"(period={self.period:.2f} s, "
                  f"mean speed={LEMNISCATE_MEAN_FACTOR * self._amp * self._omega:.2f} m/s)")

    def __call__(self, t: float):
        s1 = np.sin(self._omega * t)
        s2 = np.sin(2 * self._omega * t)
        c1 = np.cos(self._omega * t)
        c2 = np.cos(2 * self._omega * t)
        pos = np.array([self._amp * s1, self._amp * s2, 0.0])
        vel = self._amp * self._omega * np.array([c1, 2 * c2, 0.0])
        return pos, vel

    @property
    def period(self) -> float:
        return 2 * np.pi / self._omega

    @property
    def name(self) -> str:
        return "lissajous"
