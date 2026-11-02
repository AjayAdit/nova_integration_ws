import numpy as np
from .base import BaseTrajectory


class StraightLineTrajectory(BaseTrajectory):
    """
    Back-and-forth straight line on the ground plane (along X axis).

    Uses a triangle-wave so the target moves at constant speed,
    reversing direction at ±amplitude.

    x(t) = triangle wave with amplitude amp and period T
    y(t) = 0
    """

    def __init__(self, args):
        self._amp = args.amplitude if getattr(args, 'amplitude', None) is not None else 149.0
        if getattr(args, 'target_speed', None) is not None:
            self._speed = args.target_speed
        else:
            self._speed = 2.0  # default 2 m/s
        # Period = time to go from -amp to +amp and back
        self._period = 4.0 * self._amp / self._speed
        print(f" StraightLine: speed={self._speed:.2f} m/s  "
              f"amplitude={self._amp:.2f} m  (period={self._period:.2f} s)")

    def __call__(self, t: float):
        T = self._period
        # Phase within one period [0, T)
        phase = t % T
        quarter_T = T / 4.0
        three_quarter_T = 3.0 * T / 4.0

        if phase < quarter_T:
            # Moving forward: 0 → +amp
            x = self._speed * phase
            vx = self._speed
        elif phase < three_quarter_T:
            # Moving backward: +amp → -amp
            x = self._amp - self._speed * (phase - quarter_T)
            vx = -self._speed
        else:
            # Moving forward again: -amp → 0
            x = -self._amp + self._speed * (phase - three_quarter_T)
            vx = self._speed

        pos = np.array([x, 0.0, 0.0])
        vel = np.array([vx, 0.0, 0.0])
        return pos, vel

    @property
    def period(self) -> float:
        return self._period

    @property
    def name(self) -> str:
        return "straight_line"
