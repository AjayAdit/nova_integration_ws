import numpy as np
from .base import BaseTrajectory


class SpeedRampTrajectory(BaseTrajectory):
    """
    Time-warp wrapper: walks any trajectory's geometric path at a speed that
    eases from rest to full over `ramp` seconds.  The path is untouched; only
    the rate at which phase advances along it changes.

        sigma(t) = 0.5 * (1 - cos(pi t / T))     speed factor, 0 -> 1
        phi(t)   = integral of sigma             warped phase
        pos(t)   = raw_pos(phi(t))
        vel(t)   = raw_vel(phi(t)) * sigma(t)

    vel is exactly d(pos)/dt by construction, so the mocap motion, the UKF
    ground truth and the telemetry can never disagree.  sigma has zero slope
    at both ends, so there is no acceleration step at t=0 or at t=T.

    Peak acceleration is v_cruise * pi / (2T), reached at t = T/2.

    After the ramp the phase keeps a permanent lag of T/2.  It is never caught
    up: doing so would need an overspeed burst, which is the exact disturbance
    this class exists to remove.

    Only instantiated when the ramp is strictly positive -- see sim_world.py,
    which leaves the raw trajectory untouched otherwise.
    """

    def __init__(self, raw, ramp: float):
        self._raw  = raw
        self._ramp = float(ramp)
        self._last_heading = 0.0
        print(f" SpeedRamp: target eases 0 -> full speed over {self._ramp:.2f} s "
              f"(phase lag {self._ramp / 2:.2f} s)")

    def _sigma(self, t):
        if t >= self._ramp:
            return 1.0
        if t <= 0.0:
            return 0.0
        return 0.5 * (1.0 - np.cos(np.pi * t / self._ramp))

    def _phase(self, t):
        T = self._ramp
        if t <= 0.0:
            return 0.0
        if t >= T:
            return t - 0.5 * T
        return 0.5 * (t - (T / np.pi) * np.sin(np.pi * t / T))

    def __call__(self, t: float):
        pos, vel = self._raw(self._phase(t))
        return (np.asarray(pos, dtype=float),
                np.asarray(vel, dtype=float) * self._sigma(t))

    def heading(self, t: float) -> float:
        """Path-tangent yaw taken from the RAW velocity, which stays valid
        while sigma(t) is 0 and the ramped velocity has no direction."""
        _, vel = self._raw(self._phase(t))
        if np.hypot(vel[0], vel[1]) > 1e-9:
            self._last_heading = float(np.arctan2(vel[1], vel[0]))
        return self._last_heading

    @property
    def period(self) -> float:
        return self._raw.period

    @property
    def name(self) -> str:
        return f"{self._raw.name}+ramp"

    @property
    def raw(self):
        """Un-warped trajectory -- use for drawing the static path geometry."""
        return self._raw
