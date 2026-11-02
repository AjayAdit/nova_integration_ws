from abc import ABC, abstractmethod
import numpy as np


class BaseTrajectory(ABC):
    """
    Common interface for all target trajectories.
    A trajectory is callable:  pos, vel = trajectory(t)
    — identical signature to the original lissajous_target(t) so that
    draw_trail(), make_history_traj_fn(), and the sim loop all work unchanged.
    """

    @abstractmethod
    def __call__(self, t: float):
        """Return (pos_3d, vel_3d) at simulation time t (seconds)."""

    @property
    @abstractmethod
    def period(self) -> float:
        """Duration of one full loop in seconds.
        Used by draw_trail() and the drone-history time window."""

    @property
    @abstractmethod
    def name(self) -> str:
        """Short lowercase label shown by --list."""
