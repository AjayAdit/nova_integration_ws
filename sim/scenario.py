"""The baseline scenario: target path and spawn pose. Shared by the sim and the launch file."""
import numpy as np
from types import SimpleNamespace

from trajectories import REGISTRY, SpeedRampTrajectory


def make_trajectory(name='lissajous', target_speed=3.0, ramp=5.0, amplitude=None):
    raw = REGISTRY[name](SimpleNamespace(
        target_speed=target_speed, amplitude=amplitude, wavelength=None,
        rec_side_start=False))
    return SpeedRampTrajectory(raw, ramp=ramp)


def path_points(trajectory, t_end, count):
    """Points along the car's path from t=0 to t_end, for the viewer marker."""
    return np.array([trajectory(t)[0] for t in np.linspace(0.0, t_end, count)])


def spawn_offset(trajectory):
    """Drone position relative to the car at t=0: behind it, 1 m up. Same as sim_world.py."""
    yaw = trajectory.heading(0.0)
    return np.array([-1.2 * np.cos(yaw), -1.2 * np.sin(yaw), 1.0])


def target_accel(trajectory, t, h=1e-4):
    """True target acceleration: central difference, same as sim_world.py."""
    t0 = max(t - h, 0.0)
    t1 = t + h
    return (trajectory(t1)[1] - trajectory(t0)[1]) / (t1 - t0)
