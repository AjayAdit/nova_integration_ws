"""guidance.py, fed the recorded inputs, must reproduce sim_world.py exactly."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'sim'))
from guidance import Guidance  # noqa: E402


def load():
    with np.load(os.path.join(HERE, 'data', 'baseline_control.npz')) as f:
        return {k: f[k] for k in f.files}


def test_matches_baseline_exactly():
    d = load()
    g = Guidance(float(d['fx']), float(d['cx']), float(d['initial_yaw']))
    for i in range(len(d['t'])):
        u = None if np.isnan(d['u_px'][i]) else float(d['u_px'][i])
        yaw = g.yaw_to_target(float(d['psi'][i]), u)
        assert yaw == d['yaw'][i], f'step {i}'
        positions, velocities, yaws = g.horizon(d['x_meas'][i][:3], yaw, float(d['t'][i]))
        assert np.array_equal(positions, d['positions'][i]), f'step {i}'
        assert np.array_equal(velocities, d['velocities'][i]), f'step {i}'
        assert np.array_equal(yaws, d['yaws'][i]), f'step {i}'
    print(f'{len(d["t"])} steps matched exactly')


def test_yaw_freezes_without_detection():
    g = Guidance(432.97, 320.0, initial_yaw=0.7)
    assert g.yaw_to_target(0.1, None) == 0.7
    first = g.yaw_to_target(0.1, 400.0)
    assert g.yaw_to_target(2.0, None) == first


def test_goal_keeps_last_direction_when_on_top_of_target():
    g = Guidance(432.97, 320.0, 0.0)
    g.shifted_goal(np.array([0.0, 2.0, 1.0]))
    goal = g.shifted_goal(np.array([0.0, 0.0, 1.0]))
    assert np.array_equal(goal, [0.0, 1.0, 1.0])


def test_ramp_starts_at_drone_and_ends_at_goal():
    g = Guidance(432.97, 320.0, 0.0)
    p = np.array([-1.2, 0.0, 0.5])
    assert np.array_equal(g.blended_goal(p, 0.0), p)
    assert np.array_equal(g.blended_goal(p, 1.0), [-1.0, 0.0, 1.0])
