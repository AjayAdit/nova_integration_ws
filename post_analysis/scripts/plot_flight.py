#!/usr/bin/env python3
"""One flight on one page: tracking error over time, and expected against actual in 3D.

  python plot_flight.py <run folder> [<run folder> ...]   writes both plots below

plots/trajectory_3d.png shows the 3D error, and plots/trajectory_2d.png the 2D error,
which ignores height: the horizontal distance from the car against the 1 m goal.

Expected is the goal the score uses: 1 m from the car along the current horizontal bearing,
1 m above it. The distance from actual to expected is therefore exactly the 3D error.
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d import Axes3D  # noqa: E402,F401
from matplotlib.ticker import MultipleLocator  # noqa: E402
import numpy as np  # noqa: E402

D_SAFE, Z_OFFSET = 1.0, 1.0
ACTUAL, EXPECTED, CAR = '#2a78d6', '#eb6834', '#8a8983'
SETTLE_S = 2.0


def load(run):
    d = np.genfromtxt(os.path.join(run, 'raw', 'log.csv'), delimiter=',', names=True)
    drone = np.c_[d['drone_x'], d['drone_y'], d['drone_z']]
    car = np.c_[d['car_x'], d['car_y'], d['car_z']]
    away = drone[:, :2] - car[:, :2]
    unit = away / np.maximum(np.linalg.norm(away, axis=1, keepdims=True), 1e-9)
    expected = np.c_[car[:, :2] + D_SAFE * unit, car[:, 2] + Z_OFFSET]
    error_3d = np.linalg.norm(drone - expected, axis=1) * 100.0
    error_2d = np.linalg.norm(drone[:, :2] - expected[:, :2], axis=1) * 100.0
    return d['t'], drone, expected, car, error_3d, error_2d


def plot(run, kind):
    t, drone, expected, car, error_3d, error_2d = load(run)
    error = error_3d if kind == '3D' else error_2d
    late = t >= SETTLE_S
    name = os.path.basename(run.rstrip('/'))

    fig = plt.figure(figsize=(14, 6.4))
    fig.suptitle(f'Expected vs actual trajectory: {name}  ({t[-1]:.0f} s)')
    left = fig.add_subplot(1, 2, 1)
    left.plot(t, error, color=ACTUAL, linewidth=2, label=f'{kind} tracking error')
    left.axhline(error.mean(), color=EXPECTED, linestyle='--', linewidth=1.5,
                 label=f'mean {error.mean():.2f} cm')
    left.set_xlabel('time [s]')
    left.set_ylabel(f'{kind} tracking error [cm]')
    left.set_title(f'{kind} tracking error')
    left.set_xlim(0, t[-1])
    left.set_ylim(bottom=0)
    left.grid(color='#e4e3df', linewidth=0.8)
    left.set_axisbelow(True)
    left.spines[['top', 'right']].set_visible(False)
    left.legend(frameon=True, loc='upper right')
    left.text(0.5, -0.2,
              f'whole flight: max {error.max():.2f} cm, mean {error.mean():.2f} cm\n'
              f'after {SETTLE_S:.0f} s:    max {error[late].max():.2f} cm, mean {error[late].mean():.2f} cm',
              transform=left.transAxes, ha='center', va='top', family='monospace', fontsize=10,
              bbox=dict(boxstyle='square', facecolor='white', edgecolor='#c9c8c3'))

    right = fig.add_subplot(1, 2, 2, projection='3d')
    right.plot(*car.T, color=CAR, linewidth=1.2, label='car')
    right.plot(*expected.T, color=EXPECTED, linestyle='--', linewidth=1.5, label='expected')
    right.plot(*drone.T, color=ACTUAL, linewidth=2, label='actual')
    right.set_xlabel('x [m]')
    right.set_ylabel('y [m]')
    right.set_zlabel('z [m]')
    right.set_title('3D')
    right.zaxis.set_major_locator(MultipleLocator(0.5))
    right.set_box_aspect((np.ptp(drone[:, 0]), np.ptp(drone[:, 1]), max(np.ptp(drone[:, 2]) * 2, 1.0)))
    right.legend(frameon=True, loc='upper left')

    path = os.path.join(run, 'plots', f'trajectory_{kind.lower()}.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'{name} {kind}: mean {error.mean():.2f} cm, max {error.max():.2f} cm -> {path}')


if __name__ == '__main__':
    for arg in sys.argv[1:]:
        plot(arg, '3D')
        plot(arg, '2D')
