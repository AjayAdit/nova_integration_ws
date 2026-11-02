#!/usr/bin/env python3
"""Tracking error of the ROS 2 flights against the Python baseline. Writes a table and a plot.

Only valid flights after the startup fix are used. All are compared over t = 0 to 20 s.
"""
import glob
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GROUPS = {
    'Python baseline': ['run01'],
    'ROS 2 motors': ['run05', 'run11', 'run12'],
    'ROS 2 AcroMode': ['run06', 'run07', 'run08', 'run09', 'run10', 'run13', 'run14', 'run15',
                       'nowin-1', 'nowin-2', 'nowin-3', 'nowin-4', 'final-nowindows-2'],
    'ROS 2 AcroMode, windows on': ['windows-check', 'windows-check-2', 'win-1', 'win-3', 'win-4',
                                   'final-windows-2', 'yellow-path'],
    'True state, motors': ['truth-motors-1', 'truth-motors-2', 'truth-motors-3'],
    'True state, AcroMode': ['truth-acro-1', 'truth-acro-2', 'truth-acro-3'],
}
PLOTTED = {'Python baseline': '#eb6834', 'ROS 2 motors': '#2a78d6', 'ROS 2 AcroMode': '#1baf7a'}
TRUTH_PLOT = {'Python baseline': '#eb6834', 'ROS 2 AcroMode': '#1baf7a', 'True state, AcroMode': '#7c5cd6'}
WINDOWS = [(0, 5), (5, 10), (10, 15), (15, 20)]
GRID = np.arange(0.0, 20.0, 0.01)
D_SAFE = 1.0
AT_2MS = {'Python baseline, 2 m/s': ['baseline-2ms'], 'ROS 2 AcroMode, 2 m/s': ['ukf-acro-2ms']}


def load(token, column='err_3d_cm'):
    pattern = f'*_{token}_*' if token.startswith('run') else f'*_{token}*'
    hits = [p for p in glob.glob(os.path.join(ROOT, 'runs', pattern))
            if os.path.basename(p).split('_', 2)[2].startswith(token) or token.startswith('run')]
    hits = [p for p in hits if 'windows-check-2' not in p or token == 'windows-check-2']
    assert len(hits) == 1, (token, hits)
    data = np.genfromtxt(os.path.join(hits[0], 'raw', 'log.csv'), delimiter=',', names=True)
    return np.interp(GRID, data['t'], data[column])


def load_2d(token):
    """Horizontal error: distance from the car against the 1 m goal, ignoring height."""
    return np.abs(load(token, 'standoff') - D_SAFE) * 100.0


def span(values, digits=2):
    if len(values) == 1:
        return f'{values[0]:.{digits}f}'
    return f'{np.mean(values):.{digits}f} ({np.min(values):.{digits}f} to {np.max(values):.{digits}f})'


def plot(flights, series, title, filename):
    fig, ax = plt.subplots(figsize=(10, 4.6))
    ax.grid(color='#e4e3df', linewidth=0.8)
    ax.set_axisbelow(True)
    for name, color in series.items():
        err = flights[name]
        if len(err) > 1:
            ax.fill_between(GRID, err.min(axis=0), err.max(axis=0), color=color, alpha=0.15, linewidth=0)
        ax.plot(GRID, err.mean(axis=0), color=color, linewidth=2,
                linestyle='--' if name == 'Python baseline' else '-',
                label=f'{name} ({len(err)} flight{"s" if len(err) > 1 else ""})')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('3D tracking error [cm]')
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 80)   # headroom so the legend clears the peak at t = 5.5 s
    ax.set_title(title,
                 fontsize=11, loc='left')
    ax.legend(frameon=False, loc='upper right', ncol=3)
    ax.spines[['top', 'right']].set_visible(False)
    fig.savefig(os.path.join(ROOT, filename), dpi=150, bbox_inches='tight')



def main():
    flights = {name: np.array([load(t) for t in tokens]) for name, tokens in GROUPS.items()}
    base = flights['Python baseline'][0]

    lines = ['# Tracking error against the Python baseline', '',
             'Lissajous at 3 m/s, t = 0 to 20 s. Cells are the mean over flights (lowest to highest flight).', '',
             '| Setup | Flights | Mean 3D error (cm) | Change from baseline (cm) | RMSE (cm) | Peak (cm) | Last 2 s (cm) |',
             '|---|---|---|---|---|---|---|']
    for name, err in flights.items():
        means = err.mean(axis=1)
        lines.append(f'| {name} | {len(err)} | {span(means)} | {means.mean() - base.mean():+.2f} | '
                     f'{span(np.sqrt((err ** 2).mean(axis=1)))} | {span(err.max(axis=1), 1)} | '
                     f'{span(err[:, GRID >= 18].mean(axis=1))} |')
    lines += ['', '## Mean error by flight phase (cm)', '',
              '| Setup | ' + ' | '.join(f'{a} to {b} s' for a, b in WINDOWS) + ' |',
              '|---|' + '---|' * len(WINDOWS)]
    for name, err in flights.items():
        cells = [f'{err[:, (GRID >= a) & (GRID < b)].mean():.1f}' for a, b in WINDOWS]
        lines.append(f'| {name} | ' + ' | '.join(cells) + ' |')
    lines += ['', '## 2D and 3D tracking error (cm)', '',
              '2D ignores height. The 3 m/s rows are as above; the 2 m/s rows are one flight each.', '',
              '| Setup | Flights | Mean 2D | Max 2D | Mean 3D | Max 3D |', '|---|---|---|---|---|---|']
    for name, tokens in {**GROUPS, **AT_2MS}.items():
        flat = np.array([load_2d(t) for t in tokens])
        full = np.array([load(t) for t in tokens])
        lines.append(f'| {name} | {len(tokens)} | {flat.mean():.2f} | {flat.max(axis=1).mean():.1f} | '
                     f'{full.mean():.2f} | {full.max(axis=1).mean():.1f} |')
    lines += ['', '![comparison](comparison_baseline.png)', '',
              '![true state](comparison_true_state.png)', '']
    text = '\n'.join(lines)
    with open(os.path.join(ROOT, 'comparison_baseline.md'), 'w') as f:
        f.write(text)
    print(text)

    plot(flights, PLOTTED, 'Tracking error against the Python baseline (lines: mean; bands: lowest to highest flight)',
         'comparison_baseline.png')
    plot(flights, TRUTH_PLOT, 'What the estimate costs: UKF against true state (lines: mean; bands: lowest to highest flight)',
         'comparison_true_state.png')

if __name__ == '__main__':
    main()
