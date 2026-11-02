#!/usr/bin/env python3
"""Record the Python NOVA_PAMPC pipeline as the post-analysis baseline run.

Runs on a temporary copy, because sim_world.py rewrites Last_run/ where it runs.

  MUJOCO_GL=egl python record_baseline_run.py <NOVA_PAMPC dir> <run dir>/raw [speed]
"""
import csv
import os
import shutil
import sys
import tempfile

import numpy as np

ARGV = ['--fast', '--countdown', '0', '--duration', '20', '--target-speed', '3',
        '--trajectory', 'lissajous', '--q-pixel', '250', '--q-pixel-vel', '0',
        '--ramp', '5', '--oracle-accel-ukf', '--oracle-accel-mpc']


def main():
    source, raw = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
    if len(sys.argv) > 3:
        ARGV[ARGV.index('--target-speed') + 1] = sys.argv[3]
    workdir = tempfile.mkdtemp(prefix='nova_baseline_')
    copy = os.path.join(workdir, 'nova')
    shutil.copytree(source, copy, ignore=shutil.ignore_patterns('.git', 'log', 'practice'))
    os.chdir(copy)
    sys.path.insert(0, copy)

    import perception_feed
    import sim_world

    truth = {}
    real_log = sim_world.log_tracking_errors

    def log_tracking_errors(t, drone, car, d_safe, z_offset):
        truth[t] = (np.array(drone), np.array(car))
        return real_log(t, drone, car, d_safe, z_offset)

    sim_world.log_tracking_errors = log_tracking_errors

    frames = []
    real_measure = perception_feed.PerceptionFeed.get_measurement

    def get_measurement(self, data):
        result = real_measure(self, data)
        frames.append((data.time, int(self.last_bbox is None)))
        return result

    perception_feed.PerceptionFeed.get_measurement = get_measurement

    runner = sim_world.Sim_Runner(sim_world.parse_args(ARGV))
    runner.spawn_and_countdown()
    frames.clear()

    rows = []
    try:
        runner.run(lambda t, x, u: rows.append((t, np.array(x), np.array(u))))
    except SystemExit:
        pass

    os.makedirs(raw, exist_ok=True)
    with open(os.path.join(raw, 'log.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'standoff_ref', 'standoff', 'height_ref', 'height', 'err_ref',
                    'err_3d_cm', 'ukf_err_cm', 'u0', 'u1', 'u2', 'u3',
                    'drone_x', 'drone_y', 'drone_z', 'car_x', 'car_y', 'car_z'])
        for t, x, u in rows:
            drone, car = truth[t]
            standoff = np.linalg.norm(drone[:2] - car[:2])
            height = drone[2] - car[2]
            err = np.hypot((standoff - 1.0) * 100.0, (height - 1.0) * 100.0)
            ukf = np.linalg.norm(x[:3] - (drone - car)) * 100.0
            w.writerow([t, 1.0, standoff, 1.0, height, 0.0, err, ukf, *u, *drone, *car])
    with open(os.path.join(raw, 'frames.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_frame', 'lost_ref', 'lost'])
        for t, lost in frames:
            w.writerow([t, 0.0, lost])
    with open(os.path.join(raw, 'trace.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'p_T_x', 'p_T_y', 'p_T_z', 'v_T_x', 'v_T_y', 'v_T_z', 'q_w', 'q_x', 'q_y', 'q_z',
                    'w_x', 'w_y', 'w_z', 'b_a_x', 'b_a_y', 'b_a_z', 'b_g_x', 'b_g_y', 'b_g_z',
                    'u0', 'u1', 'u2', 'u3'])
        for t, x, u in rows:
            w.writerow([t, *x, *u])
    shutil.rmtree(workdir)
    print(f'{len(rows)} control steps, {len(frames)} frames')


if __name__ == '__main__':
    main()
