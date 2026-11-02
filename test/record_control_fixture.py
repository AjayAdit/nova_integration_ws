#!/usr/bin/env python3
"""Record every guidance and NMPC step of the NOVA_PAMPC golden run.

Runs on a temporary copy, because sim_world.py rewrites Last_run/ where it runs.

  MUJOCO_GL=egl python record_control_fixture.py <NOVA_PAMPC dir> <out.npz>
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
    source, out = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
    workdir = tempfile.mkdtemp(prefix='nova_fixture_')
    copy = os.path.join(workdir, 'nova')
    shutil.copytree(source, copy, ignore=shutil.ignore_patterns('.git', 'log', 'practice'))
    os.chdir(copy)
    sys.path.insert(0, copy)

    import nmpc_controller
    import sim_world

    rows = []
    step = {}
    runner = None

    real_yaw = sim_world.Sim_Runner.update_yaw_to_car

    def update_yaw(self):
        yaw = real_yaw(self)
        box = self.perc_feed.last_bbox
        step['psi'] = sim_world.yaw_from_quat(self.data.joint('drone_root').qpos[3:7])
        step['u_px'] = np.nan if box is None else box[0] + box[2] / 2.0
        step['yaw'] = yaw
        step['t'] = self.data.time
        return yaw

    real_set = nmpc_controller.NMPC.set_target_trajectory

    def set_target(self, positions, velocities, yaws):
        step['positions'] = np.array(positions)
        step['velocities'] = np.array(velocities)
        step['yaws'] = np.array(yaws)
        real_set(self, positions, velocities, yaws)

    real_action = nmpc_controller.NMPC.compute_action

    def compute_action(self, x_meas, a_tgt):
        u = real_action(self, x_meas, a_tgt)
        rows.append((step['t'], step['psi'], step['u_px'], step['yaw'], step['positions'],
                     step['velocities'], step['yaws'], np.array(x_meas), np.array(a_tgt),
                     np.array(u), self.solver.status, self.solver.get(1, 'x')[10:13].copy()))
        return u

    sim_world.Sim_Runner.update_yaw_to_car = update_yaw
    nmpc_controller.NMPC.set_target_trajectory = set_target
    nmpc_controller.NMPC.compute_action = compute_action

    args = sim_world.parse_args(ARGV)
    runner = sim_world.Sim_Runner(args)
    runner.spawn_and_countdown()
    initial_yaw = runner.sv._frozen_pixel_yaw

    trace_path = os.path.join(workdir, 'trace.csv')
    with open(trace_path, 'w', newline='') as f:
        writer = csv.writer(f)
        try:
            runner.run(lambda t, x, u: writer.writerow([t, *x, *u]))
        except SystemExit:
            pass

    columns = list(map(np.array, zip(*rows)))
    names = ['t', 'psi', 'u_px', 'yaw', 'positions', 'velocities', 'yaws', 'x_meas',
             'a_tgt', 'u', 'status', 'omega_1']
    np.savez_compressed(out, initial_yaw=initial_yaw, fx=sim_world.FX, cx=sim_world.CX,
                        **dict(zip(names, columns)))
    shutil.copy(trace_path, os.path.splitext(out)[0] + '_trace.csv')
    shutil.rmtree(workdir)
    print(f'{len(rows)} steps; non-zero solver status: {int((columns[10] != 0).sum())}')


if __name__ == '__main__':
    main()
