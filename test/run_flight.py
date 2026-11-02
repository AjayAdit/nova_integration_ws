#!/usr/bin/env python3
"""Fly the full pipeline once and score it against the agreed pass criterion.

  python run_flight.py <what-changed> [--motors] [--duration 20] [--seed 0] [--viewer]

Needs setup_ws.sh built and install/setup.bash sourced. Writes one post-analysis run.
Pass: mean 3D error at most 32 cm (baseline about 29 cm), and no lost detections.
"""
import argparse
import csv
import json
import math
import os
import re
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data  # noqa: F401
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import MultiDOFJointTrajectory
from vision_msgs.msg import Detection2DArray

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PA = os.path.join(ROOT, 'post_analysis', 'scripts', 'pa.py')
DOMAIN_ID = '83'
PASS_MEAN_ERROR_CM = 32.0
D_SAFE, Z_OFFSET = 1.0, 1.0


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class Recorder:

    def __init__(self, node):
        self.node = node
        self.sim_ns = 0
        self.truth, self.car, self.states = {}, {}, {}
        self.frames, self.positions, self.commands = [], [], []
        self.ref_flips = 0
        self.last_q = None
        self.first_wall = None
        self.first_sim = None
        self.last_wall = None
        node.create_subscription(Clock, '/clock', self._on_clock, 50)
        node.create_subscription(Odometry, '/ground_truth/odom', self._on_truth, 50)
        node.create_subscription(Odometry, '/ground_truth/target_odom', self._on_car, 50)
        node.create_subscription(Float64MultiArray, '/ukf/state', self._on_state, 100)
        node.create_subscription(Detection2DArray, '/perception/target_bbox', self._on_box, 20)
        node.create_subscription(PointStamped, '/perception/position', self._on_position, 20)
        node.create_subscription(Actuators, '/motor_cmd', self._on_command, 50)
        node.create_subscription(MultiDOFJointTrajectory, '/nmpc/reference', self._on_reference, 50)

    def _on_clock(self, msg):
        self.sim_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        now = time.monotonic()
        if self.first_wall is None:
            self.first_wall, self.first_sim = now, self.sim_ns
        self.last_wall = now

    def _on_truth(self, msg):
        p = msg.pose.pose.position
        self.truth[stamp_ns(msg)] = (p.x, p.y, p.z)

    def _on_car(self, msg):
        p = msg.pose.pose.position
        self.car[stamp_ns(msg)] = (p.x, p.y, p.z)

    def _on_state(self, msg):
        data = np.array(msg.data)
        self.states[int(round(data[0] * 1e9))] = data[1:]

    def _on_box(self, msg):
        self.frames.append((stamp_ns(msg), int(not msg.detections)))

    def _on_position(self, msg):
        self.positions.append((stamp_ns(msg), self.sim_ns - stamp_ns(msg)))

    def _on_command(self, msg):
        self.commands.append((stamp_ns(msg), self.sim_ns - stamp_ns(msg), list(msg.normalized)))

    def _on_reference(self, msg):
        r = msg.points[0].transforms[0].rotation
        q = np.array([r.w, r.z])
        if self.last_q is not None and np.dot(q, self.last_q) < 0.0:
            self.ref_flips += 1
        self.last_q = q


def write_results(rec, raw):
    state_times = np.array(sorted(rec.states))
    t_cmd = {s: (lat, u) for s, lat, u in rec.commands}

    def latest_state(t_ns):
        i = np.searchsorted(state_times, t_ns, side='right') - 1
        return None if i < 0 else rec.states[int(state_times[i])]

    errors, ukf_errors = [], []
    with open(os.path.join(raw, 'log.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'standoff_ref', 'standoff', 'height_ref', 'height', 'err_ref',
                    'err_3d_cm', 'ukf_err_cm', 'u0', 'u1', 'u2', 'u3',
                    'drone_x', 'drone_y', 'drone_z', 'car_x', 'car_y', 'car_z'])
        last_u = [math.nan] * 4
        for t_ns in sorted(rec.truth.keys() & rec.car.keys()):
            drone, car = np.array(rec.truth[t_ns]), np.array(rec.car[t_ns])
            standoff = np.linalg.norm(drone[:2] - car[:2])
            height = drone[2] - car[2]
            err = float(np.hypot((standoff - D_SAFE) * 100.0, (height - Z_OFFSET) * 100.0))
            state = latest_state(t_ns)
            ukf = '' if state is None else float(np.linalg.norm(state[:3] - (drone - car)) * 100.0)
            if t_ns in t_cmd:
                last_u = t_cmd[t_ns][1]
            errors.append(err)
            if ukf != '':
                ukf_errors.append(ukf)
            w.writerow([t_ns * 1e-9, D_SAFE, standoff, Z_OFFSET, height, 0.0, err, ukf,
                        *['' if math.isnan(v) else v for v in last_u], *drone, *car])
    with open(os.path.join(raw, 'frames.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_frame', 'lost_ref', 'lost'])
        for t_ns, lost in rec.frames:
            w.writerow([t_ns * 1e-9, 0.0, lost])
    with open(os.path.join(raw, 'latency.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_latency', 'latency_ref', 'position_latency_ms'])
        for t_ns, lat in rec.positions:
            w.writerow([t_ns * 1e-9, 0.0, lat * 1e-6])
    with open(os.path.join(raw, 'commands.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_command', 'command_ref', 'command_latency_ms'])
        for t_ns, lat, _ in rec.commands:
            w.writerow([t_ns * 1e-9, 0.0, lat * 1e-6])
    with open(os.path.join(raw, 'trace.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'p_T_x', 'p_T_y', 'p_T_z', 'v_T_x', 'v_T_y', 'v_T_z', 'q_w', 'q_x', 'q_y', 'q_z',
                    'w_x', 'w_y', 'w_z', 'b_a_x', 'b_a_y', 'b_a_z', 'b_g_x', 'b_g_y', 'b_g_z',
                    'u0', 'u1', 'u2', 'u3'])
        for t_ns, _, u in rec.commands:
            state = latest_state(t_ns)
            if state is not None:
                w.writerow([t_ns * 1e-9, *state, *u])

    rtf = (rec.sim_ns - rec.first_sim) * 1e-9 / max(rec.last_wall - rec.first_wall, 1e-9)
    lost = sum(v for _, v in rec.frames)
    latency = np.array([v for _, v in rec.positions]) * 1e-6
    cmd_latency = np.array([v for _, v, _ in rec.commands]) * 1e-6
    return {
        'mean_3d_error_cm': float(np.mean(errors)),
        'max_3d_error_cm': float(np.max(errors)),
        'mean_ukf_error_cm': float(np.mean(ukf_errors)),
        'frames': len(rec.frames),
        'lost_frames': int(lost),
        'positions': len(rec.positions),
        'commands': len(rec.commands),
        'position_latency_ms_mean': float(latency.mean()) if len(latency) else None,
        'position_latency_ms_p99': float(np.percentile(latency, 99)) if len(latency) else None,
        'command_latency_ms_mean': float(cmd_latency.mean()) if len(cmd_latency) else None,
        'command_latency_ms_p99': float(np.percentile(cmd_latency, 99)) if len(cmd_latency) else None,
        'real_time_factor': float(rtf),
        'reference_quaternion_flips': rec.ref_flips,
        'sim_end_s': rec.sim_ns * 1e-9,
    }


def verdict(summary, finished):
    summary['finished_by_itself'] = finished
    summary['passed'] = bool(finished and summary['mean_3d_error_cm'] <= PASS_MEAN_ERROR_CM
                             and summary['lost_frames'] == 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('name')
    ap.add_argument('--motors', action='store_true', help='drive motors directly, not through AcroMode')
    ap.add_argument('--duration', type=float, default=20.0)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--viewer', action='store_true')
    ap.add_argument('--speed', type=float, default=3.0, help='car speed, m/s')
    ap.add_argument('--true-state', action='store_true', help='TEST ONLY: true state replaces the UKF')
    args = ap.parse_args()

    out = subprocess.run([sys.executable, PA, 'new', args.name, '--changed',
                          f'{args.name} ({"motors" if args.motors else "acro"}, seed {args.seed})'],
                         check=True, capture_output=True, text=True, cwd=ROOT).stdout
    run_dir = os.path.join(ROOT, re.search(r'Created (\S+)', out).group(1))
    raw = os.path.join(run_dir, 'raw')

    os.environ['ROS_DOMAIN_ID'] = DOMAIN_ID
    logfile = open(os.path.join(raw, 'launch.log.txt'), 'w')
    launch = subprocess.Popen(
        ['ros2', 'launch', os.path.join(ROOT, 'launch', 'nova.launch.py'),
         f'acro_mode:={"false" if args.motors else "true"}', f'duration:={args.duration}',
         f'viewer:={"true" if args.viewer else "false"}',
         f'detection_view:={"true" if args.viewer else "false"}', f'seed:={args.seed}',
         f'true_state:={"true" if args.true_state else "false"}', f'target_speed:={args.speed}',
         'record:=false',
         f'nmpc_log:={os.path.join(raw, "nmpc_log.txt")}'],
        stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)

    rclpy.init()
    node = rclpy.create_node('flight_recorder')
    rec = Recorder(node)
    start = time.monotonic()
    wall_limit = args.duration * 6.0 + 60.0
    try:
        while launch.poll() is None and time.monotonic() - start < wall_limit:
            rclpy.spin_once(node, timeout_sec=0.01)
        finished = launch.poll() is not None
    finally:
        if launch.poll() is None:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
        node.destroy_node()
        rclpy.shutdown()
        logfile.close()

    if not rec.commands or not rec.truth:
        sys.exit(f'FAILED: no flight data; see {raw}/launch.log.txt')

    summary = write_results(rec, raw)
    verdict(summary, finished)
    with open(os.path.join(run_dir, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=2)
    subprocess.run([sys.executable, PA, 'analyze', os.path.basename(run_dir)],
                   check=True, capture_output=True, cwd=ROOT)
    subprocess.run([sys.executable, os.path.join(ROOT, 'post_analysis', 'scripts', 'plot_flight.py'), run_dir],
                   check=True, capture_output=True, cwd=ROOT)
    print(os.path.basename(run_dir))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
