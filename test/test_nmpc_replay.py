"""The C++ nmpc_node with the NOVA solver against the Python NMPC of the baseline.

Sends the recorded state, reference and target acceleration over real topics, one
step at a time, and compares the commands. Needs the NOVA build sourced.

Why the limits are not "match on every step": this closed-loop solver amplifies
rounding. The ORIGINAL Python NMPC, replayed with 1e-15 rad added to yaw, leaves
the recording near step 330 and reaches a difference of 1.0, with an RMS of 0.150.
Replayed unchanged it matches exactly, so this is not noise in the replay. No
controller can match step by step after that point, so this test requires:
  - a close match before the divergence starts
  - afterwards, no more difference than rounding alone causes in the original
Closed-loop tracking is what proves the rest (step 5).
"""
import os
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from actuator_msgs.msg import Actuators
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Transform, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

HERE = os.path.dirname(os.path.abspath(__file__))
DOMAIN_ID = '82'
COMMAND_TOLERANCE = 1e-6
STEPS_BEFORE_DIVERGENCE = 300
MAX_RMS_DIFFERENCE = 0.20    # 0.150-0.155 measured on the original with 1e-15, 1e-13 nudges


def build_reference(d, i, stamp):
    m = MultiDOFJointTrajectory()
    m.header.stamp.sec, m.header.stamp.nanosec = stamp
    m.header.frame_id = 'target'
    m.joint_names = ['base_link']
    for k in range(21):
        tf = Transform()
        tf.translation.x, tf.translation.y, tf.translation.z = map(float, d['positions'][i][k])
        yaw = float(d['yaws'][i][k])
        tf.rotation.w, tf.rotation.z = float(np.cos(yaw / 2)), float(np.sin(yaw / 2))
        vel = Twist()
        vel.linear.x, vel.linear.y, vel.linear.z = map(float, d['velocities'][i][k])
        point = MultiDOFJointTrajectoryPoint(transforms=[tf], velocities=[vel])
        point.time_from_start.nanosec = k * 50_000_000 % 1_000_000_000
        point.time_from_start.sec = k * 50_000_000 // 1_000_000_000
        m.points.append(point)
    return m


def build_odom(x, stamp):
    m = Odometry()
    m.header.stamp.sec, m.header.stamp.nanosec = stamp
    m.header.frame_id, m.child_frame_id = 'target', 'base_link'
    p, t = m.pose.pose, m.twist.twist
    p.position.x, p.position.y, p.position.z = map(float, x[:3])
    t.linear.x, t.linear.y, t.linear.z = map(float, x[3:6])
    p.orientation.w, p.orientation.x = float(x[6]), float(x[7])
    p.orientation.y, p.orientation.z = float(x[8]), float(x[9])
    t.angular.x, t.angular.y, t.angular.z = map(float, x[10:13])
    return m


def test_commands_match_python_nmpc():
    with np.load(os.path.join(HERE, 'data', 'baseline_control.npz')) as f:
        d = {k: f[k] for k in f.files}
    assert (d['status'] == 0).all()

    env = dict(os.environ, ROS_DOMAIN_ID=DOMAIN_ID)
    os.environ['ROS_DOMAIN_ID'] = DOMAIN_ID
    logdir = tempfile.mkdtemp(prefix='nmpc_replay_')
    proc = subprocess.Popen(
        [os.path.join(get_package_prefix('quad_nmpc_ros_cpp'), 'lib', 'quad_nmpc_ros_cpp', 'nmpc_node'),
         '--ros-args',
         '-p', 'mass:=1.25', '-p', 'k_tau:=20.0',
         '-p', 'reference_topic:=/nmpc/reference', '-p', 'target_accel_topic:=/target/accel',
         '-p', f'log_file:={logdir}/log.csv'], env=env)

    rclpy.init()
    node = rclpy.create_node('nmpc_test_harness')
    odom = node.create_publisher(Odometry, '/odom', 10)
    ref = node.create_publisher(MultiDOFJointTrajectory, '/nmpc/reference', 10)
    acc = node.create_publisher(Vector3Stamped, '/target/accel', 10)
    got, rates = [], []
    node.create_subscription(Actuators, '/motor_cmd', lambda m: got.append(np.array(m.normalized)), 10)
    node.create_subscription(
        Vector3Stamped, '/nmpc/body_rate_ref',
        lambda m: rates.append(np.array([m.vector.x, m.vector.y, m.vector.z])), 10)
    try:
        end = time.monotonic() + 20.0
        while not (odom.get_subscription_count() and ref.get_subscription_count()
                   and acc.get_subscription_count()):
            assert time.monotonic() < end, 'nmpc_node did not connect'
            rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.5)

        n = len(d['t'])
        solve_ms = []
        for i in range(n):
            stamp = divmod(1_000_000_000 + i * 10_000_000, 1_000_000_000)
            a = Vector3Stamped()
            a.header.stamp.sec, a.header.stamp.nanosec = stamp
            a.vector.x, a.vector.y, a.vector.z = map(float, d['a_tgt'][i])
            reference = build_reference(d, i, stamp)
            state = build_odom(d['x_meas'][i], stamp)
            t0 = time.perf_counter()
            acc.publish(a)
            ref.publish(reference)
            odom.publish(state)
            end = time.monotonic() + 5.0
            while len(got) <= i or len(rates) <= i:
                assert time.monotonic() < end, f'no command for step {i}'
                rclpy.spin_once(node, timeout_sec=0.01)
            solve_ms.append((time.perf_counter() - t0) * 1e3)

        if 'NMPC_REPLAY_DUMP' in os.environ:
            np.savez(os.environ['NMPC_REPLAY_DUMP'], got=np.array(got), rates=np.array(rates))
        u_err = np.abs(np.array(got) - d['u']).max(axis=1)
        w_err = np.abs(np.array(rates) - d['omega_1']).max(axis=1)
        early_u = u_err[:STEPS_BEFORE_DIVERGENCE].max()
        early_w = w_err[:STEPS_BEFORE_DIVERGENCE].max()
        rms = np.sqrt((u_err ** 2).mean())
        print(f'{n} steps: first {STEPS_BEFORE_DIVERGENCE}: worst command difference {early_u:.3e}, '
              f'worst body-rate difference {early_w:.3e} rad/s; whole run: RMS {rms:.3f}; '
              f'round trip mean {np.mean(solve_ms):.2f} ms, p99 {np.percentile(solve_ms, 99):.2f} ms')
        assert early_u < COMMAND_TOLERANCE
        assert early_w < COMMAND_TOLERANCE
        assert rms < MAX_RMS_DIFFERENCE
    finally:
        proc.terminate()
        proc.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()
