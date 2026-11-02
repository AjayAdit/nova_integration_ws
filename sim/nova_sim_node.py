#!/usr/bin/env python3
"""NOVA test sim: MuJoCo world, moving car, sensors and temporary guidance.

Topics are defined in ../INTERFACES.md. Every stamp is data.time. It holds no estimator or
controller, and only guidance.py is control logic; that must move out before hardware.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from actuator_msgs.msg import Actuators  # noqa: E402
from ament_index_python.packages import get_package_prefix  # noqa: E402
from geometry_msgs.msg import QuaternionStamped, Transform, Twist, Vector3Stamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, Imu  # noqa: E402
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint  # noqa: E402
from vision_msgs.msg import Detection2DArray  # noqa: E402

from guidance import Guidance  # noqa: E402
from scenario import make_trajectory, path_points, spawn_offset, target_accel  # noqa: E402

CAM_NAME = 'rgbd_cam'
IMG_W, IMG_H = 640, 480
FOVY_DEG = 58.0
FY = (IMG_H / 2.0) / np.tan(np.deg2rad(FOVY_DEG) / 2.0)
FX = FY
CX, CY = IMG_W / 2.0, IMG_H / 2.0

STEPS_PER_TICK = 1   # callbacks only run between ticks, so more than 1 delays every command
IMU_EVERY = 2
CONTROL_EVERY = 5
PERCEPTION_EVERY = 50
ACCEL_Z_EXTRA_NOISE = np.sqrt(1.0 ** 2 - 0.1 ** 2)   # same as sim_world.py
HORIZON_DT_NS = 50_000_000
VIEWER_EVERY_TICKS = 8   # about 60 frames per second
PATH_POINTS = 601
PATH_T_END_DEFAULT = 30.0   # s, used when the flight has no set duration
PATH_WIDTH = 0.015          # m
PATH_LIFT = 0.01            # m above the car's path, so the floor does not hide it
PATH_RGBA = np.array([1.0, 0.9, 0.0, 0.9], dtype=np.float32)
# Visible size per metre of camera distance, for the viewer's 45 degree view at 16:9.
VIEW_HEIGHT_PER_M = 0.83
VIEW_WIDTH_PER_M = 1.47
VIEW_MARGIN = 1.5


def set_stamp(header, t_ns):
    header.stamp.sec, header.stamp.nanosec = divmod(t_ns, 1_000_000_000)


def yaw_from_quat(q):
    qw, qx, qy, qz = q
    return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))


class NovaSim(Node):

    def __init__(self):
        super().__init__('nova_sim_node')
        self.acro = self.declare_parameter('acro_mode', True).value
        self.duration = self.declare_parameter('duration', 0.0).value
        use_viewer = self.declare_parameter('viewer', False).value
        seed = self.declare_parameter('seed', 0).value
        speed = self.declare_parameter('target_speed', 3.0).value
        ramp = self.declare_parameter('ramp', 5.0).value
        self.use_target_accel = self.declare_parameter('use_target_accel', True).value
        self.image_subscribers = self.declare_parameter('image_subscribers', 1).value
        self.true_state = self.declare_parameter('true_state', False).value

        if self.acro:
            mujoco.mj_loadPluginLibrary(os.path.join(
                get_package_prefix('mujoco_ros_utils'), 'lib', 'libMujocoRosUtilsPlugin.so'))
        world = 'nova_world_acro.xml' if self.acro else 'nova_world.xml'
        self.model = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'worlds', world))
        self.data = mujoco.MjData(self.model)
        self.get_logger().info(f'world: {world}')

        self.drone_body = self.model.body('quadrotor').id
        self.car_body = self.model.body('tracking_target').id
        self.car_mocap = self.model.body_mocapid[self.car_body]
        self.joint = self.data.joint('drone_root')
        self.k_tau = None if self.acro else float(self.model.actuator_ctrlrange[0, 1])
        self.u = np.zeros(4)

        self.trajectory = make_trajectory(target_speed=speed, ramp=ramp)
        self.rng = np.random.default_rng(seed)
        self.renderer = mujoco.Renderer(self.model, height=IMG_H, width=IMG_W)

        initial_yaw = self._spawn()
        self.guidance = Guidance(FX, CX, initial_yaw)
        self.box_u = None
        self.step_idx = 0
        self.next_frame_step = 0
        self.started = False
        self.ticks_since_draw = 0
        self.first_command = False
        self.sensors_sent = False
        self.ready_since = None
        self.last_car_pos = np.array(self.data.mocap_pos[self.car_mocap])

        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 1)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 1)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 1)
        self.imu_pub = self.create_publisher(Imu, '/imu', 100)
        self.att_pub = self.create_publisher(QuaternionStamped, '/drone/attitude', 100)
        self.accel_pub = self.create_publisher(Vector3Stamped, '/target/accel', 10)
        self.ref_pub = self.create_publisher(MultiDOFJointTrajectory, '/nmpc/reference', 10)
        self.truth_pub = self.create_publisher(Odometry, '/ground_truth/odom', 10)
        self.car_pub = self.create_publisher(Odometry, '/ground_truth/target_odom', 10)

        if self.true_state:
            self.get_logger().warning('TEST ONLY: true state replaces the UKF')
            self.true_odom_pub = self.create_publisher(Odometry, '/ukf/odom', 10)
            self.true_state_pub = self.create_publisher(Float64MultiArray, '/ukf/state', 100)
        self.create_subscription(Odometry, '/ukf/odom', self._on_ukf_odom, 10)
        self.create_subscription(Detection2DArray, '/perception/target_bbox', self._on_bbox, 10)
        self.create_subscription(Actuators, '/motor_cmd', self._on_motor_cmd, 10)

        self.viewer = None
        if use_viewer:
            self.viewer = mujoco.viewer.launch_passive(
                self.model, self.data, show_left_ui=False, show_right_ui=False)
        if self.viewer is not None:
            self._draw_path()
        self.create_timer(self.model.opt.timestep * STEPS_PER_TICK, self._tick)

    def _draw_path(self):
        """Draw the car's path in the viewer only. The camera images never contain it."""
        t_end = self.duration if self.duration > 0.0 else PATH_T_END_DEFAULT
        points = path_points(self.trajectory, t_end, PATH_POINTS) + [0.0, 0.0, PATH_LIFT]
        scene = self.viewer.user_scn
        with self.viewer.lock():
            scene.ngeom = 0
            for start, end in zip(points[:-1], points[1:]):
                geom = scene.geoms[scene.ngeom]
                mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3),
                                    np.zeros(9), PATH_RGBA)
                mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, PATH_WIDTH, start, end)
                scene.ngeom += 1
            # Start with the whole path in view; the mouse can still move the camera.
            low, high = points.min(axis=0), points.max(axis=0)
            span = high - low
            cam = self.viewer.cam
            cam.lookat[:] = (low + high) / 2.0
            cam.distance = max(span[0] / VIEW_WIDTH_PER_M, span[1] / VIEW_HEIGHT_PER_M) * VIEW_MARGIN
            cam.azimuth, cam.elevation = 90.0, -70.0
        self.viewer.sync()

    def _spawn(self):
        pos, _ = self.trajectory(0.0)
        car_yaw = self.trajectory.heading(0.0)
        self.data.mocap_pos[self.car_mocap] = pos
        self.data.mocap_quat[self.car_mocap] = [np.cos(car_yaw / 2), 0, 0, np.sin(car_yaw / 2)]
        offset = spawn_offset(self.trajectory)
        self.joint.qpos[0:3] = pos + offset
        yaw = np.arctan2(-offset[1], -offset[0])
        self.joint.qpos[3:7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        mujoco.mj_forward(self.model, self.data)
        return yaw

    # --- inputs -----------------------------------------------------------------------
    def _on_motor_cmd(self, msg):
        self.u = np.asarray(msg.normalized, dtype=float)
        self.first_command = True

    def _on_bbox(self, msg):
        self.box_u = msg.detections[0].bbox.center.position.x if msg.detections else None

    def _on_ukf_odom(self, msg):
        p = msg.pose.pose.position
        p_T = np.array([p.x, p.y, p.z])
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        psi = yaw_from_quat(self.joint.qpos[3:7])
        yaw = self.guidance.yaw_to_target(psi, self.box_u)
        positions, velocities, yaws = self.guidance.horizon(p_T, yaw, t)

        ref = MultiDOFJointTrajectory()
        ref.header.stamp = msg.header.stamp
        ref.header.frame_id = 'target'
        ref.joint_names = ['base_link']
        for k in range(len(positions)):
            tf = Transform()
            tf.translation.x, tf.translation.y, tf.translation.z = map(float, positions[k])
            tf.rotation.w, tf.rotation.z = float(np.cos(yaws[k] / 2)), float(np.sin(yaws[k] / 2))
            vel = Twist()
            vel.linear.x, vel.linear.y, vel.linear.z = map(float, velocities[k])
            point = MultiDOFJointTrajectoryPoint(transforms=[tf], velocities=[vel])
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(
                k * HORIZON_DT_NS, 1_000_000_000)
            ref.points.append(point)
        self.ref_pub.publish(ref)

    # --- stepping ---------------------------------------------------------------------
    def _everyone_ready(self):
        # Without the UKF, attitude and target acceleration each lose one listener and the IMU has none.
        ukf_listeners = 0 if self.true_state else 1
        ready = (self.imu_pub.get_subscription_count() >= ukf_listeners
                 and self.att_pub.get_subscription_count() >= 1 + ukf_listeners
                 and self.color_pub.get_subscription_count() >= self.image_subscribers
                 and self.depth_pub.get_subscription_count()
                 and self.ref_pub.get_subscription_count()
                 and self.count_publishers('/ukf/odom') and self.count_publishers('/motor_cmd')
                 and self.count_publishers('/perception/target_bbox'))
        if ready and self.use_target_accel:
            ready = self.accel_pub.get_subscription_count() >= 1 + ukf_listeners
        return bool(ready)

    def _tick(self):
        if not self.started:
            if not self._everyone_ready():
                self.ready_since = None
                return
            now = self.get_clock().now().nanoseconds
            if self.ready_since is None:
                self.ready_since = now
            if now - self.ready_since < 1_000_000_000:
                return
            self.started = True
            self.get_logger().info('all nodes connected, starting physics')

        if self.duration > 0.0 and self.data.time >= self.duration:
            self.get_logger().info(f'reached duration {self.duration:.1f} s, stopping')
            raise SystemExit

        if self.viewer is None:
            for _ in range(STEPS_PER_TICK):
                self._physics_step()
            return
        # The viewer draws from its own thread, so it must not see a half-finished step.
        with self.viewer.lock():
            for _ in range(STEPS_PER_TICK):
                self._physics_step()
        self.ticks_since_draw += 1
        if self.ticks_since_draw >= VIEWER_EVERY_TICKS and self.viewer.is_running():
            self.ticks_since_draw = 0
            self.viewer.sync()

    def _physics_step(self):
        # Hold at t=0 until the first command is back. Like the baseline, the first
        # camera update and the first command then both happen before any physics.
        if self.sensors_sent and not self.first_command:
            return
        if not self.sensors_sent:
            self._publish_sensors()
            self.sensors_sent = True
            if not self.first_command:
                return
        if not self.acro:
            self.data.ctrl[:] = self.k_tau * self.u ** 2
        self.step_idx += 1
        mujoco.mj_step(self.model, self.data)
        self.sensors_sent = False

    def _publish_sensors(self):
        t_ns = int(round(self.data.time * 1e9))
        if self.step_idx >= self.next_frame_step:
            self._publish_frame(t_ns)
            self.next_frame_step += PERCEPTION_EVERY
        if self.step_idx % IMU_EVERY == 0:
            self._publish_imu(t_ns)
        if self.step_idx % CONTROL_EVERY == 0:
            self._drive_car()
            self._publish_truth(t_ns)
            if self.true_state:
                self._publish_true_state(t_ns)

    def _drive_car(self):
        pos, _ = self.trajectory(self.data.time)
        self.data.mocap_pos[self.car_mocap] = pos
        yaw = self.trajectory.heading(self.data.time)
        self.data.mocap_quat[self.car_mocap] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]

    def _publish_imu(self, t_ns):
        accel = self.data.sensor('accel').data.copy()
        accel[2] += self.rng.normal(0.0, ACCEL_Z_EXTRA_NOISE)
        gyro = self.data.sensor('gyro').data

        if self.use_target_accel:
            a = target_accel(self.trajectory, self.data.time)
            msg = Vector3Stamped()
            set_stamp(msg.header, t_ns)
            msg.header.frame_id = 'target'
            msg.vector.x, msg.vector.y, msg.vector.z = map(float, a)
            self.accel_pub.publish(msg)

        q = self.data.xquat[self.drone_body]
        att = QuaternionStamped()
        set_stamp(att.header, t_ns)
        att.header.frame_id = 'target'
        att.quaternion.w, att.quaternion.x = float(q[0]), float(q[1])
        att.quaternion.y, att.quaternion.z = float(q[2]), float(q[3])
        self.att_pub.publish(att)

        imu = Imu()
        set_stamp(imu.header, t_ns)
        imu.header.frame_id = 'base_link'
        imu.orientation_covariance[0] = -1.0
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = map(float, accel)
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = map(float, gyro)
        self.imu_pub.publish(imu)

    def _publish_frame(self, t_ns):
        self.renderer.disable_depth_rendering()
        self.renderer.update_scene(self.data, camera=CAM_NAME)
        rgb = self.renderer.render().copy()
        self.renderer.enable_depth_rendering()
        self.renderer.update_scene(self.data, camera=CAM_NAME)
        depth = self.renderer.render().copy()

        info = CameraInfo(height=IMG_H, width=IMG_W)
        info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        color = Image(height=IMG_H, width=IMG_W, encoding='rgb8', step=IMG_W * 3, data=rgb.tobytes())
        dep = Image(height=IMG_H, width=IMG_W, encoding='32FC1', step=IMG_W * 4,
                    data=depth.astype(np.float32).tobytes())
        for m in (info, color, dep):
            set_stamp(m.header, t_ns)
            m.header.frame_id = CAM_NAME
        self.info_pub.publish(info)
        self.color_pub.publish(color)
        self.depth_pub.publish(dep)

    def _publish_true_state(self, t_ns):
        """Stand in for ukf_node: same layouts as /ukf/odom and /ukf/state."""
        qpos, qvel = self.joint.qpos, self.joint.qvel
        car_pos, car_vel = self.trajectory(self.data.time)
        p_T = qpos[:3] - car_pos
        v_T = qvel[:3] - car_vel
        odom = Odometry()
        set_stamp(odom.header, t_ns)
        odom.header.frame_id, odom.child_frame_id = 'target', 'base_link'
        pose, twist = odom.pose.pose, odom.twist.twist
        pose.position.x, pose.position.y, pose.position.z = map(float, p_T)
        pose.orientation.w, pose.orientation.x = float(qpos[3]), float(qpos[4])
        pose.orientation.y, pose.orientation.z = float(qpos[5]), float(qpos[6])
        twist.linear.x, twist.linear.y, twist.linear.z = map(float, v_T)
        twist.angular.x, twist.angular.y, twist.angular.z = map(float, qvel[3:6])
        self.true_odom_pub.publish(odom)
        state = Float64MultiArray()
        state.data = [t_ns * 1e-9, *map(float, p_T), *map(float, v_T), *map(float, qpos[3:7]),
                      *map(float, qvel[3:6]), *[0.0] * 6]
        self.true_state_pub.publish(state)

    def _publish_truth(self, t_ns):
        clock = Clock()
        clock.clock.sec, clock.clock.nanosec = divmod(t_ns, 1_000_000_000)
        self.clock_pub.publish(clock)

        qpos, qvel = self.joint.qpos, self.joint.qvel
        drone = Odometry()
        set_stamp(drone.header, t_ns)
        drone.header.frame_id, drone.child_frame_id = 'world', 'base_link'
        p, t = drone.pose.pose, drone.twist.twist
        p.position.x, p.position.y, p.position.z = map(float, qpos[:3])
        p.orientation.w, p.orientation.x = float(qpos[3]), float(qpos[4])
        p.orientation.y, p.orientation.z = float(qpos[5]), float(qpos[6])
        t.linear.x, t.linear.y, t.linear.z = map(float, qvel[:3])
        t.angular.x, t.angular.y, t.angular.z = map(float, qvel[3:6])
        self.truth_pub.publish(drone)

        pos, vel = self.trajectory(self.data.time)
        car = Odometry()
        set_stamp(car.header, t_ns)
        car.header.frame_id, car.child_frame_id = 'world', 'target'
        c = self.data.xpos[self.car_body]
        car.pose.pose.position.x, car.pose.pose.position.y, car.pose.pose.position.z = map(float, c)
        car.twist.twist.linear.x, car.twist.twist.linear.y, car.twist.twist.linear.z = map(float, vel)
        self.car_pub.publish(car)


def main():
    rclpy.init(args=sys.argv)
    node = NovaSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, rclpy.executors.ExternalShutdownException):
        pass
    except RuntimeError:
        # Ctrl+C can shut ROS down while a message is being taken. Anything else is a real error.
        if rclpy.ok():
            raise
    finally:
        if node.viewer is not None:
            # close() returns while the viewer thread is still freeing its data, and normal
            # teardown then crashes about 1 time in 10 (double free). The flight is over,
            # so leave without tearing down.
            node.viewer.close()
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
