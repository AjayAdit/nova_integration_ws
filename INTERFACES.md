# NOVA ROS 2 interface contract

Status: v1.1, 2026-11-01. v1 was approved; the startup rules below were added in step 5 and need review. Change this file first, then the code.

Scope: the topics between the test sim, `target_perception_ros`,
`target_ukf_ros` and the C++ NMPC in `quad_nmpc_ros`.
Reference behaviour: `NOVA_PAMPC` branch `NOVA-OG-PAMPC`, commit `6a51381`.

## Global rules

| Rule | Value |
|---|---|
| Time | Every `header.stamp` is sim time (`data.time`). Never the wall clock. |
| Clock | The sim publishes `/clock`. All nodes run with `use_sim_time:=true`. |
| Target frame `target` | Origin at the car, axes parallel to `world`, Z up. |
| Body frame `base_link` | The drone body. |
| Quaternion order | ROS `(x,y,z,w)` in messages. Convert to `(w,x,y,z)` only at the message boundary. |
| Ground truth | `/ground_truth/*` is for scoring only. No node in the control loop may subscribe to it. |
| Packages | The UKF and perception packages must not import MuJoCo, and must not depend on each other. Topic names are remappable. |
| Distro | ROS 2 Humble first. |

## Rates

| Item | Rate | Source |
|---|---|---|
| Physics | 500 Hz | `timestep=0.002` |
| IMU and attitude | 250 Hz | `UKF_PREDICT_EVERY=2` |
| Images | 10 Hz | `PERC_EVERY_STEPS=50` |
| `/ukf/odom`, NMPC solve | 100 Hz | every 5 physics steps |
| `/nmpc/reference` | 100 Hz | one per `/ukf/odom` |

## System graph

```
nova_sim_node
  |- images + camera_info ----------> perception_node
  |- /drone/attitude ---------------> perception_node, ukf_node
  |- /imu, /target/accel -----------> ukf_node
  |- /target/accel -----------------> nmpc_node
  |- /nmpc/reference ---------------> nmpc_node
  `- /ground_truth/* ---------------> test scorer only

perception_node
  |- /perception/position ----------> ukf_node
  `- /perception/target_bbox -------> nova_sim_node (eq. 22)

ukf_node
  |- /ukf/odom ---------------------> nmpc_node, nova_sim_node (eq. 19-20)
  `- /ukf/state --------------------> test scorer

nmpc_node --> trpy_converter_node --> /quadrotor/trpy_cmd --> AcroMode (in nova_sim_node)
```

## Startup (added in step 5)

- `ukf_node` publishes nothing until its first position update (`wait_for_first_update`, default true). Before it, the attitude is only the default identity, and flying on that made the flight diverge.
- `nova_sim_node` publishes the t=0 sensors, then holds physics until the first `/motor_cmd`. This matches the baseline, where the first update and first command both happen before any physics.
- The sim steps 1 physics step per timer tick, so commands are read between steps.

## Debug windows (added in step 5, off by default)

- `viewer:=true`: MuJoCo window with the car's path drawn in the viewer only. Camera images never contain it.
- `detection_view:=true`: `sim/detection_view_node.py` subscribes best effort to the colour image and `/perception/target_bbox`, and publishes nothing. The sim then waits for 2 colour image subscribers instead of 1.

## True-state test (test only, off by default)

`true_state:=true` is the one exception to the ground truth rule, used to measure the controller's limit. `ukf_node` is not started, and the sim publishes the true state on `/ukf/odom` and `/ukf/state` in the same layout, at 100 Hz, with zero biases. Everything else is unchanged. It can never apply to hardware.

## Sim outputs

Published by `nova_sim_node`, from Python, so every stamp is exactly `data.time`.

| Topic | Type | QoS depth | Contents |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | 10 | Sim time |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 1 | `rgb8`, 640x480, frame `rgbd_cam` |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 1 | `32FC1`, metres, same stamp as the colour image |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 1 | `k` holds fx, fy, cx, cy; same stamp |
| `/imu` | `sensor_msgs/Imu` | 100 | Specific force and gyro, body frame, with baseline noise. Orientation covariance `[0] = -1` (not provided). |
| `/drone/attitude` | `geometry_msgs/QuaternionStamped` | 100 | q_TB, same stamp as `/imu` |
| `/target/accel` | `geometry_msgs/Vector3Stamped` | 10 | Oracle target acceleration, `target` axes. Sent only when oracle mode is on. |
| `/nmpc/reference` | see "Reference" | 10 | Output of `guidance.py` |
| `/ground_truth/odom` | `nav_msgs/Odometry` | 10 | True drone state, world frame |
| `/ground_truth/target_odom` | `nav_msgs/Odometry` | 10 | True car state, world frame |

The ImuPublisher and ImagePublisher plugins are NOT used. See "Why not the plugins".

## Perception node

| Direction | Topic | Type |
|---|---|---|
| in | `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info` | as above |
| in | `/drone/attitude` | as above |
| out | `/perception/target_bbox` | `vision_msgs/Detection2DArray` |
| out | `/perception/position` | `geometry_msgs/PointStamped` |

- Pairs colour and depth on an exact stamp.
- Takes the attitude with the same stamp. If none lies within 5 ms, drops the frame and counts it.
- `target_bbox` is published for EVERY frame. Empty `detections` means no detection, so the scorer can count detection loss.
- `center.position` is the box centre in pixels, and `size_x`, `size_y` are its size. Same layout as `Drone_Detector_YOLO_ROS2`.
- `position` is p_T: the drone's position in `target`. It is published only on success, and carries the image stamp.
- Parameters: `q_bc` (w,x,y,z), `t_bc`, `bin_width` (0.15 m), `depth_min` (0.1 m), `depth_max` (50 m).
- Intrinsics come from `camera_info`, not constants.
- The detector stays behind `detect(rgb) -> (x,y,w,h) | None`.

## UKF node

| Direction | Topic | Type |
|---|---|---|
| in | `/imu` | predict step |
| in | `/perception/position` | update step |
| in | `/drone/attitude` | paired with position for the 7-dim update |
| in (optional) | `/target/accel` | predict step, only if `use_target_accel:=true` |
| out | `/ukf/odom` | `nav_msgs/Odometry` |
| out | `/ukf/state` | `std_msgs/Float64MultiArray`, the 19 states, for scoring |

- Predict `dt` is the difference between IMU stamps. A non-positive `dt` is dropped and counted.
- An update is applied when it arrives, with no rollback. Each measurement is used once.
- `/ukf/state` has no header, so it holds 20 elements: element 0 is the sim time, followed by the 19 states.
- `/ukf/odom` layout (same convention as today's `sim_node`):
  - `header.frame_id = target`, `child_frame_id = base_link`
  - `pose.position` = p_T, `pose.orientation` = q_TB
  - `twist.linear` = v_T in **target axes** (ROS convention would be child frame)
  - `twist.angular` = bias-corrected omega in **body axes**
- Parameters: `init_position`, `pos_noise_std` (0.30 for all axes, from `sim_world.py:210`), `use_target_accel`, `publish_rate` (100 Hz).

## Reference

`/nmpc/reference` is `trajectory_msgs/MultiDOFJointTrajectory`.

| Field | Value |
|---|---|
| `header.frame_id` | `target` |
| `header.stamp` | Stamp of the `/ukf/odom` that produced it |
| `joint_names` | `["base_link"]` |
| `points` | Exactly 21 (N+1), spaced 0.05 s through `time_from_start` |
| `transforms[0].translation` | Goal position |
| `transforms[0].rotation` | Yaw-only quaternion, sign preserved |
| `velocities[0].linear` | Goal velocity (all zero for NOVA, eq. 21) |

A solve waits for `/ukf/odom` and the reference with the same stamp, so a lost
reference skips that solve. `nmpc_node` rejects any message without exactly 21 points. It keeps the quaternion
sign: yaw is recovered as `2*atan2(qz, qw)`, which restores the same `qw`, `qz`.

## Guidance (temporary, inside the test sim)

`sim/guidance.py` has no MuJoCo or ROS imports. It contains eq. 19, 20, 21 and 22.

| Input | Source |
|---|---|
| p_T | `/ukf/odom`, never ground truth |
| Box centre u | Latest `/perception/target_bbox` |
| Drone yaw psi | True attitude (same as `sim_world.py:311`) |
| t | Sim time |

Constants: `D_SAFE = 1.0`, `Z_OFFSET = 1.0`, `T_RAMP = 1.0`.
On an empty bbox, yaw freezes at its last value, as the baseline does.
This must move to its own node before any hardware flight.

## Changes needed in quad_nmpc_ros

Each change must leave the default circle demo unchanged.

| # | Change | Default |
|---|---|---|
| N1 | Parameter `reference_topic`; when set, use it instead of the internal circle | Empty: circle |
| N2 | Parameter `target_accel_topic`; feeds the solver parameter `p` on every node | Empty: zeros |
| N3 | CMake option selecting the generated solver directory | `acados_generated/` |
| N4 | Read `kNy`, `kNyN` from the generated header (NOVA: 22 and 17; today: 18 and 13) | Same values as now |
| N5 | Publish no command until the first odometry AND first reference have arrived | Circle: unaffected |

`/odom` is remapped to `/ukf/odom` at launch; no code change is needed.

NOVA settings: mass 1.25 kg, `k_tau` 20, `l` 0.125, `q_pixel` 250, `q_pixel_vel` 0.

## Actuation

`trpy_converter_node` and `TRPYCommand` are used unchanged, with `k_tau:=20.0`.
The NOVA AcroMode world needs these per-axis limits (motors 0 to 20 N, l = 0.125,
yaw gear 0.022): fz 0 to 80 N, tx and ty +/-5 N m, tz +/-0.88 N m.

## Why not the plugins

- `ImuPublisher` stamps with `get_clock()->now()`, not `data.time`
  (`ImuPublisher.cpp:120`), and its orientation is always identity.
- `sim_world.py` adds extra accelerometer noise in Python (`sim_world.py:442`),
  which the plugin does not.
- The baseline renders through Python `mujoco.Renderer`. Keeping that makes the
  images comparable.

Only AcroMode stays a plugin. Real hardware drivers would replace these publishers later.

## Known differences from the baseline

| Difference | Effect | How it is handled |
|---|---|---|
| Measurements arrive late over ROS | May raise tracking error | Scorer reports image-to-update latency |
| AcroMode instead of direct motors | May change tracking error | Run motors-direct too if the result differs |
| Solves run asynchronously | No byte-exact trace | Compare `trace_metrics.py` metrics with tolerances |
| Python UKF at 250 Hz could lag | IMU queue grows or drops | Step 2: vectorised predict is 0.26 ms mean, 5 ms p99 odometry lag when run alone. Recheck under full load in step 5. |

## The solver amplifies rounding (measured in step 4)

Replaying the golden run through the original Python NMPC with 1e-15 rad added to
yaw changes the commands from about step 330 (t = 3.3 s), by up to 1.0 and with an
RMS of 0.15. Unchanged, the replay matches exactly. The C++ node behaves the same
way: first 300 steps match within 1e-6, then RMS 0.156.

So step-by-step command comparison cannot pass after that point, for any controller.
Judge the integration on closed-loop tracking metrics. The size of these command
swings may be worth its own look, separately from this port.

## Open point to check in step 5

Eq. 22 in the baseline uses a wrapped `atan2` yaw (`sim_world.py:311-312`), while
`nmpc_node.cpp` warns that a wrap flips the quaternion and reads as a 180 degree
error. `guidance.py` keeps the baseline behaviour so the comparison is fair.
The scorer flags reference quaternion sign flips: 0 in all 11 flights after the startup fix, so the wrap was not triggered on this path. It is not shown safe in general.
