# localization_pf

Particle-filter based LiDAR–IMU–wheel localization for a known 2D occupancy map
(ROS 2). Publishes a `map → odom` correction on top of a continuous wheel–IMU EKF
dead-reckoning, so pose output stays smooth and high-rate while scan correction
and global relocalization run asynchronously.

## Overview

The node runs three loosely-coupled, asynchronous components that communicate
only through a shared trajectory buffer and the `map → odom` transform:

1. **Particle filter** (scan-rate, ~40 Hz) — wheel+IMU EKF odometry, motion
   deskew, likelihood-field scan scoring with a sharp/wide mixture kernel,
   AMCL-style dynamic beam skipping, and observability-weighted (anisotropic)
   fusion between scan and wheel odometry.
2. **Relocalization** (~1 Hz, when localization failure is detected) — global
   localization via a **relative principal-axis index**: per-cell precomputed
   axis maps prune heading candidates analytically, followed by range matching,
   non-maximum suppression, multi-hypothesis output, and a verification gate so
   only verified hypotheses seed the filter.
3. **Output odom estimator** (100 Hz, read-only) — interpolates the EKF
   trajectory buffer and composes the latest `map → odom`, independent of the
   scan / relocalization latency.

## Dependencies

- **ROS 2** (rclcpp, geometry_msgs, nav_msgs, sensor_msgs, std_msgs, tf2,
  tf2_geometry_msgs, tf2_ros, visualization_msgs)
- [`vesc_msgs`](https://github.com/f1tenth/vesc) — VESC state message package,
  place in your ROS 2 workspace `src/`.
- **Eigen3**
- [`range_libc`](https://github.com/kctess5/range_libc) — bundled as a git
  submodule under `third_party/range_libc` (header + lodepng only; CUDA/Vulkan
  paths are disabled).

## Build

```bash
# clone into a ROS 2 workspace, with submodules
cd <ros2_ws>/src
git clone --recurse-submodules https://github.com/rcv-formula/localization_pf.git
# (if you cloned without --recurse-submodules)
cd localization_pf && git submodule update --init --recursive && cd ..

# make sure vesc_msgs is also present in src/, then build from the ws root
cd <ros2_ws>
colcon build --packages-select localization_pf --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Run

```bash
source install/setup.bash
ros2 launch localization_pf localization_pf.launch.py
# override parameters:
ros2 launch localization_pf localization_pf.launch.py params_file:=/path/to/config.yaml
```

All tuning lives in [`config/config.yaml`](config/config.yaml), organized by
component (IO / particle filter / relocalization). Pipeline branch selection is
fixed in code — the config carries values only.

## Interface

**Subscriptions**

| topic | type | note |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | transient_local (latched) |
| `/scan` | `sensor_msgs/LaserScan` | best-effort |
| `/imu` | `sensor_msgs/Imu` | best-effort |
| `/sensors/core` | `vesc_msgs/VescStateStamped` | wheel odometry |
| `/tf`, `/tf_static` | — | `base_link → laser` extrinsic lookup |

(topic names are configurable in `config.yaml`.)

**Publications**

| topic | type | note |
|---|---|---|
| `localization_pf/pose` | `geometry_msgs/PoseWithCovarianceStamped` | fused pose, 100 Hz |
| `localization_pf/state` | `std_msgs/UInt8` | 0=Lost, 1=Converging, 2=Tracking |
| TF `map → odom → base_link` | — | `base_link → laser` static optional (`output.publish_laser_tf`) |
| `localization_pf/{particles, scan_particles, slip, skipped_scan, latency, ...}` | — | diagnostics / visualization |

## License

Apache-2.0. `third_party/range_libc` retains its own license.
