# localization_pf

Particle-filter LiDAR–IMU–wheel localization on a known 2D occupancy grid (ROS 2),
with global relocalization.

## Build

```bash
cd <ros2_ws>/src
git clone https://github.com/rcv-formula/localization_pf.git
cd localization_pf
./scripts/setup.sh        # fetches range_libc + vesc_msgs, then builds
```

## Run

```bash
source <ros2_ws>/install/setup.bash
ros2 launch localization_pf localization_pf.launch.py
```

## Configure

All tuning lives in [`config/config.yaml`](config/config.yaml). The launch file
reads it from the source tree, so **edit and relaunch — no rebuild needed**.

To use a different map, put `<name>.yaml` + image in [`map/`](map) and set:

```yaml
map_loader:
  map_name: track2
```

Set `map_loader.enabled: false` to subscribe to an external `map_server` instead.

## Topics

| Direction | Topic | Type |
|---|---|---|
| in | `/scan` | `sensor_msgs/LaserScan` |
| in | `/imu` | `sensor_msgs/Imu` |
| in | `/sensors/core` | `vesc_msgs/VescStateStamped` |
| in | `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` |
| out | `localization_pf/pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| out | `/odom` | `nav_msgs/Odometry` (same result, `map` → `base_link`) |
| out | `localization_pf/state` | `std_msgs/UInt8` — 0 Lost, 1 Converging, 2 Tracking |
| out | `/localization_confidence` | `std_msgs/Float64MultiArray` — `[stamp_sec, stamp_nanosec, confidence]`, confidence in [0,1] |
| out | TF `map → odom → base_link` | — |

Topic names are configurable. `localization_pf/{particles, slip, skipped_scan,
latency, ...}` are published for diagnostics.

## Manual pose

RViz's **2D Pose Estimate** tool seeds the filter directly. Set RViz's Fixed
Frame to `map` — a pose in any other frame is rejected. Automatic
relocalization stays active; it is only held for `manual_seed_grace_s` (5 s)
so the filter can settle where you pointed.

## License

Apache-2.0. `third_party/range_libc` keeps its own license.
