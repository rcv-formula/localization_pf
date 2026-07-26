# localization_pf

알고 있는 2D 점유 격자 지도 위에서 동작하는 파티클 필터 기반 LiDAR–IMU–휠
로컬라이제이션 (ROS 2). 연속적인 휠–IMU EKF dead-reckoning 위에 `map → odom`
보정을 얹어 발행하므로, 스캔 보정과 전역 relocalization이 비동기로 도는 동안에도
pose 출력은 매끄럽고 높은 주기를 유지합니다.

## 개요

노드는 공유 궤적 버퍼와 `map → odom` 변환만으로 통신하는 3개의 느슨하게 결합된
비동기 컴포넌트로 구성됩니다:

1. **파티클 필터** (스캔 주기, ~40 Hz) — 휠+IMU EKF odometry, 모션 왜곡 보정
   (deskew), sharp/wide 혼합 커널 likelihood-field 스캔 채점, AMCL 방식의 동적
   beam skipping, 관측성 기반(이방성) 스캔–휠 융합.
2. **Relocalization** (~1 Hz, 위치 상실 감지 시) — **상대주축 인덱스(relative
   principal-axis index)** 기반 전역 위치추정: 셀별로 미리 계산한 축 맵으로
   heading 후보를 해석적으로 pruning한 뒤 range matching, non-maximum
   suppression, 다중 가설 출력, 그리고 검증 게이트를 통과한 가설만 필터에
   시드합니다.
3. **출력 odom estimator** (100 Hz, read-only) — EKF 궤적 버퍼를 보간하고 최신
   `map → odom`을 합성. 스캔/relocalization 지연과 독립적으로 동작합니다.

## 의존성

- **ROS 2** (rclcpp, geometry_msgs, nav_msgs, sensor_msgs, std_msgs, tf2,
  tf2_geometry_msgs, tf2_ros, visualization_msgs)
- **Eigen3**
- [`range_libc`](https://github.com/kctess5/range_libc) — `third_party/range_libc`
  에 git submodule로 포함 (헤더 + lodepng만 사용, CUDA/Vulkan 경로는 비활성).
- **`vesc_msgs`** — 플랫폼에서 쓰는 VESC 메시지 패키지로,
  [`rcv-formula/f1_stack_for_damvi`](https://github.com/rcv-formula/f1_stack_for_damvi)
  의 `vesc/vesc_msgs`에서 가져옵니다. 노드는 `vesc_msgs/VescStateStamped`를 구독해
  `state.displacement`(휠 타코미터)를 읽습니다.

둘 다 `scripts/setup.sh`가 자동으로 받아옵니다 — 아래 참고.

## 빌드

ROS 2 워크스페이스의 `src/`에 클론한 뒤 setup 스크립트를 실행하면 됩니다 —
`range_libc` submodule을 초기화하고, `f1_stack_for_damvi`에서 `vesc_msgs`를
가져온 뒤 빌드합니다:

```bash
cd <ros2_ws>/src
git clone https://github.com/rcv-formula/localization_pf.git
cd localization_pf
./scripts/setup.sh                 # 의존성 + 빌드   (--no-build 시 의존성만 받음)
```

수동으로 하려면:

```bash
cd <ros2_ws>/src
git clone --recurse-submodules https://github.com/rcv-formula/localization_pf.git
# vesc_msgs(메시지 패키지만)를 src/에 함께 배치
git clone --depth 1 --filter=blob:none --sparse \
    https://github.com/rcv-formula/f1_stack_for_damvi.git /tmp/f1stack
git -C /tmp/f1stack sparse-checkout set vesc/vesc_msgs
cp -r /tmp/f1stack/vesc/vesc_msgs ./vesc_msgs
cd <ros2_ws>
colcon build --packages-select vesc_msgs localization_pf \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> 워크스페이스에 이미 `vesc_msgs`가 있으면 `setup.sh`는 기존 것을 그대로 둡니다.
> 노드는 정수형 `state.displacement` 타코미터 필드를 가진 `VescStateStamped`만
> 있으면 되며, 입력 토픽은 config의 `vesc_state_topic`으로 지정합니다.

## 실행

```bash
source install/setup.bash
ros2 launch localization_pf localization_pf.launch.py
# 파라미터 파일 지정:
ros2 launch localization_pf localization_pf.launch.py params_file:=/path/to/config.yaml
```

모든 튜닝 값은 [`config/config.yaml`](config/config.yaml)에 있고, 컴포넌트별
(IO / 파티클 필터 / relocalization)로 정리되어 있습니다. 처리 파이프라인 분기는
코드에 고정되어 있어 config는 값만 담습니다.

## 인터페이스

**구독 (Subscriptions)**

| 토픽 | 타입 | 비고 |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | transient_local (latched) |
| `/scan` | `sensor_msgs/LaserScan` | best-effort |
| `/imu` | `sensor_msgs/Imu` | best-effort |
| `/sensors/core` | `vesc_msgs/VescStateStamped` | 휠 odometry |
| `/tf`, `/tf_static` | — | `base_link → laser` extrinsic 조회 |

(토픽 이름은 `config.yaml`에서 변경 가능)

**발행 (Publications)**

| 토픽 | 타입 | 비고 |
|---|---|---|
| `localization_pf/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 융합 pose, 100 Hz |
| `localization_pf/state` | `std_msgs/UInt8` | 0=Lost, 1=Converging, 2=Tracking |
| TF `map → odom → base_link` | — | `base_link → laser` static은 선택 (`output.publish_laser_tf`) |
| `localization_pf/{particles, scan_particles, slip, skipped_scan, latency, ...}` | — | 진단 / 시각화 |

## 라이선스

Apache-2.0. `third_party/range_libc`는 자체 라이선스를 따릅니다.
