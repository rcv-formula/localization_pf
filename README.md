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
2. **Relocalization** (위치 상실 감지 시, 워커 스레드) — **상대주축 인덱스
   (relative principal-axis index)** 기반 전역 위치추정: 셀별로 미리 계산한 축
   맵으로 heading 후보를 해석적으로 pruning한 뒤 range matching, non-maximum
   suppression, 다중 가설 출력, 그리고 검증 게이트를 통과한 가설만 필터에
   시드합니다. 탐색 자체는 별도 스레드에서 돌아 스캔 주기를 막지 않습니다.
   잘못된 위치에 시드하는 것이 시드하지 않는 것보다 나쁘므로, 채택 경로에는
   네 겹의 방어가 있습니다:
   - **탐색 전제조건** — 유효 빔 비율·차체 기울기·직전 스캔과의 일관성이
     모두 만족될 때만 전역 탐색을 시작합니다(넘어지거나 가려진 순간의 스캔으로
     탐색하지 않음).
   - **부호 인식 검증** — 관측 거리와 예측 거리의 차를 부호로 나눠 봅니다.
     짧게 나오는 것은 가림(occlusion)이라 벌하지 않고, **길게 나오는 것
     (see-through)** 만 오정합의 증거로 셉니다. 각도 양자화를 흡수하도록
     인접 빈의 min/max 대역을 씁니다.
   - **절대 RMS 게이트** — 후보들 사이의 상대 점수뿐 아니라 절대 잔차 상한을
     둬서, "다 같이 나쁜" 후보군에서 1등이 뽑히는 것을 막습니다.
   - **이력 가중 다수결** — 최신 스캔 하나가 아니라 이동 이력(위치/각도 간격으로
     솎은 과거 스캔들)에서 각 가설을 재채점하고, 지지 비율이 낮으면 기각하거나
     적은 파티클로 보호관찰(probation) 시드합니다.
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
  기본적으로 `state.speed`(ERPM)를 읽습니다(`ekf.wheel_use_displacement: true`로
  두면 `state.displacement` 타코미터 경로).

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

> **config 라이브 반영** — launch는 워크스페이스의 소스 `config/config.yaml`을
> 직접 읽습니다. 따라서 값을 수정한 뒤 **`colcon build` 없이 재런치만** 하면
> 바로 반영됩니다(설치본이 아니라 소스를 읽음; 소스 트리가 없는 배포 설치에서는
> 설치된 config로 폴백).

## 지도(map)

노드는 자체 맵 로더를 내장하고 있어 별도 `map_server` 없이 동작합니다. 켜져 있으면
(`map_loader.enabled: true`, 기본값):

1. 패키지의 [`map/`](map) 폴더에서 `map_loader.map_name`(기본 `map`)에 해당하는
   `<map_name>.yaml` + 이미지(PGM)를 읽어
2. 로컬라이제이션에 직접 사용하고(내부 보유),
3. `map_topic`(기본 `/map`)에 **transient_local**로 latch 발행합니다 — RViz나
   상위 노드가 늦게 붙어도 지도를 받습니다.

`map/` 폴더에는 표준 `map_server` 형식의 샘플 지도가 두 개 들어 있습니다
(`map.yaml`/`map.pgm`, `track2.yaml`/`track2.pgm`). 여러 지도를 이 폴더에 두고
**config의 `map_loader.map_name` 값만 바꿔 재런치**하면 지도가 전환됩니다
(`colcon build` 불필요):

```yaml
map_loader:
  map_name: track2      # map/track2.yaml + track2.pgm 을 로드
```

실제 운용 시에는 자신의 지도 파일을 `map/`에 넣고 `map_name`을 맞추면 됩니다.

지도 이미지는 **P5(binary)와 P2(ASCII) PGM을 모두** 읽으며, 16-bit(maxval>255)
이미지는 8-bit로 자동 스케일합니다. 그림판·GIMP 등으로 편집해 어떤 방식으로
저장하든 그대로 쓸 수 있습니다. 다만 PNG/BMP를 확장자만 `.pgm`으로 바꾼 파일은
읽을 수 없으며, 이 경우 로그에 실제 감지된 형식과 해결 방법이 함께 출력됩니다.

외부 `map_server`를 쓰려면 `map_loader.enabled: false`로 두면 `map_topic`을
구독합니다.

## 인터페이스

**구독 (Subscriptions)**

| 토픽 | 타입 | 비고 |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | transient_local (latched) |
| `/scan` | `sensor_msgs/LaserScan` | best-effort |
| `/imu` | `sensor_msgs/Imu` | best-effort |
| `/sensors/core` | `vesc_msgs/VescStateStamped` | 휠 odometry. 기본은 `state.speed`(ERPM)를 `vesc_to_odom`과 동일한 식으로 속도 변환 후 적분 (`ekf.speed_to_erpm_gain`). `ekf.wheel_use_displacement: true`로 두면 `state.displacement`(타코미터) 경로 |
| `/tf`, `/tf_static` | — | `base_link → laser` extrinsic 조회 |

(토픽 이름은 `config.yaml`에서 변경 가능)

**발행 (Publications)**

| 토픽 | 타입 | 비고 |
|---|---|---|
| `localization_pf/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 최종 융합 pose (frame: `map`), 100 Hz |
| `/odom` (`odom_topic`) | `nav_msgs/Odometry` | **같은 최종 결과**, 100 Hz. `frame_id: map`, `child_frame_id: base_link` — 즉 map 기준 base_link 절대 pose. twist는 base_link 기준. 관례상 `/odom`은 휠 오돔용이므로 충돌 시 `odom_topic`으로 변경 |
| `localization_pf/state` | `std_msgs/UInt8` | 0=Lost, 1=Converging, 2=Tracking |
| `/localization_confidence` | `std_msgs/Float64MultiArray` | 신뢰도 `[stamp_sec, stamp_nanosec, confidence]`. pose/odom과 **동일한 stamp** 라 시각으로 짝지을 수 있습니다. `confidence` 는 [0,1] — 스캔 정합도·불일치 빔 비율·beam skip·파티클 구름 퍼짐·가설 확정도 중 **가장 나쁜 값**에 상태 배율(Lost 0, Converging 0.5)을 곱한 값 |
| TF `map → odom → base_link` | — | `base_link → laser` static은 선택 (`output.publish_laser_tf`) |
| `localization_pf/{particles, scan_particles, slip, skipped_scan, latency, ...}` | — | 진단 / 시각화 |

## 라이선스

Apache-2.0. `third_party/range_libc`는 자체 라이선스를 따릅니다.
