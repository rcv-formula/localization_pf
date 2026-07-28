#pragma once

#include <array>
#include <deque>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "structs.h"
#include "observability_map.h"
#include "particle_filter.h"
#include "particle_propagation.h"
#include "pose_estimation.h"
#include "relocalization.h"
#include "scan_deskew.h"
#include "scan_scoring.h"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"


class mainNode : public rclcpp::Node
{
public:
  mainNode();

private:
  // 전역 초기화가 끝나기 전에는 추적을 시작하지 않습니다.
  // Converging은 다중 가설이 아직 여럿 살아있는 모호 단계로, 지배 모드
  // pose를 발행하되 주행으로 판별이 끝나야 Tracking으로 확정됩니다.
  enum class LocalizationState {
    WaitingForMap,
    Lost,
    Converging,
    Tracking
  };

  // 평면 pose 하나를 다루기 위한 최소 표현입니다.
  struct Pose2D {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  void declareParameters();

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  // 맵으로 relocalization/scoring/observability를 (재)구성하고 추적 상태를
  // 리셋합니다. 외부 /map 구독과 자체 파일 로더가 공유하는 경로입니다.
  bool setupMap(const nav_msgs::msg::OccupancyGrid &grid);
  // 패키지 map 폴더의 <map_name>.yaml + 이미지를 OccupancyGrid로 읽어
  // setupMap을 호출하고, transient_local로 map_topic에 latch 발행합니다.
  bool loadMapFromFile(const std::string &map_dir, const std::string &map_name);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void vesc_state_callback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg);
  // 100Hz 출력 경로입니다. 궤적 버퍼를 읽기만 하므로 PF와 간섭하지 않습니다.
  void output_timer_callback();

  // scan 트리거 한 사이클: advance -> deskew -> propagate -> score -> resample
  //                        -> estimate -> 이방성 융합 -> map->odom 갱신
  void runFilterCycle(const sensor_msgs::msg::LaserScan &scan);
  // Lost 상태에서 전역 위치를 찾습니다. 성공하면 파티클을 다시 뿌립니다.
  bool tryRelocalize(const sensor_msgs::msg::LaserScan &scan);
  // 상태 전이는 반드시 이 함수로 해서 상태 토픽과 어긋나지 않게 합니다.
  void setState(LocalizationState next);
  void publishState();
  // reloc 트리거 사유를 맵 중앙 텍스트 마커로 알립니다(사유별 색상).
  void announceReloc(const std::string &reason, float r, float g, float b);
  // 가설 목록으로 필터를 재초기화하는 공통 경로입니다(초기화/재수렴 공용).
  void seedFilter(const std::vector<ParticleFilter::ModeSeed> &seeds);
  // 이동/회전 간격을 만족할 때 스캔을 히스토리에 남깁니다(모든 상태 공용).
  void updateScanHistory(const sensor_msgs::msg::LaserScan &scan);
  // 히스토리가 있으면 multi-scan, 없으면 단일 스캔 전역 탐색을 돌립니다.
  std::vector<Relocalization::Hypothesis> runGlobalSearch(
    const sensor_msgs::msg::LaserScan &scan);
  // base 위치가 주행 가능(free) 셀인지 봅니다. 맵 밖/unknown/occupied는 false.
  bool isFreeCell(double x, double y) const;
  // 스캔 탐색 풀 한 사이클: 이동 적용 -> 채점 -> 컬링/복제 -> 우위 시 주입.
  void updateScanParticles(double best_main_score, particle *particles,
                           int32_t particle_count);
  // base_link -> laser 정적 변환을 최초 1회만 조회해 반영합니다.
  void ensureLaserExtrinsic();

  // 라이다 pose <-> 뒤차축(base_link) pose 변환입니다.
  Pose2D laserToBase(const Pose2D &laser) const;
  // map->base 와 odom->base 로부터 map->odom 을 만듭니다.
  Pose2D computeMapToOdom(const Pose2D &map_to_base, const Pose2D &odom_to_base) const;

  void publishTransforms(const rclcpp::Time &stamp);
  void publishPose(const rclcpp::Time &stamp);
  void publishParticles(const rclcpp::Time &stamp);

  // ---- 파라미터 ----
  std::string vesc_state_topic_{"/sensors/core"};
  std::string scan_topic_{"/scan"};
  std::string imu_topic_{"/imu"};
  std::string map_topic_{"/map"};
  // 자체 맵 로더: 켜면 map_dir/<map_name>.yaml+이미지를 읽어 localization에
  // 쓰고 map_topic에 latch 발행합니다. 끄면 외부 /map을 구독합니다.
  bool map_loader_enabled_{true};
  std::string map_loader_dir_{};
  std::string map_loader_name_{"map"};
  std::string map_frame_{"map"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string laser_frame_{"laser"};
  double output_rate_hz_{100.0};
  // Lost 상태에서 relocalization을 시도하는 최소 간격입니다.
  // 전역 탐색은 수백 ms가 걸리므로 매 scan마다 시도하지 않습니다.
  double relocalize_period_s_{1.0};
  // 리샘플링을 허용할 최소 이동/회전량입니다. 정지 중 다양성 고갈을 막습니다.
  double resample_min_translation_{0.02};
  double resample_min_rotation_{0.01};
  bool publish_particles_{true};
  // base_link->laser 정적 TF를 우리 extrinsic으로 발행할지 여부입니다.
  // tf_static이 비어 있는 bag(icra 등)에서는 켜야 laser 프레임이 트리에
  // 생기고, 상위에서 이미 base_link->laser를 발행하면 꺼서 중복을 막습니다.
  bool publish_laser_tf_{true};

  // ---- 다중 가설 수렴/감시 파라미터 ----
  // 지배 모드 질량이 이 값 이상인 상태로 아래 거리만큼 주행해야 수렴을
  // 선언합니다. 정지 중에는 앨리어스를 구분할 수 없으므로 절대 수렴하지
  // 않습니다(0526-1 교훈).
  double converge_mass_{0.9};
  double converge_distance_m_{1.0};
  // 직선 주행에서는 복도 앨리어스끼리 똑같이 잘 맞아 판별이 안 됩니다
  // (0526-1 스래싱 교훈: 판별은 전부 코너에서 일어남). 수렴에는 거리와
  // 함께 이만큼의 누적 회전(코너 통과)이 필요합니다.
  double converge_rotation_deg_{90.0};
  // 노이즈 회복 건강도 스케일의 하한 score(감시 트리거로는 쓰지 않음 —
  // Lost 트리거는 빔 합의와 off-map 둘뿐입니다).
  double score_fail_threshold_{-9.0};
  // 노이즈 회복 확장의 건강도 상한 기준 score(이 값 이상이면 health=1).
  double score_good_{-1.0};
  // ---- multi-scan 전역 탐색 ----
  // 단일 스캔은 닮은꼴 복도/루프를 구분하지 못하므로(0526-1 전역 앨리어스)
  // 주행 중 모아둔 과거 스캔들을 DR 상대이동으로 묶어 공동 채점합니다.
  int multi_scan_count_{12};
  double multi_scan_spacing_m_{0.5};
  double multi_scan_spacing_deg_{15.0};
  // ---- 맵 정합성 감시 (off-map) ----
  // 로봇은 unknown/벽 위를 달리지 않으므로 fused가 free 아닌 셀에 오래
  // 머무는 것은 score와 무관한 배치 오류 신호입니다(닮은꼴 루프 판별).
  double map_consistency_window_s_{5.0};
  double map_consistency_threshold_{0.3};
  // ---- 스캔 점수 전용 탐색 파티클 (무상태) ----
  // 매 프레임 "이전 최종 pose를 이번 이동량으로 전파한 지점" 주위 고정
  // 범위에 균일하게 새로 뿌리는 추가 파티클. 스캔 score만으로 채점하고,
  // 최고점이 본 필터보다 명확히 좋으면 본 필터 최하위를 교체합니다.
  int scan_particle_count_{100};
  double scan_particle_spread_pos_{0.5};
  double scan_particle_spread_yaw_{0.26};
  double scan_particle_inject_margin_{0.5};
  int scan_particle_inject_count_{20};
  // ---- 빔 아웃라이어 기반 즉각 노이즈 회복 ----
  // best 파티클에서 정합 낮은 빔 비율이 good을 넘으면 즉시 예측 노이즈를
  // 넓힙니다(bad에서 최대). 평균 score보다 민감하고 반응이 빠릅니다.
  double outlier_prob_{0.1};
  double outlier_frac_good_{0.15};
  double outlier_frac_bad_{0.5};
  // ---- 궤적-모양 전역 정합 ----
  // 스캔 채점이 앨리어스에 속는 맵(낡은 벽/자기유사 트랙)에서도 주행로
  // 배치는 유지되므로, 코너를 포함한 충분한 누적 궤적이 모이면 스캔
  // 탐색 대신 궤적 정합으로 가설을 만듭니다.
  double trajectory_fit_min_distance_m_{8.0};
  double trajectory_fit_min_rotation_deg_{180.0};
  double trajectory_fit_history_m_{30.0};

  Relocalization::Parameters relocalization_parameters_;
  ObservabilityMap::Parameters observability_parameters_;
  // sigma[m] -> confidence[0,1] 변환 기준(0.5가 되는 sigma).
  double observability_reference_sigma_{1.0};
  // 관측성 조회 실패 시의 등방 중립 신뢰도입니다.
  double observability_fallback_confidence_{0.5};
  particlePropagation::EkfParameters ekf_parameters_;
  particlePropagation::MotionNoise motion_noise_;
  particlePropagation::ImuExtrinsic imu_extrinsic_;
  particlePropagation::LaserExtrinsic laser_extrinsic_;
  ParticleFilter::Parameters filter_parameters_;
  PoseEstimation::Parameters estimation_parameters_;
  ScanDeskew::Parameters deskew_parameters_;
  float scoring_factor_{0.2f};
  // 채점 빔 솎음(1=전체). AMCL은 60빔만 쓰는 관행이라 2~4도 보수적입니다.
  int scoring_beam_stride_{2};
  double scoring_sharp_sigma_{0.05};
  double scoring_sharp_weight_{0.6};
  // ---- beam skipping (항상 활성) ----
  double beam_skip_prob_{0.05};
  double beam_skip_consensus_{0.3};
  double beam_skip_error_threshold_{0.5};
  int beam_skip_particle_stride_{8};
  // 스킵 제안 비율이 lost_fraction을 넘는 프레임이 창 N 중 M개를 넘으면
  // 위치 상실로 보고 relocalization을 트리거합니다 (score 감시가 못 잡는
  // 미아 상태 대응). lost_fraction은 스킵-포기 밸브(error_threshold)와
  // 별개의, 더 민감한 기준입니다.
  double beamskip_lost_fraction_{0.30};
  int beamskip_lost_window_{10};
  int beamskip_lost_count_{4};
  // 채점 스레드 수(0=auto: min(8, hw)). 파티클 청크로 분할합니다.
  int scoring_threads_{0};

  // ---- 구성 요소 ----
  nav_msgs::msg::OccupancyGrid map_;
  std::unique_ptr<Relocalization> relocalization_;
  // 맵 기반 기하 퇴화 사전계산(미끄러짐 민감도). 맵 콜백에서 build되고
  // runFilterCycle이 조회만 합니다.
  ObservabilityMap observability_;
  std::unique_ptr<scanScoring> scoring_;
  std::unique_ptr<particlePropagation> propagation_;
  ParticleFilter filter_;
  PoseEstimation estimation_;
  ScanDeskew deskew_;

  // ---- 상태 ----
  LocalizationState state_{LocalizationState::WaitingForMap};
  bool laser_extrinsic_ready_{false};
  bool laser_tf_published_{false};
  Pose2D map_to_odom_{};
  bool map_to_odom_valid_{false};
  rclcpp::Time last_relocalize_attempt_{0, 0, RCL_ROS_TIME};
  // 진단용 카운터입니다. raw_gyro_yaw_는 EKF를 거치지 않은 자이로 적분으로,
  // 데이터셋에서 휠오돔 yaw와 일치함을 확인한 사실상의 정답 회전입니다.
  double raw_gyro_yaw_{0.0};
  double raw_gyro_bias_{0.0};
  double raw_gyro_last_time_{0.0};
  int raw_gyro_bias_samples_{0};
  int scan_count_{0};
  int cycle_count_{0};
  double last_cycle_time_{0.0};
  double accumulated_translation_{0.0};
  double accumulated_rotation_{0.0};
  // 수렴 판정용: 지배 질량 조건을 유지한 채 누적한 주행 거리/회전입니다.
  double converge_accumulated_{0.0};
  double converge_rotation_accumulated_{0.0};
  // 직전 사이클의 지배 모드입니다. 바뀌면 융합 기준을 리베이스해 두 모드
  // 사이를 블렌딩한 헛 포즈가 나가지 않게 합니다.
  int last_dominant_mode_{0};
  // 직전 사이클의 생존 모드 수(빔 합의 무장 판정용, 파종 직후는 다수).
  int last_alive_modes_{1};
  // score 감시: 임계 미만이 시작된 시각(<0이면 정상)입니다.
  // 주기 검증: 마지막 수행 시각과 연속 불일치 횟수입니다.
  double node_start_seconds_{0.0};
  // 레이턴시 계측 누적(진단 로그용).
  double latency_arrival_ms_{0.0};
  double latency_processing_ms_{0.0};
  double latency_max_processing_ms_{0.0};
  int latency_samples_{0};
  // 직전 스캔 콜백의 PF 처리 시간[ms]. 레이턴시 토픽 발행에 씁니다.
  double last_processing_ms_{0.0};
  // 맵 정합성 감시의 off-track 지표 EMA입니다(시간상수 = window).
  double map_inconsistency_ema_{0.0};
  // beam skip 합의붕괴 프레임 링버퍼(창 N, 1비트/프레임).
  std::deque<bool> beamskip_bad_frames_;
  int beamskip_bad_count_{0};
  // multi-scan용 스캔 히스토리. pose는 캡처 시점에 조회한 DR(odom) 프레임
  // 라이다 pose라 오래된 궤적 버퍼 조회가 필요 없습니다.
  struct ScanSnapshot {
    sensor_msgs::msg::LaserScan scan;
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };
  std::deque<ScanSnapshot> scan_history_;
  // 궤적 정합용 라이다 DR pose 이력(0.3 m 간격, 최근 trajectory_fit_history_m).
  std::deque<std::array<double, 3>> pose_history_;
  // 스캔 탐색 풀과 그 전용 난수원.
  std::vector<particle> scan_particles_;
  std::mt19937 scan_rng_{20260724u};

  // ---- ROS 입출력 ----
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_state_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr scan_particles_pub_;
  // 순간 슬립: 수치 토픽(rqt_plot용) + 로봇 위 색상 텍스트 마커(RViz용).
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr slip_marker_pub_;
  // relocalization 사유 안내: 맵 중앙에 사유별 색상 텍스트(3초 수명).
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr reloc_reason_pub_;
  // 최근 reloc 사유 이력 보드(맵 상단 고정, 수명 무한 갱신형).
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr reloc_history_pub_;
  std::deque<std::string> reloc_history_;
  // 벤치마크용 레이턴시 [interp_ms, processing_ms].
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr latency_pub_;
  // beam skip 시각화: 스킵된 빔만 담은 LaserScan(나머지 NaN).
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr skipped_scan_pub_;
  // 0=Lost, 1=Converging, 2=Tracking. transient_local이라 늦게 붙는 상위
  // 제어도 현재 상태를 바로 받습니다. Converging 동안 감속 여부는 상위가
  // 결정합니다.
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr output_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // base_link->laser를 자급 발행(static). tf_static이 비어 있는 bag(icra)
  // 에서도 laser 프레임이 항상 존재하게 합니다.
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};
