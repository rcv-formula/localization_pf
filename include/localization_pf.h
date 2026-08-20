#pragma once

#include <array>
#include <deque>
#include <future>
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
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
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
  ~mainNode();

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
  // RViz "2D Pose Estimate"(기본 /initialpose)로 사람이 찍은 pose입니다.
  // 전역 탐색을 대체하는 것이 아니라, 사람이 아는 위치를 직접 넣는
  // 별도 입구입니다. 어떤 상태에서든 받습니다.
  // 전역 레이스라인(map 프레임, 폐곡선, 인덱스 증가 = 주행 방향).
  // 헤딩 사전정보로 전역 탐색 후보를 거르는 데 씁니다.
  void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void initialpose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void vesc_state_callback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg);
  // 100Hz 출력 경로입니다. 궤적 버퍼를 읽기만 하므로 PF와 간섭하지 않습니다.
  void output_timer_callback();

  // scan 트리거 한 사이클: advance -> deskew -> propagate -> score -> resample
  //                        -> estimate -> 이방성 융합 -> map->odom 갱신
  void runFilterCycle(const sensor_msgs::msg::LaserScan &scan);
  // Lost 상태에서 전역 위치를 찾습니다. 성공하면 파티클을 다시 뿌립니다.
  // 전역 탐색은 수백 ms가 걸립니다. 단일 스레드 executor에서 콜백 안에 두면
  // 그동안 100 Hz 출력 타이머가 통째로 굶어 pose가 100 ms 넘게 끊깁니다.
  // 그래서 (a) 메인 스레드에서 입력만 스냅샷하고 (b) 워커 스레드에서 탐색을
  // 돌린 뒤 (c) 결과를 다시 메인 스레드에서 수확해 시드합니다.
  struct GlobalSearchRequest {
    std::vector<sensor_msgs::msg::LaserScan> scans;
    std::vector<Relocalization::RelativeMotion> motions;
    // 비어 있지 않으면 궤적 정합을 먼저 시도합니다.
    std::vector<std::array<double, 2>> trajectory_points;
    double laser_yaw{0.0};
    double trajectory_distance{0.0};
    double trajectory_rotation{0.0};
  };
  struct GlobalSearchResult {
    std::vector<Relocalization::Hypothesis> hypotheses;
    double elapsed_ms{0.0};
    std::size_t scan_points{0};
    bool from_trajectory{false};
    double trajectory_distance{0.0};
    double trajectory_rotation{0.0};
    // 워커 스레드에서는 로거를 만지지 않고, 메인 스레드에서 출력합니다.
    std::string error;
  };
  GlobalSearchRequest buildGlobalSearchRequest(
    const sensor_msgs::msg::LaserScan &scan);
  // 현재 EKF dead reckoning 기준 라이다 pose(파티클과 같은 프레임).
  bool currentLaserDrPose(double &x, double &y, double &yaw) const;
  // 스캔 요약값(유효 빔 평균 사거리)과 직전 시도 대비 변화량[m].
  static double scanSignature(const sensor_msgs::msg::LaserScan &scan);
  double scanChangeMetric(const sensor_msgs::msg::LaserScan &scan) const;
  // 전역 탐색을 돌려도 되는 관측인지. 유효점 비율 + 자세 + 연속 스캔
  // 자기일관성(DR SE(2) 보정 후 빔별 사거리 비교)을 모두 만족해야 한다.
  bool searchPreconditionsMet(const sensor_msgs::msg::LaserScan &scan,
                              std::string &reason);
  // 매 스캔 호출 — 연속 스캔 자기일관성 스트릭을 갱신합니다.
  void updateScanConsistency(const sensor_msgs::msg::LaserScan &scan);
  GlobalSearchResult executeGlobalSearch(const GlobalSearchRequest &request);
  // 워커가 끝났으면 결과를 수확해 시드합니다. 시드했으면 true.
  bool harvestRelocalization();
  // 감시 창의 스킵 중심이 맵에서 빠르게 움직였는가 = 움직이는 물체인가.
  // speed_out/ego_out은 로그용(각각 중심 속도, 에고 속도).
  bool skipLooksDynamic(double &speed_out, double &ego_out) const;
  void scoreAllParticles(particle *particles, int32_t particle_count);
  // 워커가 도는 중이면 끝날 때까지 기다립니다(맵 교체/종료 시 필수).
  void waitForRelocalizationWorker();
  // 상태 전이는 반드시 이 함수로 해서 상태 토픽과 어긋나지 않게 합니다.
  void setState(LocalizationState next);
  void publishState();
  // reloc 트리거 사유를 맵 중앙 텍스트 마커로 알립니다(사유별 색상).
  void announceReloc(const std::string &reason, float r, float g, float b);
  // 가설 목록으로 필터를 재초기화하는 공통 경로입니다(초기화/재수렴 공용).
  // position_std/yaw_std 가 양수이고 시드가 1개면 그 퍼짐으로 뿌립니다
  // (수동 시드용 — 사람 클릭의 불확실성은 검증된 리로컬 시드보다 훨씬 큽니다).
  // 음수면 필터 기본값(filter.init_*_std)을 씁니다.
  void seedFilter(
    const std::vector<ParticleFilter::ModeSeed> &seeds,
    double position_std = -1.0, double yaw_std = -1.0);
  // 이동/회전 간격을 만족할 때 스캔을 히스토리에 남깁니다(모든 상태 공용).
  void updateScanHistory(const sensor_msgs::msg::LaserScan &scan);
  // 히스토리가 있으면 multi-scan, 없으면 단일 스캔 전역 탐색을 돌립니다.
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
  // 최종 결과를 nav_msgs/Odometry로 내보낼 토픽. 관례상 /odom은 휠 오돔에
  // 쓰이므로, 충돌하는 스택에서는 이 값을 바꿔 주세요.
  std::string odom_topic_{"/odom"};
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
  // 직전 시도가 빈손일 때의 짧은 재시도 주기. 실패가 반복되는 구간은 대개
  // 입력이 나쁜 구간이라, 입력이 좋아지는 순간을 놓치지 않는 게 중요합니다.
  double relocalize_retry_period_s_{0.1};
  // 장면이 이만큼 바뀌면(평균 사거리 변화[m]) 주기를 무시하고 즉시 시도.
  double reloc_scan_change_m_{0.5};
  bool reloc_last_empty_{false};
  // ---- 수동 시드(RViz 2D Pose Estimate) ----
  std::string initialpose_topic_{"/initialpose"};
  // 수동 시드 직후 이 시간 동안은 전역 탐색을 돌리지 않습니다. 사람이 찍은
  // 자리에서 필터가 수렴할 시간을 주기 위한 것이고, 유예가 끝나면 평소대로
  // 자동 복구가 동작합니다(잘못 찍었어도 결국 스스로 빠져나옵니다).
  double manual_seed_grace_s_{5.0};
  // RViz가 실어 보내는 covariance의 표준편차 하한. RViz 도구 설정에 따라
  // 0이 실려 올 수 있는데, 그대로 쓰면 파티클이 한 점에 모여 수렴할 여지가
  // 사라집니다.
  double manual_seed_min_pos_std_{0.20};
  double manual_seed_min_yaw_std_{0.09};
  // 유예 시작 시각[s]. 음수면 유예 없음.
  double manual_seed_stamp_{-1.0};
  // ---- 전역 탐색 전제조건 ----
  //
  // 관측이 무의미한 구간에서 탐색을 돌리면 13만 후보 중 "반쯤 눈먼 스캔에
  // 그럴듯한" 앨리어스가 반드시 나오고, 게이트는 그 스캔만 보므로 막지
  // 못한다(실측 icra: 유효점 266/541, 붕괴 0.07초 뒤 시드, 17 m 오차,
  // 모든 게이트 통과). 스캔이 신뢰할 만해질 때까지 탐색 자체를 멈춘다.
  double search_min_valid_ratio_{0.6};
  double search_max_tilt_deg_{15.0};
  double search_consistency_inlier_m_{0.2};
  double search_consistency_fraction_{0.8};
  int search_consistency_frames_{3};
  int search_consistency_ok_count_{0};
  double last_consistency_fraction_{0.0};
  // 자기일관성 비교용 직전 스캔과 그 시각의 DR pose.
  sensor_msgs::msg::LaserScan prev_scan_;
  bool prev_scan_valid_{false};
  double prev_scan_dr_x_{0.0};
  double prev_scan_dr_y_{0.0};
  double prev_scan_dr_yaw_{0.0};
  // ---- 전복/미아 구간 DR 오차 계측 ----
  //
  // 전복은 kidnap이 아니라 '관측 정전'이다 — pose가 무효가 된 게 아니라
  // 관측이 무효가 된 것이고, 그동안의 이동은 물리적으로 유계다. 그렇다면
  // 전역 탐색 대신 마지막 정상 pose 주변 로컬 탐색으로 복구하는 게 맞는데,
  // 그 반경을 정하려면 "정전 구간을 통과한 DR이 얼마나 틀리는가"를 알아야
  // 한다. Lost 진입 시 (융합 pose, DR pose)를 기억해 두고, 복구 시드 때
  // 예측(마지막 융합 + 그동안의 DR)과 실제 시드의 차이를 남긴다.
  bool lost_anchor_valid_{false};
  double lost_fused_x_{0.0};
  double lost_fused_y_{0.0};
  double lost_fused_yaw_{0.0};
  double lost_dr_x_{0.0};
  double lost_dr_y_{0.0};
  double lost_dr_yaw_{0.0};
  double lost_stamp_{0.0};
  // 이번 Lost 에피소드에 IMU 자세 이벤트(전복/들림)가 있었는가. 있으면
  // heading prior가 정당하게 무효이므로 플립 조항을 면제한다.
  bool lost_imu_event_{false};
  // ---- 플립 조항 ----
  //
  // prior와 90도 넘게 다른 시드는 '비범한 주장'이라 비범한 증거를 요구한다.
  // 다만 하드 블록은 금지다 — 이전 시드가 플립이라 prior가 뒤집혀 있고 새
  // 후보가 교정인 경우, 교정 후보는 건강한 이력 지지를 갖고 있어 상향된
  // 기준을 통과한다(실측 busan2 정상 시드: 지지율 1.00). 조항이 교정을 막는
  // 경로는 없다.
  bool flip_clause_enabled_{true};
  double flip_clause_deg_{90.0};
  double flip_majority_fraction_{0.7};
  double flip_max_see_through_{0.10};
  // ---- 이력 다수결 밴드 + 보호관찰 ----
  //
  // 하드 이분법은 두 가지로 실패한다. (1) 교정 시드를 막는다 — 체인이 열화된
  // 상황에서는 정답도 지지 증거가 흐려진다(실측 icra seed3: 지지율 0.47).
  // (2) 데드락 — Lost 중에는 이력이 동결되므로 다음 재시도도 같은 뷰로 같은
  // 판정을 내려 영원히 거부한다.
  //
  // 그래서 애매한 구간은 거부가 아니라 '보호관찰'로 심는다: 적은 질량으로
  // 함께 심고 주행이 판별하게 한다. 맞으면 살아남아 지연이 0이고, 틀리면
  // 조용히 죽는다.
  bool majority_band_enabled_{false};
  double majority_accept_{0.60};
  double majority_probation_{0.25};
  double probation_mass_{0.3};
  // 데드락 보험: Lost가 이 시간을 넘고 그동안 모든 후보가 거부 밴드였다면
  // 최선 후보를 보호관찰로 승격한다. 반복을 '정답의 증거'로 쓰는 게 아니라
  // '게이트 교착의 증거'로 써서 중재로 넘기는 것이다.
  double majority_escape_s_{5.0};
  // 직전 시도에 쓴 스캔의 요약값(평균 사거리). 0이면 없음.
  double reloc_attempt_signature_{0.0};
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
  // 수렴 누적을 기하 관측성으로 가중할지. 끄면 기존 생거리 누적.
  bool converge_use_observability_{false};
  // 퇴화 구간 무한 대기 방지용 생(raw) 이동량 상한 배수.
  double converge_raw_cap_factor_{3.0};
  double converge_raw_translation_{0.0};
  double converge_raw_rotation_{0.0};
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
  // 이력 '저장' 용량. 사용 장수(multi_scan_count)와 분리한다 — 저장은 조밀하게
  // (0.1 m 간격, 서행에서도 증거가 굶지 않게), 사용은 그중 기하적으로 퍼진
  // 부분집합만. 60장 x 0.1 m = 최대 6 m 스팬으로, 검색 측 노후 컷(4 m)을 덮는다.
  int multi_scan_buffer_{60};
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
  std::future<GlobalSearchResult> reloc_future_;
  bool reloc_in_flight_{false};
  // 수동 시드가 들어오면 이미 떠 있는 탐색의 결과를 버립니다. 안 그러면
  // 사람이 찍은 pose를 나중에 도착한 탐색 결과가 조용히 덮어씁니다. 더
  // 나쁜 경우, 수동 시드로 Lost 를 빠져나가면 그 future 가 수확되지 않은
  // 채 남았다가 다음 Lost 에서 낡은 결과로 시드됩니다.
  bool reloc_discard_result_{false};
  // 탐색 요청을 만든 시점의 뷰별 자세[deg] (roll, pitch). 워커가 도는 동안
  // scan_history_ 는 계속 갱신되므로, 수확 시점에 그걸 읽으면 요청에 실린
  // 뷰와 어긋납니다. reloc_anchor_* 와 같은 이유로 요청 시점에 떠 둡니다.
  std::vector<std::array<double, 2>> reloc_view_tilt_;
  // 탐색을 시작한 순간의 라이다 DR pose. 탐색이 도는 동안에도 EKF는 계속
  // 전진하므로, 수확 시점에 그만큼의 이동을 가설에 합성해야 시드가 현재
  // 시각과 맞습니다. (동기 구현에서는 executor가 막혀 EKF도 함께 멈췄기
  // 때문에 이 보정이 필요 없었습니다.)
  bool reloc_anchor_valid_{false};
  double reloc_anchor_x_{0.0};
  double reloc_anchor_y_{0.0};
  double reloc_anchor_yaw_{0.0};
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
  // 스킵 빔 중심의 세계 좌표 궤적(감시 창과 같은 길이).
  //
  // 목적은 미아를 더 잘 잡는 게 아니라, 미아가 아닌 상황을 미아로 만드는
  // 것을 줄이는 것이다. 같은 속도로 앞서가는 상대차는 로봇 프레임에서
  // 정지라 창 조건을 그대로 통과하지만, 세계 좌표에서는 에고 속도로
  // 움직인다.
  //
  // 좌표는 직전 융합 pose(스캔 보정 포함, 맵 프레임)로 낸다. 이 환경은
  // 휠 슬립이 심해 dead reckoning으로는 정지 물체도 움직이는 것처럼 보인다.
  // 각 샘플이 자기 시점의 보정된 pose를 쓰므로 창 안에 DR 누적이 없고,
  // 감시가 무장된 구간은 아직 미아 확정 전이라 PF pose가 유효하다.
  struct SkipCentroidSample {
    double x{0.0};        // 스킵 중심 (odom)
    double y{0.0};
    double ego_x{0.0};    // 같은 시각의 라이다 DR pose (odom)
    double ego_y{0.0};
    double stamp{0.0};
    bool valid{false};
  };
  std::deque<SkipCentroidSample> beamskip_centroids_;
  bool dynamic_skip_reject_{false};
  double dynamic_skip_speed_mps_{0.3};
  double dynamic_skip_ego_ratio_{0.5};
  // 속도 추정의 최소 근거. 표본이 적거나 baseline이 짧으면 판단하지 않는다.
  int dynamic_skip_min_samples_{4};
  double dynamic_skip_min_dt_{0.1};
  // multi-scan용 스캔 히스토리. pose는 캡처 시점에 조회한 DR(odom) 프레임
  // 라이다 pose라 오래된 궤적 버퍼 조회가 필요 없습니다.
  struct ScanSnapshot {
    sensor_msgs::msg::LaserScan scan;
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    // 스냅샷 시점의 차대 자세(시동 자세 대비 상대)[deg]. 2D 라이다는 차대에
    // 고정돼 있으므로 roll/pitch 가 곧 스캔 평면의 기울기이고, 뷰마다 다른
    // 평면을 본 것을 같은 평면으로 취급하면 뷰별 RMS 가 pose 오차가 아니라
    // 자세 차이를 재게 됩니다. yaw 만으로는 이 정보가 남지 않습니다.
    double roll_deg{0.0};
    double pitch_deg{0.0};
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
  // RViz "2D Pose Estimate". 사람이 드물게 한 번 쏘는 입력이라 depth 1.
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_sub_;
  // 전역 경로는 보통 latch(transient_local)로 한 번 발행됩니다.
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  std::string global_path_topic_{"/global_path"};
  bool global_path_ready_{false};
  // 재발행 감지용 서명(점 수 + 시작/중간/끝 좌표).
  std::array<double, 6> global_path_signature_{};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_state_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  // 최종 pose를 nav_msgs/Odometry로도 발행합니다(frame: map, child: base_link).
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  // twist(body frame)를 궤적 버퍼 차분으로 만들기 위한 직전 출력 샘플입니다.
  double odom_prev_time_{-1.0};
  Pose2D odom_prev_pose_{};
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
  // 신뢰도 진단: pose와 같은 stamp로 발행해 상위에서 짝지을 수 있게 합니다.
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr confidence_pub_;
  // 스캔 사이클(40Hz)에서 계산한 신뢰도 재료를 출력 타이머(100Hz)로 넘기는
  // 캐시입니다. 로그우도 단독으로는 판별이 안 되므로(맵/빔 수에 따라 절대값이
  // 달라짐) 정규화 지표와 구름 분산을 함께 봅니다.
  double last_score_health_{1.0};     // score 정규화 [0,1]
  double last_outlier_fraction_{0.0}; // 아웃라이어 빔 비율 [0,1]
  double last_skip_fraction_{0.0};    // beam skip 제안 비율 [0,1]
  double last_pos_sigma_{0.0};        // 파티클 구름 위치 표준편차 [m]
  double last_dominant_mass_{0.0};    // 지배 가설 질량 [0,1]
  // 위 재료를 하나로 합친 신뢰도 [0,1]. 약한 고리(min) 방식이라 어느 하나가
  // 나빠지면 값이 떨어집니다.
  double localizationConfidence() const;
  // 신뢰도의 재료 5항(각 [0,1], 상태 배율 적용 전). min이 어느 항인지가
  // 소비자에게 중요하다 — "모호해서 낮음"(mass)과 "정합이 나빠서 낮음"(score)
  // 은 반대 대응을 요구한다(참을성 있는 대기 vs 즉시 기각).
  struct ConfidenceTerms {
    double score{1.0};
    double outlier{1.0};
    double skip{1.0};
    double spread{1.0};
    double mass{1.0};
  };
  ConfidenceTerms confidenceTerms() const;
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
