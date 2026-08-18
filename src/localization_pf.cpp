#include "localization_pf.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double normalizeAngle(double angle) {
  angle = std::fmod(angle + kPi, kTwoPi);
  if (angle < 0.0) {
    angle += kTwoPi;
  }
  return angle - kPi;
}

double stampToSeconds(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) +
    static_cast<double>(stamp.nanosec) * 1.0e-9;
}

builtin_interfaces::msg::Time secondsToStamp(double seconds) {
  builtin_interfaces::msg::Time stamp;
  if (seconds <= 0.0) {
    return stamp;
  }
  stamp.sec = static_cast<int32_t>(std::floor(seconds));
  stamp.nanosec = static_cast<uint32_t>(
    (seconds - static_cast<double>(stamp.sec)) * 1.0e9);
  if (stamp.nanosec >= 1000000000U) {
    stamp.sec += 1;
    stamp.nanosec -= 1000000000U;
  }
  return stamp;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

}  // namespace

mainNode::mainNode()
: Node("localization_pf") {
  declareParameters();

  propagation_ = std::make_unique<particlePropagation>();
  propagation_->setEkfParameters(ekf_parameters_);
  propagation_->setImuExtrinsic(imu_extrinsic_);
  propagation_->setLaserExtrinsic(laser_extrinsic_);
  propagation_->setMotionNoise(motion_noise_);

  filter_.setParameters(filter_parameters_);
  estimation_.setParameters(estimation_parameters_);
  deskew_.setParameters(deskew_parameters_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "localization_pf/pose", 10);
  // 같은 최종 pose를 nav_msgs/Odometry로도 발행합니다(상위 스택 호환용).
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "localization_pf/particles", 1);
  // 늦게 붙는 상위 제어도 현재 상태를 바로 받도록 transient_local로 둡니다.
  scan_particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "localization_pf/scan_particles", 1);
  slip_pub_ = this->create_publisher<std_msgs::msg::Float32>(
    "localization_pf/slip", 10);
  skipped_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "localization_pf/skipped_scan", rclcpp::SensorDataQoS());
  latency_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
    "localization_pf/latency", 10);
  // pose/odom과 같은 stamp로 발행되는 신뢰도 진단입니다. std_msgs 에는
  // header가 없으므로 stamp를 배열 앞 두 칸에 담습니다.
  confidence_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "localization_confidence", 10);
  slip_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "localization_pf/slip_marker", 1);
  reloc_reason_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "localization_pf/reloc_reason", 1);
  reloc_history_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "localization_pf/reloc_history", 1);
  state_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
    "localization_pf/state", rclcpp::QoS(1).transient_local());
  publishState();

  // ROS 입출력은 노드가 소유하고 각 구성 요소에는 메시지만 전달합니다.
  // 자체 맵 로더가 켜져 있으면 파일에서 맵을 읽어 쓰고 latch 발행합니다.
  // 꺼져 있으면 외부 map_server의 map_topic을 구독합니다.
  if (map_loader_enabled_) {
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local());
    if (!loadMapFromFile(map_loader_dir_, map_loader_name_)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "map loader enabled but failed to load '%s' from '%s' — "
        "falling back to subscribing %s.",
        map_loader_name_.c_str(), map_loader_dir_.c_str(), map_topic_.c_str());
      map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(1).transient_local(),
        std::bind(&mainNode::map_callback, this, std::placeholders::_1));
    }
  } else {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&mainNode::map_callback, this, std::placeholders::_1));
  }

  // 센서 입력은 best-effort로 받습니다. reliable 구독자는 best-effort 퍼블리셔의
  // 메시지를 아예 받지 못하지만(QoS 비호환), best-effort 구독자는 reliable
  // 퍼블리셔의 것도 받습니다. 드라이버/재생기 어느 쪽에도 붙으려면 이쪽입니다.
  initialpose_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_, 1,
      std::bind(&mainNode::initialpose_callback, this, std::placeholders::_1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS().keep_last(10),
    std::bind(&mainNode::scan_callback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::SensorDataQoS().keep_last(100),
    std::bind(&mainNode::imu_callback, this, std::placeholders::_1));

  vesc_state_sub_ = this->create_subscription<vesc_msgs::msg::VescStateStamped>(
    vesc_state_topic_, rclcpp::SensorDataQoS().keep_last(50),
    std::bind(&mainNode::vesc_state_callback, this, std::placeholders::_1));

  // 출력 경로는 궤적 버퍼를 읽기만 하므로 PF 주기와 독립적으로 돕니다.
  const double period = 1.0 / std::max(1.0, output_rate_hz_);
  output_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(period),
    std::bind(&mainNode::output_timer_callback, this));

  last_relocalize_attempt_ = this->now();
  node_start_seconds_ = this->now().seconds();
}

void mainNode::declareParameters() {
  vesc_state_topic_ = this->declare_parameter<std::string>(
    "vesc_state_topic", "/sensors/core");
  scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");
  imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu/data");
  map_topic_ = this->declare_parameter<std::string>("map_topic", "/map");
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
  // 자체 맵 로더. map_dir는 보통 launch가 패키지 map 폴더로 주입합니다.
  map_loader_enabled_ = this->declare_parameter<bool>("map_loader.enabled", true);
  map_loader_dir_ = this->declare_parameter<std::string>("map_loader.map_dir", "");
  map_loader_name_ = this->declare_parameter<std::string>("map_loader.map_name", "map");
  // RViz "2D Pose Estimate" 입력.
  initialpose_topic_ = this->declare_parameter<std::string>(
    "initialpose_topic", "/initialpose");
  manual_seed_grace_s_ = std::max(0.0, this->declare_parameter<double>(
    "relocalization.manual_seed_grace_s", 5.0));
  manual_seed_min_pos_std_ = std::max(0.01, this->declare_parameter<double>(
    "relocalization.manual_seed_min_pos_std", 0.20));
  manual_seed_min_yaw_std_ = std::max(0.005, this->declare_parameter<double>(
    "relocalization.manual_seed_min_yaw_std", 0.09));
  map_frame_ = this->declare_parameter<std::string>("frames.map", "map");
  odom_frame_ = this->declare_parameter<std::string>("frames.odom", "odom");
  base_frame_ = this->declare_parameter<std::string>("frames.base", "base_link");
  laser_frame_ = this->declare_parameter<std::string>("frames.laser", "laser");
  output_rate_hz_ = this->declare_parameter<double>("output.rate_hz", 100.0);
  publish_particles_ = this->declare_parameter<bool>("output.publish_particles", true);
  publish_laser_tf_ = this->declare_parameter<bool>("output.publish_laser_tf", true);

  // ---- relocalization ----
  // 처리 경로 고정(단일 파이프라인): unknown=occupied, MaskedMSE 채점,
  // 비강건 PCA 축, coarse-to-fine 탐색 — 분기·config 노출 없이 구조체
  // 기본값을 그대로 씁니다.
  relocalization_parameters_.occupied_threshold = this->declare_parameter<int>(
    "relocalization.occupied_threshold", 65);
  relocalization_parameters_.ray_count = static_cast<std::size_t>(
    std::max<int>(4, this->declare_parameter<int>("relocalization.ray_count", 720)));
  relocalization_parameters_.raycast_batch_size = static_cast<std::size_t>(
    std::max<int>(1, this->declare_parameter<int>(
      "relocalization.raycast_batch_size", 512)));
  relocalization_parameters_.minimum_scan_points = static_cast<std::size_t>(
    std::max<int>(1, this->declare_parameter<int>(
      "relocalization.minimum_scan_points", 10)));
  relocalization_parameters_.max_range = this->declare_parameter<double>(
    "relocalization.max_range", 30.0);
  relocalization_parameters_.fov_deg = this->declare_parameter<double>(
    "relocalization.fov_deg", 270.0);
  relocalization_parameters_.axis_run_tolerance_deg = this->declare_parameter<double>(
    "relocalization.axis_run_tolerance_deg", 3.0);
  relocalization_parameters_.axis_match_tolerance_deg = this->declare_parameter<double>(
    "relocalization.axis_match_tolerance_deg", 1.0);
  relocalization_parameters_.yaw_refine_bins = std::max<int>(
    0, this->declare_parameter<int>("relocalization.yaw_refine_bins", 2));
  relocalization_parameters_.max_heading_hypotheses = static_cast<std::size_t>(
    std::max<int>(1, this->declare_parameter<int>(
      "relocalization.max_heading_hypotheses", 12)));
  relocalization_parameters_.max_hypotheses = static_cast<std::size_t>(
    std::max<int>(1, this->declare_parameter<int>(
      "relocalization.max_hypotheses", 5)));
  relocalization_parameters_.hypothesis_score_ratio = this->declare_parameter<double>(
    "relocalization.hypothesis_score_ratio", 3.0);
  relocalization_parameters_.hypothesis_min_separation_m = this->declare_parameter<double>(
    "relocalization.hypothesis_min_separation_m", 0.8);
  relocalization_parameters_.hypothesis_min_separation_deg = this->declare_parameter<double>(
    "relocalization.hypothesis_min_separation_deg", 25.0);
  relocalization_parameters_.hypothesis_max_rms_m = this->declare_parameter<double>(
    "relocalization.hypothesis_max_rms_m", 1.0);
  relocalization_parameters_.hypothesis_verify_fraction = std::clamp(
    this->declare_parameter<double>(
      "relocalization.hypothesis_verify_fraction", 0.7), 0.0, 1.0);
  relocalization_parameters_.hypothesis_verify_inlier_m = std::max(0.05,
    this->declare_parameter<double>(
      "relocalization.hypothesis_verify_inlier_m", 0.2));
  relocalization_parameters_.verify_signed_gate = this->declare_parameter<bool>(
    "relocalization.verify_signed_gate", false);
  relocalization_parameters_.verify_occlusion_m = std::max(0.05,
    this->declare_parameter<double>("relocalization.verify_occlusion_m", 0.2));
  relocalization_parameters_.verify_see_through_m = std::max(0.05,
    this->declare_parameter<double>("relocalization.verify_see_through_m", 0.25));
  relocalization_parameters_.verify_see_through_max = std::clamp(
    this->declare_parameter<double>("relocalization.verify_see_through_max", 0.10),
    0.0, 1.0);
  relocalization_parameters_.verify_visible_fraction = std::clamp(
    this->declare_parameter<double>("relocalization.verify_visible_fraction", 0.8),
    0.0, 1.0);
  relocalization_parameters_.verify_min_visible_frac = std::clamp(
    this->declare_parameter<double>("relocalization.verify_min_visible_frac", 0.40),
    0.0, 1.0);
  relocalization_parameters_.verify_min_sector_beams =
    static_cast<std::size_t>(std::max<int>(1, this->declare_parameter<int>(
      "relocalization.verify_min_sector_beams", 15)));
  relocalization_parameters_.verify_min_visible_sectors =
    static_cast<std::size_t>(std::max<int>(0, this->declare_parameter<int>(
      "relocalization.verify_min_visible_sectors", 4)));
  relocalization_parameters_.normal_chord_m = this->declare_parameter<double>(
    "relocalization.normal_chord_m", 0.30);
  relocalization_parameters_.normal_max_half_window = std::max<int>(
    1, this->declare_parameter<int>("relocalization.normal_max_half_window", 20));
  relocalization_parameters_.normal_residual_limit_m = this->declare_parameter<double>(
    "relocalization.normal_residual_limit_m", 0.06);
  relocalization_parameters_.observability_reference = this->declare_parameter<double>(
    "relocalization.observability_reference", 40.0);
  relocalization_parameters_.coarse_position_step_m = this->declare_parameter<double>(
    "relocalization.coarse_position_step_m", 0.30);
  relocalization_parameters_.fine_position_radius_m = this->declare_parameter<double>(
    "relocalization.fine_position_radius_m", 0.30);
  relocalization_parameters_.top_candidates = static_cast<std::size_t>(
    std::max<int>(1, this->declare_parameter<int>("relocalization.top_candidates", 24)));
  relocalize_retry_period_s_ = std::max(0.0, this->declare_parameter<double>(
    "relocalization.empty_retry_period_s", relocalize_retry_period_s_));
  reloc_scan_change_m_ = std::max(0.0, this->declare_parameter<double>(
    "relocalization.scan_change_trigger_m", reloc_scan_change_m_));
  flip_clause_enabled_ = this->declare_parameter<bool>(
    "relocalization.flip_clause_enabled", true);
  flip_clause_deg_ = std::max(0.0, this->declare_parameter<double>(
    "relocalization.flip_clause_deg", 90.0));
  flip_majority_fraction_ = std::clamp(this->declare_parameter<double>(
    "relocalization.flip_majority_fraction", 0.7), 0.0, 1.0);
  flip_max_see_through_ = std::clamp(this->declare_parameter<double>(
    "relocalization.flip_max_see_through", 0.10), 0.0, 1.0);
  majority_band_enabled_ = this->declare_parameter<bool>(
    "relocalization.majority_band_enabled", false);
  majority_accept_ = std::clamp(this->declare_parameter<double>(
    "relocalization.majority_accept", 0.60), 0.0, 1.0);
  majority_probation_ = std::clamp(this->declare_parameter<double>(
    "relocalization.majority_probation", 0.25), 0.0, 1.0);
  probation_mass_ = std::clamp(this->declare_parameter<double>(
    "relocalization.probation_mass", 0.3), 0.01, 1.0);
  majority_escape_s_ = std::max(0.0, this->declare_parameter<double>(
    "relocalization.majority_escape_s", 5.0));
  search_min_valid_ratio_ = std::clamp(this->declare_parameter<double>(
    "relocalization.search_min_valid_ratio", 0.6), 0.0, 1.0);
  search_max_tilt_deg_ = std::max(0.0, this->declare_parameter<double>(
    "relocalization.search_max_tilt_deg", 15.0));
  search_consistency_inlier_m_ = std::max(0.01, this->declare_parameter<double>(
    "relocalization.search_consistency_inlier_m", 0.2));
  search_consistency_fraction_ = std::clamp(this->declare_parameter<double>(
    "relocalization.search_consistency_fraction", 0.8), 0.0, 1.0);
  search_consistency_frames_ = std::max(1, static_cast<int>(
    this->declare_parameter<int>("relocalization.search_consistency_frames", 3)));
  relocalize_period_s_ = this->declare_parameter<double>(
    "relocalization.retry_period_s", 1.0);

  // ---- 관측성(기하 퇴화) 사전계산 ----
  observability_parameters_.grid_step_m = this->declare_parameter<double>(
    "observability.grid_step_m", 0.30);
  observability_parameters_.heading_bins = std::max<int>(
    1, this->declare_parameter<int>("observability.heading_bins", 32));
  observability_parameters_.clearance_m = this->declare_parameter<double>(
    "observability.clearance_m", 0.15);
  // 실주행 맵에서 로봇은 unknown에 들어가지 않으므로 격자점은 free 셀에만
  // 둡니다. raycast는 unknown을 통과시킵니다(벽 아님).
  observability_parameters_.include_unknown = false;
  observability_parameters_.max_range_m = this->declare_parameter<double>(
    "observability.max_range_m", 5.0);
  observability_parameters_.slip_tolerance_m = this->declare_parameter<double>(
    "observability.slip_tolerance_m", 0.10);
  observability_parameters_.max_sigma_m = this->declare_parameter<double>(
    "observability.max_sigma_m", 10.0);
  observability_parameters_.direction_cluster_deg = this->declare_parameter<double>(
    "observability.direction_cluster_deg", 5.0);
  // sigma[m] -> confidence[0,1] 사상 기준. sigma가 이 값일 때 0.5입니다.
  observability_reference_sigma_ = this->declare_parameter<double>(
    "observability.reference_sigma_m", 1.0);
  observability_fallback_confidence_ = std::clamp(this->declare_parameter<double>(
    "observability.fallback_confidence", 0.5), 0.0, 1.0);

  // ---- 다중 가설 수렴/감시 ----
  converge_mass_ = std::clamp(this->declare_parameter<double>(
    "tracking.converge_mass", 0.9), 0.0, 1.0);
  converge_distance_m_ = std::max(0.0, this->declare_parameter<double>(
    "tracking.converge_distance_m", 1.0));
  converge_rotation_deg_ = std::max(0.0, this->declare_parameter<double>(
    "tracking.converge_rotation_deg", 90.0));
  converge_use_observability_ = this->declare_parameter<bool>(
    "tracking.converge_use_observability", false);
  converge_raw_cap_factor_ = std::max(1.0, this->declare_parameter<double>(
    "tracking.converge_raw_cap_factor", 3.0));
  score_fail_threshold_ = this->declare_parameter<double>(
    "tracking.score_fail_threshold", -9.0);
  multi_scan_buffer_ = std::max<int>(2, this->declare_parameter<int>(
    "relocalization.multi_scan_buffer", 60));
  multi_scan_count_ = std::max<int>(2, this->declare_parameter<int>(
    "relocalization.multi_scan_count", 12));
  multi_scan_spacing_m_ = std::max(0.05, this->declare_parameter<double>(
    "relocalization.multi_scan_spacing_m", 0.5));
  multi_scan_spacing_deg_ = std::max(1.0, this->declare_parameter<double>(
    "relocalization.multi_scan_spacing_deg", 15.0));
  relocalization_parameters_.history_support_fraction = std::clamp(
    this->declare_parameter<double>(
      "relocalization.history_support_fraction", 0.6), 0.0, 1.0);
  relocalization_parameters_.history_valid_dr_max_m = std::max(0.0,
    this->declare_parameter<double>(
      "relocalization.history_valid_dr_max_m", 4.0));
  relocalization_parameters_.history_min_visible_frac = std::clamp(
    this->declare_parameter<double>(
      "relocalization.history_min_visible_frac", 0.5), 0.0, 1.0);
  relocalization_parameters_.history_majority_gate = this->declare_parameter<bool>(
    "relocalization.history_majority_gate", false);
  relocalization_parameters_.history_pass_ratio = std::max(1.0,
    this->declare_parameter<double>("relocalization.history_pass_ratio", 1.5));
  relocalization_parameters_.history_pass_slack_m = std::max(0.0,
    this->declare_parameter<double>("relocalization.history_pass_slack_m", 0.3));
  relocalization_parameters_.history_view_useless_m = std::max(0.0,
    this->declare_parameter<double>("relocalization.history_view_useless_m", 1.5));
  relocalization_parameters_.history_weight_low_f = std::clamp(
    this->declare_parameter<double>("relocalization.history_weight_low_f", 0.3),
    0.0, 1.0);
  relocalization_parameters_.history_weight_high_f = std::clamp(
    this->declare_parameter<double>("relocalization.history_weight_high_f", 0.7),
    0.0, 1.0);
  relocalization_parameters_.history_majority_fraction = std::clamp(
    this->declare_parameter<double>(
      "relocalization.history_majority_fraction", 0.5), 0.0, 1.0);
  relocalization_parameters_.history_min_support_weight = std::max(0.0,
    this->declare_parameter<double>(
      "relocalization.history_min_support_weight", 2.0));
  relocalization_parameters_.history_new_aggregate = this->declare_parameter<bool>(
    "relocalization.history_new_aggregate", false);
  relocalization_parameters_.multi_scan_top_candidates = static_cast<std::size_t>(
    std::max<int>(4, this->declare_parameter<int>(
      "relocalization.multi_scan_top_candidates", 64)));
  map_consistency_window_s_ = std::max(0.5, this->declare_parameter<double>(
    "tracking.map_consistency_window_s", 5.0));
  map_consistency_threshold_ = std::clamp(this->declare_parameter<double>(
    "tracking.map_consistency_threshold", 0.3), 0.01, 1.0);
  outlier_prob_ = std::clamp(this->declare_parameter<double>(
    "scoring.outlier_prob", 0.1), 1.0e-6, 1.0);
  scoring_beam_stride_ = std::max<int>(1, this->declare_parameter<int>(
    "scoring.beam_stride", scoring_beam_stride_));
  scoring_sharp_sigma_ = std::max(0.01, this->declare_parameter<double>(
    "scoring.sharp_sigma", scoring_sharp_sigma_));
  scoring_sharp_weight_ = std::clamp(this->declare_parameter<double>(
    "scoring.sharp_weight", scoring_sharp_weight_), 0.0, 1.0);
  scoring_threads_ = std::max<int>(0, this->declare_parameter<int>(
    "scoring.threads", scoring_threads_));
  beam_skip_prob_ = std::clamp(this->declare_parameter<double>(
    "scoring.beam_skip_prob", beam_skip_prob_), 1.0e-6, 1.0);
  beam_skip_consensus_ = std::clamp(this->declare_parameter<double>(
    "scoring.beam_skip_consensus", beam_skip_consensus_), 0.0, 1.0);
  beam_skip_error_threshold_ = std::clamp(this->declare_parameter<double>(
    "scoring.beam_skip_error_threshold", beam_skip_error_threshold_), 0.0, 1.0);
  beam_skip_particle_stride_ = std::max<int>(1, this->declare_parameter<int>(
    "scoring.beam_skip_particle_stride", beam_skip_particle_stride_));
  beamskip_lost_fraction_ = std::clamp(this->declare_parameter<double>(
    "tracking.beamskip_lost_fraction", beamskip_lost_fraction_), 0.0, 1.0);
  dynamic_skip_reject_ = this->declare_parameter<bool>(
    "tracking.dynamic_skip_reject", dynamic_skip_reject_);
  dynamic_skip_speed_mps_ = std::max(0.0, this->declare_parameter<double>(
    "tracking.dynamic_skip_speed_mps", dynamic_skip_speed_mps_));
  dynamic_skip_ego_ratio_ = std::max(0.0, this->declare_parameter<double>(
    "tracking.dynamic_skip_ego_ratio", dynamic_skip_ego_ratio_));
  dynamic_skip_min_samples_ = std::max(2, static_cast<int>(
    this->declare_parameter<int>(
      "tracking.dynamic_skip_min_samples", dynamic_skip_min_samples_)));
  dynamic_skip_min_dt_ = std::max(0.0, this->declare_parameter<double>(
    "tracking.dynamic_skip_min_dt", dynamic_skip_min_dt_));
  beamskip_lost_window_ = std::max<int>(1, this->declare_parameter<int>(
    "tracking.beamskip_lost_window", beamskip_lost_window_));
  beamskip_lost_count_ = std::max<int>(1, this->declare_parameter<int>(
    "tracking.beamskip_lost_count", beamskip_lost_count_));
  outlier_frac_good_ = std::clamp(this->declare_parameter<double>(
    "tracking.outlier_frac_good", 0.15), 0.0, 1.0);
  outlier_frac_bad_ = std::clamp(this->declare_parameter<double>(
    "tracking.outlier_frac_bad", 0.5), 0.01, 1.0);
  scan_particle_count_ = std::max<int>(0, this->declare_parameter<int>(
    "filter.scan_particle_count", 100));
  scan_particle_spread_pos_ = std::max(0.05, this->declare_parameter<double>(
    "filter.scan_particle_spread_pos", 0.5));
  scan_particle_spread_yaw_ = this->declare_parameter<double>(
    "filter.scan_particle_spread_yaw_deg", 15.0) * kPi / 180.0;
  scan_particle_inject_margin_ = std::max(0.0, this->declare_parameter<double>(
    "filter.scan_particle_inject_margin", 0.5));
  scan_particle_inject_count_ = std::max<int>(0, this->declare_parameter<int>(
    "filter.scan_particle_inject_count", 20));
  motion_noise_.min_trans_sigma = this->declare_parameter<double>(
    "motion_noise.min_trans_sigma", 0.02);
  motion_noise_.min_yaw_sigma = this->declare_parameter<double>(
    "motion_noise.min_yaw_sigma_deg", 0.3) * kPi / 180.0;
  trajectory_fit_min_distance_m_ = std::max(1.0, this->declare_parameter<double>(
    "relocalization.trajectory_fit_min_distance_m", 8.0));
  trajectory_fit_min_rotation_deg_ = std::max(0.0, this->declare_parameter<double>(
    "relocalization.trajectory_fit_min_rotation_deg", 180.0));
  trajectory_fit_history_m_ = std::max(5.0, this->declare_parameter<double>(
    "relocalization.trajectory_fit_history_m", 30.0));

  // ---- EKF (모션 모델) ----
  ekf_parameters_.wheel_scale = this->declare_parameter<double>(
    "ekf.wheel_scale", 1.0);
  // 휠 주행거리 산출 경로: 기본은 vesc_to_odom과 동일한 ERPM 경로입니다.
  ekf_parameters_.wheel_use_displacement = this->declare_parameter<bool>(
    "ekf.wheel_use_displacement", false);
  ekf_parameters_.speed_to_erpm_gain = this->declare_parameter<double>(
    "ekf.speed_to_erpm_gain", 1538.0);
  ekf_parameters_.speed_to_erpm_offset = this->declare_parameter<double>(
    "ekf.speed_to_erpm_offset", 0.0);
  ekf_parameters_.erpm_speed_deadband = this->declare_parameter<double>(
    "ekf.erpm_speed_deadband", 0.05);
  ekf_parameters_.velocity_deadzone = this->declare_parameter<double>(
    "ekf.velocity_deadzone", 0.03);
  ekf_parameters_.max_wheel_speed = this->declare_parameter<double>(
    "ekf.max_wheel_speed", 30.0);
  ekf_parameters_.history_keep_time = this->declare_parameter<double>(
    "ekf.history_keep_time", 1.0);
  ekf_parameters_.max_imu_extrapolation = this->declare_parameter<double>(
    "ekf.max_imu_extrapolation", 0.02);
  ekf_parameters_.max_wheel_extrapolation = this->declare_parameter<double>(
    "ekf.max_wheel_extrapolation", 0.03);
  ekf_parameters_.wheel_velocity_variance = this->declare_parameter<double>(
    "ekf.wheel_velocity_variance", 0.03 * 0.03);
  ekf_parameters_.lateral_accel_variance = this->declare_parameter<double>(
    "ekf.lateral_accel_variance", 0.35 * 0.35);
  ekf_parameters_.slip_gain = this->declare_parameter<double>("ekf.slip_gain", 20.0);
  ekf_parameters_.dr_use_imu_tilt = this->declare_parameter<bool>(
    "ekf.dr_use_imu_tilt", true);
  ekf_parameters_.dr_tilt_max_deg = this->declare_parameter<double>(
    "ekf.dr_tilt_max_deg", 45.0);
  ekf_parameters_.enable_startup_gravity_calibration = this->declare_parameter<bool>(
    "ekf.enable_startup_gravity_calibration", true);
  ekf_parameters_.gravity_calibration_path = this->declare_parameter<std::string>(
    "ekf.gravity_calibration_path", "");
  ekf_parameters_.process_position_std = this->declare_parameter<double>(
    "ekf.process_position_std", 0.02);
  ekf_parameters_.process_yaw_std = this->declare_parameter<double>(
    "ekf.process_yaw_std", 0.01);
  ekf_parameters_.process_velocity_std = this->declare_parameter<double>(
    "ekf.process_velocity_std", 0.20);

  imu_extrinsic_.rear_to_imu_x = this->declare_parameter<double>(
    "extrinsic.rear_to_imu_x", 0.25);
  imu_extrinsic_.rear_to_imu_y = this->declare_parameter<double>(
    "extrinsic.rear_to_imu_y", 0.0);
  imu_extrinsic_.rear_to_imu_z = this->declare_parameter<double>(
    "extrinsic.rear_to_imu_z", 0.0);
  // TF 조회에 성공하면 덮어쓰므로 여기 값은 fallback입니다.
  laser_extrinsic_.rear_to_laser_x = this->declare_parameter<double>(
    "extrinsic.rear_to_laser_x", 0.25);
  laser_extrinsic_.rear_to_laser_y = this->declare_parameter<double>(
    "extrinsic.rear_to_laser_y", 0.0);

  // ---- 파티클 모션 노이즈 ----
  motion_noise_.trans_per_trans = this->declare_parameter<double>(
    "motion_noise.trans_per_trans", 0.15);
  motion_noise_.lateral_per_trans = this->declare_parameter<double>(
    "motion_noise.lateral_per_trans", 0.05);
  motion_noise_.rot_per_rot = this->declare_parameter<double>(
    "motion_noise.rot_per_rot", 0.15);
  motion_noise_.rot_per_trans = this->declare_parameter<double>(
    "motion_noise.rot_per_trans", 0.03);
  motion_noise_.degeneracy_gain = this->declare_parameter<double>(
    "motion_noise.degeneracy_gain", 2.0);
  motion_noise_.recovery_gain = this->declare_parameter<double>(
    "motion_noise.recovery_gain", 3.0);
  score_good_ = this->declare_parameter<double>("tracking.score_good", -1.0);

  // ---- 파티클 필터 ----
  filter_parameters_.particle_count = std::max<int>(
    1, this->declare_parameter<int>("filter.particle_count", 1000));
  filter_parameters_.likelihood_scale = this->declare_parameter<double>(
    "filter.likelihood_scale", 1.0);
  filter_parameters_.resample_neff_ratio = this->declare_parameter<double>(
    "filter.resample_neff_ratio", 0.5);
  filter_parameters_.init_position_std = this->declare_parameter<double>(
    "filter.init_position_std", 0.10);
  filter_parameters_.init_yaw_std = this->declare_parameter<double>(
    "filter.init_yaw_std", 0.05);
  resample_min_translation_ = this->declare_parameter<double>(
    "filter.resample_min_translation", 0.02);
  resample_min_rotation_ = this->declare_parameter<double>(
    "filter.resample_min_rotation", 0.01);

  scoring_factor_ = static_cast<float>(
    this->declare_parameter<double>("scoring.factor", 0.2));

  // ---- 이방성 융합 ---- (항상 활성; 경로 고정)
  estimation_parameters_.enabled = true;
  estimation_parameters_.scan_trust = this->declare_parameter<double>(
    "estimation.scan_trust", 1.0);
  estimation_parameters_.yaw_scan_trust = this->declare_parameter<double>(
    "estimation.yaw_scan_trust", 1.0);
  estimation_parameters_.odom_position_growth = this->declare_parameter<double>(
    "estimation.odom_position_growth", 0.10);
  estimation_parameters_.odom_yaw_growth = this->declare_parameter<double>(
    "estimation.odom_yaw_growth", 0.05);
  estimation_parameters_.odom_yaw_growth_per_meter = this->declare_parameter<double>(
    "estimation.odom_yaw_growth_per_meter", 0.02);

  // ---- deskew ---- (항상 활성; 경로 고정)
  deskew_parameters_.enabled = true;
  deskew_parameters_.fallback_sweep_duration = this->declare_parameter<double>(
    "deskew.fallback_sweep_duration", 0.025);
  deskew_parameters_.max_lookup_failure_ratio = this->declare_parameter<double>(
    "deskew.max_lookup_failure_ratio", 0.2);
}

void mainNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // map_server는 transient_local로 같은 맵을 반복 전달하고, 데이터셋 재생기는
  // 주기적으로 다시 퍼블리시합니다. 내용이 같으면 사전계산(수 초)과 추적
  // 초기화를 건너뛰어야 합니다. 안 그러면 계속 Lost로 떨어져 추적이 못 붙습니다.
  if (relocalization_ && scoring_ &&
    map_.info.width == msg->info.width &&
    map_.info.height == msg->info.height &&
    map_.info.resolution == msg->info.resolution &&
    map_.info.origin.position.x == msg->info.origin.position.x &&
    map_.info.origin.position.y == msg->info.origin.position.y &&
    map_.info.origin.orientation.z == msg->info.origin.orientation.z &&
    map_.info.origin.orientation.w == msg->info.origin.orientation.w &&
    map_.data == msg->data)
  {
    return;
  }
  setupMap(*msg);
}

bool mainNode::setupMap(const nav_msgs::msg::OccupancyGrid &grid) {
  // relocalization_ 의 맵을 갈아끼우기 전에 워커가 그것을 쓰고 있지
  // 않은지 보장해야 합니다.
  waitForRelocalizationWorker();

  map_ = grid;

  try {
    // 첫 맵에서는 객체를 만들고, 이후 맵 갱신은 같은 객체를 다시 계산합니다.
    if (relocalization_) {
      relocalization_->setMap(map_);
    } else {
      relocalization_ = std::make_unique<Relocalization>(
        map_, relocalization_parameters_);
    }
    if (scoring_) {
      scoring_->setMap(map_);
    } else {
      scoring_ = std::make_unique<scanScoring>(map_, scoring_factor_);
  scoring_->setSharpKernel(scoring_sharp_sigma_, scoring_sharp_weight_);
  scoring_->configureBeamSkip(
    true, beam_skip_prob_, beam_skip_consensus_,
    beam_skip_error_threshold_, beam_skip_particle_stride_);
    }

    // 기하 퇴화 사전계산. 격자점은 free 셀만, raycast는 unknown 통과,
    // 사거리는 실효 반사거리(observability.max_range_m)입니다.
    observability_.setParameters(observability_parameters_);
    const std::size_t observability_rays = static_cast<std::size_t>(std::llround(
      relocalization_parameters_.fov_deg / 360.0 *
      static_cast<double>(relocalization_parameters_.ray_count)));
    observability_.build(
      map_, *relocalization_, relocalization_parameters_.fov_deg,
      observability_rays);
    if (observability_.ready()) {
      const auto &diag = observability_.diagnostics();
      RCLCPP_INFO(
        this->get_logger(),
        "Observability map: %zu points x %d headings, %.1f s, %.1f MB",
        diag.valid_points, observability_parameters_.heading_bins,
        diag.build_seconds, diag.memory_bytes / 1.048576e6);
    } else {
      RCLCPP_WARN(this->get_logger(), "Observability map build failed.");
    }
  } catch (const std::exception &error) {
    RCLCPP_ERROR(this->get_logger(), "Failed to prepare map: %s", error.what());
    return false;
  }

  // 맵이 바뀌면 이전 추정은 더 이상 유효하지 않습니다.
  filter_.reset();
  estimation_.invalidate();
  propagation_->resetPropagationReference();
  map_to_odom_valid_ = false;
  setState(LocalizationState::Lost);
  RCLCPP_INFO(this->get_logger(), "Map ready. Waiting for global localization.");
  return true;
}

namespace {

// map.yaml의 flat key: value를 파싱합니다(yaml-cpp 의존 없이). origin은
// [x, y, yaw] 리스트로 읽습니다. 주석(#)과 따옴표는 제거합니다.
struct MapMeta {
  std::string image;
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
  int negate{0};
  double occupied_thresh{0.65};
  double free_thresh{0.196};
};

std::string trimToken(std::string s) {
  auto notspace = [](int c) { return !std::isspace(c) && c != '"' && c != '\''; };
  while (!s.empty() && !notspace(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
  while (!s.empty() && !notspace(static_cast<unsigned char>(s.back()))) s.pop_back();
  return s;
}

bool parseMapYaml(const std::string &path, MapMeta &out) {
  std::ifstream file(path);
  if (!file) return false;
  std::string line;
  while (std::getline(file, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    const auto colon = line.find(':');
    if (colon == std::string::npos) continue;
    const std::string key = trimToken(line.substr(0, colon));
    std::string value = line.substr(colon + 1);
    if (key == "image") {
      out.image = trimToken(value);
    } else if (key == "resolution") {
      out.resolution = std::atof(trimToken(value).c_str());
    } else if (key == "negate") {
      out.negate = std::atoi(trimToken(value).c_str());
    } else if (key == "occupied_thresh") {
      out.occupied_thresh = std::atof(trimToken(value).c_str());
    } else if (key == "free_thresh") {
      out.free_thresh = std::atof(trimToken(value).c_str());
    } else if (key == "origin") {
      // [x, y, yaw]
      for (char &c : value) if (c == '[' || c == ']' || c == ',') c = ' ';
      std::istringstream ss(value);
      ss >> out.origin_x >> out.origin_y >> out.origin_yaw;
    }
  }
  return !out.image.empty();
}

// PGM 리더. 편집기(GIMP/Paint 등)가 어떤 방식으로 저장해도 읽히도록
// P5(binary)와 P2(ASCII)를 모두 지원하고, 16-bit(maxval>255)는 8-bit로
// 스케일합니다. 실패하면 why에 원인을 채워 호출부가 로그로 남깁니다.
bool loadPgm(const std::string &path, int &width, int &height,
             std::vector<uint8_t> &pixels, std::string &why) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    why = "파일을 열 수 없습니다";
    return false;
  }
  std::string magic;
  file >> magic;
  const bool ascii = (magic == "P2");
  if (magic != "P5" && !ascii) {
    why = "PGM 매직이 '" + magic + "' 입니다. P5(binary) 또는 P2(ASCII)가 "
          "필요합니다 — P6는 컬러(PPM), 그 외는 PNG/BMP를 .pgm 확장자로 "
          "저장했을 가능성이 큽니다. 편집기에서 회색조 PGM(Raw)으로 다시 "
          "저장하거나 `convert in.pgm -depth 8 out.pgm` 로 변환하세요";
    return false;
  }
  auto read_int = [&](int &value) {
    int c = file.get();
    while (true) {
      while (std::isspace(c)) c = file.get();
      if (c == '#') { while (c != '\n' && c != EOF) c = file.get(); }
      else break;
    }
    value = 0;
    while (std::isdigit(c)) { value = value * 10 + (c - '0'); c = file.get(); }
  };
  int maxval = 0;
  read_int(width); read_int(height); read_int(maxval);
  if (width <= 0 || height <= 0 || maxval <= 0 || maxval > 65535) {
    why = "헤더가 올바르지 않습니다 (width=" + std::to_string(width) +
          ", height=" + std::to_string(height) +
          ", maxval=" + std::to_string(maxval) + ")";
    return false;
  }
  const std::size_t count = static_cast<std::size_t>(width) * height;
  const bool wide = maxval > 255;   // 16-bit 샘플
  pixels.resize(count);

  if (ascii) {
    // P2: 공백으로 구분된 십진수 픽셀.
    for (std::size_t i = 0; i < count; ++i) {
      int v = 0;
      if (!(file >> v)) {
        why = "ASCII(P2) 픽셀이 " + std::to_string(i) + "개에서 끊겼습니다 (필요 " +
              std::to_string(count) + "개)";
        return false;
      }
      pixels[i] = static_cast<uint8_t>(
        wide ? std::lround(v * 255.0 / maxval) : std::min(255, std::max(0, v)));
    }
    return true;
  }

  // P5: maxval에 따라 1바이트 또는 2바이트(big-endian) 샘플.
  if (!wide) {
    file.read(reinterpret_cast<char *>(pixels.data()), count);
    if (static_cast<std::size_t>(file.gcount()) != count) {
      why = "픽셀 데이터가 " + std::to_string(file.gcount()) + " 바이트로 부족합니다 (필요 " +
            std::to_string(count) + ")";
      return false;
    }
    return true;
  }
  std::vector<uint8_t> raw(count * 2);
  file.read(reinterpret_cast<char *>(raw.data()), raw.size());
  if (static_cast<std::size_t>(file.gcount()) != raw.size()) {
    why = "16-bit 픽셀 데이터가 부족합니다";
    return false;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const int v = (static_cast<int>(raw[2 * i]) << 8) | raw[2 * i + 1];
    pixels[i] = static_cast<uint8_t>(std::lround(v * 255.0 / maxval));
  }
  return true;
}

}  // namespace

bool mainNode::loadMapFromFile(const std::string &map_dir,
                               const std::string &map_name) {
  const std::string yaml_path = map_dir + "/" + map_name + ".yaml";
  MapMeta meta;
  if (!parseMapYaml(yaml_path, meta)) {
    RCLCPP_ERROR(this->get_logger(), "map loader: cannot read %s", yaml_path.c_str());
    return false;
  }
  // 이미지 경로는 yaml 기준 상대이면 map_dir을 붙입니다.
  std::string image_path = meta.image;
  if (!image_path.empty() && image_path.front() != '/') {
    image_path = map_dir + "/" + image_path;
  }
  int width = 0, height = 0;
  std::vector<uint8_t> pixels;
  std::string why;
  if (!loadPgm(image_path, width, height, pixels, why)) {
    RCLCPP_ERROR(this->get_logger(), "map loader: cannot read PGM %s — %s",
                 image_path.c_str(), why.c_str());
    return false;
  }

  // OccupancyGrid 구성(map_server 규약): 상단 행이 PGM row0이므로 상하 반전.
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = map_frame_;
  grid.header.stamp = this->now();
  grid.info.resolution = meta.resolution;
  grid.info.width = static_cast<uint32_t>(width);
  grid.info.height = static_cast<uint32_t>(height);
  grid.info.origin.position.x = meta.origin_x;
  grid.info.origin.position.y = meta.origin_y;
  grid.info.origin.orientation.z = std::sin(meta.origin_yaw * 0.5);
  grid.info.origin.orientation.w = std::cos(meta.origin_yaw * 0.5);
  grid.data.resize(static_cast<std::size_t>(width) * height);
  for (int my = 0; my < height; ++my) {
    for (int mx = 0; mx < width; ++mx) {
      const uint8_t p = pixels[static_cast<std::size_t>(height - 1 - my) * width + mx];
      const double occ = meta.negate ? (p / 255.0) : ((255 - p) / 255.0);
      int8_t cell;
      if (occ > meta.occupied_thresh) cell = 100;
      else if (occ < meta.free_thresh) cell = 0;
      else cell = -1;
      grid.data[static_cast<std::size_t>(my) * width + mx] = cell;
    }
  }

  RCLCPP_INFO(this->get_logger(),
    "map loader: %s (%dx%d, res %.3f, origin %.2f,%.2f)",
    map_name.c_str(), width, height, meta.resolution, meta.origin_x, meta.origin_y);
  if (!setupMap(grid)) {
    return false;
  }
  // 로드한 맵을 transient_local로 한 번 latch 발행(RViz/상위가 늦게 붙어도 수신).
  if (map_pub_) {
    map_pub_->publish(map_);
  }
  return true;
}

void mainNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // 진단: EKF를 거치지 않은 생 자이로 적분(첫 100샘플 평균을 bias로 제거).
  {
    const double stamp = stampToSeconds(msg->header.stamp);
    const double rate = msg->angular_velocity.z;
    if (raw_gyro_bias_samples_ < 100) {
      raw_gyro_bias_ =
        (raw_gyro_bias_ * raw_gyro_bias_samples_ + rate) / (raw_gyro_bias_samples_ + 1);
      ++raw_gyro_bias_samples_;
    } else if (raw_gyro_last_time_ > 0.0) {
      const double dt = stamp - raw_gyro_last_time_;
      if (dt > 0.0 && dt < 0.5) {
        raw_gyro_yaw_ += (rate - raw_gyro_bias_) * dt;
      }
    }
    raw_gyro_last_time_ = stamp;
  }
  // IMU 샘플마다 경량 dead-reckoning이 돌아 궤적 버퍼를 채웁니다.
  propagation_->imuGetter(*msg, imu_extrinsic_);
}

void mainNode::vesc_state_callback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg) {
  propagation_->wheelGetter(*msg);
}

// RViz "2D Pose Estimate"로 사람이 직접 위치를 지정하는 입구입니다.
// 전역 relocalization 을 대체하지 않습니다 — 자동 탐색은 그대로 살아 있고,
// 이건 "사람이 답을 알고 있을 때 그 답을 넣는" 별도 경로입니다.
void mainNode::initialpose_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  if (!relocalization_) {
    RCLCPP_WARN(
      this->get_logger(),
      "manual seed ignored: map not ready yet");
    return;
  }

  // RViz 는 자신의 Fixed Frame 으로 발행합니다. 맵 프레임이 아니면 좌표가
  // 전혀 다른 뜻이므로 조용히 받아들이면 안 됩니다 — 어디로 찍히는지
  // 알 수 없는 시드가 됩니다.
  std::string frame = msg->header.frame_id;
  if (!frame.empty() && frame.front() == '/') {
    frame.erase(frame.begin());
  }
  if (!frame.empty() && frame != map_frame_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "manual seed ignored: frame '%s' is not the map frame '%s' — "
      "set RViz Fixed Frame to '%s'",
      msg->header.frame_id.c_str(), map_frame_.c_str(), map_frame_.c_str());
    return;
  }

  const double x = msg->pose.pose.position.x;
  const double y = msg->pose.pose.position.y;
  const double yaw = tf2::getYaw(msg->pose.pose.orientation);

  // RViz 도구가 실어 보내는 covariance 를 퍼짐으로 씁니다. 사람 클릭의
  // 불확실성은 검증된 전역 탐색 시드보다 훨씬 크므로 filter.init_*_std
  // (10 cm / 2.9 deg)를 그대로 쓰면 틀린 자리에 못박혀 수렴할 여지가
  // 없습니다. 0 이 실려 오는 설정도 있어 하한을 겁니다.
  const double pos_std = std::max(
    manual_seed_min_pos_std_,
    std::sqrt(std::max(0.0, 0.5 * (msg->pose.covariance[0] + msg->pose.covariance[7]))));
  const double yaw_std = std::max(
    manual_seed_min_yaw_std_,
    std::sqrt(std::max(0.0, msg->pose.covariance[35])));

  // 이미 떠 있는 전역 탐색의 결과는 버립니다. 남겨 두면 (a) 나중에 도착해
  // 사람이 찍은 pose 를 덮어쓰거나, (b) 수동 시드로 Lost 를 빠져나간 뒤
  // 수확되지 않은 채 남았다가 다음 Lost 에서 낡은 결과로 시드됩니다.
  if (reloc_in_flight_) {
    reloc_discard_result_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "manual seed: discarding the in-flight global search result");
  }

  std::vector<ParticleFilter::ModeSeed> seeds;
  seeds.push_back(ParticleFilter::ModeSeed{x, y, yaw, 1.0});
  seedFilter(seeds, pos_std, yaw_std);

  // 파티클을 막 퍼뜨린 직후는 정의상 아직 수렴 전이므로 Converging 으로
  // 들어갑니다. 주행하며 스캔이 가설을 걸러내면 기존 수렴 판정이 Tracking
  // 으로 올리고, 그 사이의 신뢰도는 실제 스캔 정합으로 산정돼 나갑니다.
  const auto previous = state_;
  setState(LocalizationState::Converging);
  manual_seed_stamp_ = this->now().seconds();
  reloc_last_empty_ = false;

  const char *from =
    previous == LocalizationState::Lost ? "Lost" :
    previous == LocalizationState::Converging ? "Converging" :
    previous == LocalizationState::Tracking ? "Tracking" : "WaitingForMap";
  RCLCPP_INFO(
    this->get_logger(),
    "manual seed from RViz: (%.2f, %.2f, %.1f deg) spread %.2f m / %.1f deg "
    "| was %s | global search held for %.1f s",
    x, y, yaw * 180.0 / kPi, pos_std, yaw_std * 180.0 / kPi,
    from, manual_seed_grace_s_);
  announceReloc("MANUAL: RViz 2D Pose Estimate", 0.30f, 0.60f, 0.95f);
}

void mainNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // 레이턴시 계측: 스캔 stamp -> 콜백 진입(전송+큐 지연), 진입 -> 사이클
  // 완료(처리 시간). steady_clock으로 처리 시간을, ROS clock으로 도착
  // 지연을 잰다.
  const auto wall_entry = std::chrono::steady_clock::now();
  const double arrival_delay_ms =
    (this->now() - rclcpp::Time(msg->header.stamp)).seconds() * 1000.0;
  ++scan_count_;
  ensureLaserExtrinsic();

  if (state_ == LocalizationState::WaitingForMap || !relocalization_ || !scoring_) {
    return;
  }

  // Lost 중에도 주행이 이어지므로 히스토리는 상태와 무관하게 쌓습니다.
  updateScanHistory(*msg);
  // 자기일관성은 상태와 무관하게 매 스캔 갱신해야 붕괴 직후 첫 검사가
  // 의미를 갖습니다.
  updateScanConsistency(*msg);

  if (state_ == LocalizationState::Lost) {
    // 워커가 돌고 있으면 결과만 확인하고 즉시 반환합니다. 탐색이 콜백을
    // 막지 않으므로 100 Hz 출력 타이머는 그동안에도 정상 동작합니다.
    if (reloc_in_flight_) {
      if (harvestRelocalization()) {
        // 정지 한 스캔의 최적해는 복도 앨리어스일 수 있으므로 바로 확정하지
        // 않고, 주행으로 가설이 판별될 때까지 Converging에 머뭅니다.
        setState(LocalizationState::Converging);
        announceReloc("RELOC: global search", 0.15f, 0.75f, 0.30f);
        RCLCPP_INFO(this->get_logger(), "Global localization seeded. Converging.");
      }
      return;
    }

    // 전역 탐색은 수백 ms가 걸리므로 매 scan마다 시도하지 않습니다. 다만
    // 직전 시도가 빈손이었다면 짧은 주기로 재시도합니다 — 실패가 반복되는
    // 구간은 대개 입력이 나쁜 구간이라, 입력이 좋아지는 순간을 놓치지 않는
    // 것이 중요합니다(실측 icra: 1초 주기로는 회복이 17초 지연).
    //
    // 그리고 장면이 확실히 바뀌면 주기를 기다리지 않고 바로 시도합니다.
    // 새 관측이 들어왔다는 뜻이므로 이전 실패가 근거가 되지 않습니다.
    const rclcpp::Time now = this->now();
    const double period = reloc_last_empty_ ?
      relocalize_retry_period_s_ : relocalize_period_s_;
    const double scan_change = scanChangeMetric(*msg);
    const bool scene_changed = scan_change > reloc_scan_change_m_;
    if (!scene_changed &&
        (now - last_relocalize_attempt_).seconds() < period) {
      return;
    }
    // 관측이 신뢰할 만해질 때까지 탐색 자체를 멈춘다. 이게 없으면 반쯤
    // 눈먼 스캔으로 13만 후보를 훑어 앨리어스에 시드하고, 게이트는 그
    // 스캔만 보므로 막지 못한다.
    // 수동 시드 유예: 사람이 찍은 자리에서 필터가 수렴할 시간을 줍니다.
    // 유예가 끝나면 평소대로 자동 복구가 돌아오므로, 잘못 찍었더라도
    // 영구히 갇히지 않습니다.
    if (manual_seed_stamp_ >= 0.0) {
      const double since_manual = now.seconds() - manual_seed_stamp_;
      if (since_manual < manual_seed_grace_s_) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "global search held: manual seed grace (%.1f s left)",
          manual_seed_grace_s_ - since_manual);
        return;
      }
      manual_seed_stamp_ = -1.0;
    }
    std::string block_reason;
    if (!searchPreconditionsMet(*msg, block_reason)) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "global search held: %s", block_reason.c_str());
      return;
    }
    last_relocalize_attempt_ = now;
    reloc_attempt_signature_ = scanSignature(*msg);
    // 입력 스냅샷은 메인 스레드에서 만듭니다(propagation_/이력 접근).
    // 탐색 본체만 워커로 보냅니다.
    auto request = std::make_shared<GlobalSearchRequest>(
      buildGlobalSearchRequest(*msg));
    reloc_anchor_valid_ = currentLaserDrPose(
      reloc_anchor_x_, reloc_anchor_y_, reloc_anchor_yaw_);
    reloc_future_ = std::async(
      std::launch::async,
      [this, request]() { return executeGlobalSearch(*request); });
    reloc_in_flight_ = true;
    return;
  }

  runFilterCycle(*msg);
  const double processing_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - wall_entry).count();
  latency_arrival_ms_ += arrival_delay_ms;
  latency_processing_ms_ += processing_ms;
  latency_max_processing_ms_ = std::max(latency_max_processing_ms_, processing_ms);
  last_processing_ms_ = processing_ms;
  ++latency_samples_;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "latency: arrival %.1f ms, processing %.1f ms (avg %.1f, max %.1f over %d)",
    arrival_delay_ms, processing_ms,
    latency_processing_ms_ / std::max(1, latency_samples_),
    latency_max_processing_ms_, latency_samples_);

}

mainNode::~mainNode() {
  waitForRelocalizationWorker();
}

// 워커가 돌고 있으면 끝날 때까지 막습니다. relocalization_ 을 워커와
// 동시에 만지면 안 되는 지점(맵 교체, 노드 종료)에서만 호출합니다.
void mainNode::waitForRelocalizationWorker() {
  if (!reloc_in_flight_) {
    return;
  }
  if (reloc_future_.valid()) {
    reloc_future_.wait();
    try {
      reloc_future_.get();
    } catch (const std::exception &) {
      // 결과를 버리는 경로이므로 예외도 삼킵니다.
    }
  }
  reloc_in_flight_ = false;
}

// 워커 결과를 수확해 필터에 시드합니다. 아직 안 끝났으면 false.
bool mainNode::harvestRelocalization() {
  if (!reloc_in_flight_ || !reloc_future_.valid()) {
    return false;
  }
  if (reloc_future_.wait_for(std::chrono::seconds(0)) !=
      std::future_status::ready) {
    return false;
  }

  if (reloc_discard_result_) {
    // 수동 시드가 이 탐색을 무효화했습니다. future 는 반드시 소비해서
    // 다음 Lost 가 낡은 결과를 수확하지 않게 합니다.
    try {
      reloc_future_.get();
    } catch (const std::exception &) {
      // 버릴 결과이므로 예외도 삼킵니다.
    }
    reloc_in_flight_ = false;
    reloc_discard_result_ = false;
    reloc_last_empty_ = false;
    RCLCPP_INFO(
      this->get_logger(),
      "global search result discarded (superseded by a manual seed)");
    return false;
  }

  GlobalSearchResult result;
  try {
    result = reloc_future_.get();
  } catch (const std::exception &error) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Relocalization worker threw: %s", error.what());
    reloc_in_flight_ = false;
    reloc_last_empty_ = true;
    return false;
  }
  reloc_in_flight_ = false;

  if (!result.error.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Relocalization failed: %s", result.error.c_str());
  }
  if (result.hypotheses.empty()) {
    // best_score(-MSE)를 함께 남긴다. 후보가 아예 없었는지, 있었지만 품질
    // 게이트에 걸렸는지를 구분해야 임계를 판단할 수 있다.
    const auto &diag = relocalization_->lastDiagnostics();
    const double best = diag.best_score;
    // 어느 단계에서 걸렸는지까지 남깁니다. "후보가 없다"와 "후보는 있는데
    // 게이트가 막았다"는 완전히 다른 문제입니다.
    RCLCPP_INFO(
      this->get_logger(),
      "global search: no hypotheses | best %.3f (RMS %.2f m, gate %.2f m) "
      "inlier best %.2f (gate %.2f) | pool %zu: score_floor %zu, dup %zu, "
      "verify %zu(inl %zu/see %zu/cov %zu), ok %zu | scans %zu pts %zu"
      " | top: vis %u/%u occl %u inl %.2f see %.2f sect %u visRMS %.2f fail 0x%02x",
      best, best < 0.0 ? std::sqrt(-best) : 0.0,
      relocalization_parameters_.hypothesis_max_rms_m,
      diag.best_inlier, relocalization_parameters_.hypothesis_verify_fraction,
      diag.pool_size, diag.rejected_score_floor, diag.absorbed_duplicate,
      diag.rejected_verify, diag.rejected_verify_inlier,
      diag.rejected_verify_seethrough, diag.rejected_verify_coverage,
      diag.accepted,
      scan_history_.size() + 1, diag.scan_points,
      static_cast<unsigned>(diag.best_verify.total - diag.best_verify.occluded),
      static_cast<unsigned>(diag.best_verify.total),
      static_cast<unsigned>(diag.best_verify.occluded),
      diag.best_verify.visible_inlier_frac, diag.best_verify.see_through_frac,
      static_cast<unsigned>(diag.best_verify.visible_sectors),
      diag.best_verify.visible_rms,
      static_cast<unsigned>(diag.best_verify.fail_mask));

    // 다음 시도는 짧은 주기로 — 입력이 좋아지는 순간을 놓치지 않기 위해.
    reloc_last_empty_ = true;
    return false;
  }
  reloc_last_empty_ = false;

  if (result.from_trajectory) {
    RCLCPP_INFO(
      this->get_logger(),
      "trajectory fit: %zu hypotheses over %.1f m / %.0f deg, "
      "top(%.2f,%.2f,%.1f) joint=%.4f",
      result.hypotheses.size(), result.trajectory_distance,
      result.trajectory_rotation * 180.0 / kPi,
      result.hypotheses.front().x, result.hypotheses.front().y,
      result.hypotheses.front().yaw * 180.0 / kPi,
      result.hypotheses.front().score);
  }

  // 탐색이 도는 동안 차가 움직인 만큼을 가설에 합성합니다. 가설은 탐색을
  // 시작한 scan 시각의 pose이고, 시드는 지금 적용되기 때문입니다.
  double lead_x = 0.0;
  double lead_y = 0.0;
  double lead_yaw = 0.0;
  double now_x = 0.0;
  double now_y = 0.0;
  double now_yaw = 0.0;
  if (reloc_anchor_valid_ && currentLaserDrPose(now_x, now_y, now_yaw)) {
    const double dx = now_x - reloc_anchor_x_;
    const double dy = now_y - reloc_anchor_y_;
    const double cos_ref = std::cos(-reloc_anchor_yaw_);
    const double sin_ref = std::sin(-reloc_anchor_yaw_);
    lead_x = cos_ref * dx - sin_ref * dy;
    lead_y = sin_ref * dx + cos_ref * dy;
    lead_yaw = normalizeAngle(now_yaw - reloc_anchor_yaw_);
  }
  reloc_anchor_valid_ = false;
  RCLCPP_INFO(
    this->get_logger(),
    "reloc search %.0f ms, lead(%.3f,%.3f,%.1f deg) applied to seeds",
    result.elapsed_ms, lead_x, lead_y, lead_yaw * 180.0 / kPi);

  // ---- 플립 조항 ----
  //
  // prior와 90도 넘게 다른 시드는 비범한 주장이라 비범한 증거를 요구한다.
  // 차단이 아니라 기준 상향만 한다 — 이전 시드가 플립이라 prior가 뒤집혀
  // 있고 새 후보가 교정인 경우, 교정 후보는 건강한 이력 지지를 갖고 있어
  // 상향된 기준을 통과한다. 조항이 교정을 막는 경로는 없다.
  //
  // IMU 자세 이벤트(전복/들림)가 있었던 에피소드는 면제한다 — heading
  // prior가 정당하게 무효이기 때문이다.
  // 밴드와 플립 조항의 적용 조건은 다르다.
  //   밴드: prior를 쓰지 않는다(이력 지지율만 본다). 주행 중 Lost면 적용.
  //   플립: prior heading과 비교하므로, IMU 자세 이벤트가 있으면 heading
  //         prior가 정당하게 무효라 면제한다.
  // 이 둘을 하나로 묶으면 전복 에피소드에서 밴드까지 꺼진다(실측: icra
  // seed2가 지지율 0.00인데도 시드됨).
  const bool band_applicable = majority_band_enabled_ && lost_anchor_valid_;
  double prior_yaw = 0.0;
  bool prior_valid = false;
  if (flip_clause_enabled_ && !lost_imu_event_ && lost_anchor_valid_) {
    double dr_x2 = 0.0;
    double dr_y2 = 0.0;
    double dr_yaw2 = 0.0;
    if (currentLaserDrPose(dr_x2, dr_y2, dr_yaw2)) {
      prior_yaw = normalizeAngle(
        lost_fused_yaw_ + normalizeAngle(dr_yaw2 - lost_dr_yaw_));
      prior_valid = true;
    }
  }

  std::vector<ParticleFilter::ModeSeed> seeds;
  seeds.reserve(result.hypotheses.size());
  for (std::size_t index = 0; index < result.hypotheses.size(); ++index) {
    const auto &h = result.hypotheses[index];
    bool flip_triggered = false;
    if (prior_valid) {
      const double heading_delta =
        std::abs(normalizeAngle(h.yaw + lead_yaw - prior_yaw)) * 180.0 / kPi;
      flip_triggered = heading_delta > flip_clause_deg_;
      if (flip_triggered) {
        const bool thin_evidence =
          h.support_weight < relocalization_parameters_.history_min_support_weight;
        // 증거가 얇으면 다수결을 요구할 수 없으므로 latest를 더 엄격히 본다.
        const bool ok = thin_evidence
          ? (h.verify.see_through_frac <= flip_max_see_through_)
          : (h.support_ratio >= flip_majority_fraction_);
        if (!ok) {
          RCLCPP_INFO(
            this->get_logger(),
            "flip clause: hypo[%zu] heading %.0f deg from prior, "
            "support %.2f (w %.2f) see %.2f — rejected",
            index, heading_delta, h.support_ratio, h.support_weight,
            h.verify.see_through_frac);
          continue;
        }
        RCLCPP_INFO(
          this->get_logger(),
          "flip clause: hypo[%zu] heading %.0f deg from prior — passed "
          "(support %.2f w %.2f, see %.2f)",
          index, heading_delta, h.support_ratio, h.support_weight,
          h.verify.see_through_frac);
      }
    }
    // ---- 밴드 판정 ----
    //
    // prior가 없으면(스타트업/리셋) 비교 대상이 없으므로 기존 경로 그대로
    // 전부 시드하고 Converging이 판별한다. prior가 있는 주행 중 Lost에서만
    // 밴드가 작동한다.
    double seed_weight = 1.0;
    if (band_applicable) {
      const bool thin =
        h.support_weight < relocalization_parameters_.history_min_support_weight;
      const double accept = flip_triggered ? flip_majority_fraction_
                                           : majority_accept_;
      const double probation = flip_triggered ? 0.30 : majority_probation_;
      if (thin) {
        // 증거가 얇으면 다수결을 요구할 수 없다 — 보호관찰로 심어 주행에
        // 넘긴다(thin-evidence 경쟁 시딩).
        seed_weight = probation_mass_;
      } else if (h.support_ratio >= accept) {
        seed_weight = 1.0;
      } else if (h.support_ratio >= probation) {
        seed_weight = probation_mass_;
        RCLCPP_INFO(
          this->get_logger(),
          "majority band: hypo[%zu] support %.2f (w %.2f) — probation",
          index, h.support_ratio, h.support_weight);
      } else {
        RCLCPP_INFO(
          this->get_logger(),
          "majority band: hypo[%zu] support %.2f (w %.2f) — rejected",
          index, h.support_ratio, h.support_weight);
        continue;
      }
    }
    const double cos_h = std::cos(h.yaw);
    const double sin_h = std::sin(h.yaw);
    seeds.push_back(ParticleFilter::ModeSeed{
      h.x + cos_h * lead_x - sin_h * lead_y,
      h.y + sin_h * lead_x + cos_h * lead_y,
      normalizeAngle(h.yaw + lead_yaw),
      seed_weight});
    RCLCPP_INFO(
      this->get_logger(),
      "reloc hypo[%zu](%.2f,%.2f,%.1f) score=%.4f aggRMS=%.2f | vis %u/%u "
      "occl %u inl %.2f see %.2f sect %u visRMS %.2f fail 0x%02x "
      "| support %.2f (w %.2f)%s",
      index, h.x, h.y, h.yaw * 180.0 / kPi, h.score,
      h.score < 0.0 ? std::sqrt(-h.score) : 0.0,
      static_cast<unsigned>(h.verify.total - h.verify.occluded),
      static_cast<unsigned>(h.verify.total),
      static_cast<unsigned>(h.verify.occluded),
      h.verify.visible_inlier_frac, h.verify.see_through_frac,
      static_cast<unsigned>(h.verify.visible_sectors), h.verify.visible_rms,
      static_cast<unsigned>(h.verify.fail_mask),
      h.support_ratio, h.support_weight,
      index == 0 ? " <- dominant seed" : "");
    if (h.view_count > 0) {
      std::string views;
      for (std::uint8_t v = 0; v < h.view_count; ++v) {
        views += (v ? " " : "");
        views += std::to_string(
          static_cast<int>(std::lround(h.view_rms[v] * 100.0)));
        views += "/";
        views += std::to_string(
          static_cast<int>(std::lround(h.view_visible[v] * 100.0)));
        // 그 뷰를 찍을 때의 차대 기울기. 뷰별 RMS 가 나쁠 때 그것이 pose
        // 오차인지 스캔 평면이 틀어진 탓인지 구분하려면 함께 봐야 합니다.
        if (v < reloc_view_tilt_.size()) {
          views += "/";
          views += std::to_string(
            static_cast<int>(std::lround(reloc_view_tilt_[v][0])));
          views += ",";
          views += std::to_string(
            static_cast<int>(std::lround(reloc_view_tilt_[v][1])));
        }
      }
      RCLCPP_INFO(
        this->get_logger(),
        "  hypo[%zu] views RMScm/vis%%/roll,pitch deg: [%s]",
        index, views.c_str());
    }
  }
  {
    // 신·구 집계 점수와 이력별 점수를 함께 남긴다. 같은 후보에 대한 두 점수가
    // 나란히 찍히므로 섀도 비교와 행1 포렌식(버려진 이력의 점수 분포)이
    // 추가 런 없이 나온다.
    const auto &diag = relocalization_->lastDiagnostics();
    std::string views;
    for (std::uint8_t i = 0; i < diag.top_view_count; ++i) {
      views += (i ? " " : "");
      // "RMS/가시분율" 쌍으로 남긴다 — 가림이 원인인지 판정하려면 둘이 필요.
      const double rms = diag.top_view_scores[i] < 0.0
        ? std::sqrt(-diag.top_view_scores[i]) : 0.0;
      views += std::to_string(static_cast<int>(std::lround(rms * 100.0)));
      views += "/";
      views += std::to_string(
        static_cast<int>(std::lround(diag.top_view_visible[i] * 100.0)));
    }
    RCLCPP_INFO(
      this->get_logger(),
      "reloc %zu hypotheses, pts=%zu | aggregate old %.4f new %.4f"
      " | views RMScm/vis%% [%s]",
      result.hypotheses.size(), result.scan_points,
      diag.top_score_old, diag.top_score_new, views.c_str());
  }

  if (seeds.empty() && !result.hypotheses.empty()) {
    // 전원 거부. Lost 중에는 이력이 동결되므로 다음 시도도 같은 뷰로 같은
    // 판정을 내려 영원히 거부할 수 있다(데드락). 일정 시간을 넘기면 최선
    // 후보를 보호관찰로 승격한다 — 반복을 '정답의 증거'로 쓰는 게 아니라
    // '게이트 교착의 증거'로 써서 주행 판별에 넘기는 것이다.
    const double lost_elapsed = lost_anchor_valid_
      ? this->now().seconds() - lost_stamp_ : 0.0;
    if (lost_elapsed > majority_escape_s_) {
      const auto &h = result.hypotheses.front();
      const double cos_h = std::cos(h.yaw);
      const double sin_h = std::sin(h.yaw);
      seeds.push_back(ParticleFilter::ModeSeed{
        h.x + cos_h * lead_x - sin_h * lead_y,
        h.y + sin_h * lead_x + cos_h * lead_y,
        normalizeAngle(h.yaw + lead_yaw), probation_mass_});
      RCLCPP_WARN(
        this->get_logger(),
        "majority escape: all rejected for %.1f s — seeding best on probation "
        "(support %.2f)", lost_elapsed, h.support_ratio);
    }
  }
  if (seeds.empty()) {
    // 빈 시드로 진행하면 estimation_.reset이 seeds.front()를 건드려 UB다.
    // 후보 없음과 같게 취급하고 재시도한다.
    RCLCPP_INFO(
      this->get_logger(),
      "all hypotheses rejected — treating as no hypotheses");
    reloc_last_empty_ = true;
    return false;
  }
  seedFilter(seeds);
  return true;
}

void mainNode::seedFilter(
  const std::vector<ParticleFilter::ModeSeed> &seeds,
  double position_std, double yaw_std) {
  // 정전 구간을 통과한 DR 오차: 예측(마지막 정상 pose + 그동안의 DR) 대비
  // 실제 시드 위치. 로컬 복구 반경을 이 실측으로 정한다.
  if (lost_anchor_valid_ && !seeds.empty()) {
    double dr_x = 0.0;
    double dr_y = 0.0;
    double dr_yaw = 0.0;
    if (currentLaserDrPose(dr_x, dr_y, dr_yaw)) {
      const double delta_x = dr_x - lost_dr_x_;
      const double delta_y = dr_y - lost_dr_y_;
      const double cos_ref = std::cos(-lost_dr_yaw_);
      const double sin_ref = std::sin(-lost_dr_yaw_);
      const double body_x = cos_ref * delta_x - sin_ref * delta_y;
      const double body_y = sin_ref * delta_x + cos_ref * delta_y;
      const double body_yaw = normalizeAngle(dr_yaw - lost_dr_yaw_);
      const double cos_f = std::cos(lost_fused_yaw_);
      const double sin_f = std::sin(lost_fused_yaw_);
      const double predicted_x = lost_fused_x_ + cos_f * body_x - sin_f * body_y;
      const double predicted_y = lost_fused_y_ + sin_f * body_x + cos_f * body_y;
      const double predicted_yaw = normalizeAngle(lost_fused_yaw_ + body_yaw);
      RCLCPP_INFO(
        this->get_logger(),
        "outage DR error: predicted(%.2f,%.2f,%.1f) vs seed(%.2f,%.2f,%.1f) "
        "= %.2f m / %.1f deg over %.1f s (DR travel %.2f m)",
        predicted_x, predicted_y, predicted_yaw * 180.0 / kPi,
        seeds.front().x, seeds.front().y, seeds.front().yaw * 180.0 / kPi,
        std::hypot(predicted_x - seeds.front().x, predicted_y - seeds.front().y),
        std::abs(normalizeAngle(predicted_yaw - seeds.front().yaw)) * 180.0 / kPi,
        this->now().seconds() - lost_stamp_, std::hypot(body_x, body_y));
    }
    lost_anchor_valid_ = false;
  }
  if (seeds.size() == 1 && position_std > 0.0 && yaw_std > 0.0) {
    // 수동 시드 경로: 호출자가 지정한 퍼짐으로 뿌립니다.
    filter_.initializeAround(
      seeds.front().x, seeds.front().y, seeds.front().yaw,
      position_std, yaw_std);
  } else {
    filter_.initializeMultiple(seeds);
  }
  // 파티클을 다시 뿌렸으므로 오래된 기준 pose와의 큰 차이를 적용하지 않도록
  // propagation 기준을 반드시 초기화합니다.
  propagation_->resetPropagationReference();
  // EKF 동역학 상태(속도·바이어스·서스펜션 자세)도 되돌립니다. Lost 중 차를
  // 들고 움직인 경우 그 모순이 바이어스로 흡수돼 있어, pose만 고치면 시드
  // 직후부터 일정한 드리프트가 남습니다(스캔이 계속 잡아끄는 증상).
  propagation_->resetDynamicState();
  // 융합 기준은 첫 번째(최고 점수) 가설에서 시작합니다.
  estimation_.reset(seeds.front().x, seeds.front().y, seeds.front().yaw);
  accumulated_translation_ = 0.0;
  accumulated_rotation_ = 0.0;
  converge_accumulated_ = 0.0;
  converge_rotation_accumulated_ = 0.0;
  converge_raw_translation_ = 0.0;
  converge_raw_rotation_ = 0.0;
  map_inconsistency_ema_ = 0.0;
  beamskip_bad_frames_.clear();
  beamskip_bad_count_ = 0;
  // 중심 창도 함께 비운다. 시드는 pose를 불연속으로 옮기므로, 남겨 두면
  // 재시드 전후 샘플이 한 창에 섞여 점프가 물체 운동으로 계산된다.
  // 가설이 1개면 시드 직후 alive==1이라 감시가 곧바로 무장되어 무장 해제
  // 경로로는 비워지지 않는다(icra 실측: 에고 26.5 m/s 같은 허수).
  beamskip_centroids_.clear();
  scan_particles_.clear();
  // 융합 기준을 seeds[0]으로 잡았으므로 지배 모드 추적도 0에서 시작합니다.
  last_dominant_mode_ = 0;
}

void mainNode::updateScanHistory(const sensor_msgs::msg::LaserScan &scan) {
  if (!propagation_) {
    return;
  }
  propagation_->advanceTo(scan.header.stamp);
  const auto sample = propagation_->poseAt(scan.header.stamp);
  if (!sample.valid) {
    return;
  }
  // 스캔 정합은 라이다 기준이므로 lever arm을 더한 라이다 pose로 저장합니다.
  const double cos_yaw = std::cos(sample.yaw);
  const double sin_yaw = std::sin(sample.yaw);
  ScanSnapshot snapshot;
  snapshot.x = sample.x +
    cos_yaw * laser_extrinsic_.rear_to_laser_x -
    sin_yaw * laser_extrinsic_.rear_to_laser_y;
  snapshot.y = sample.y +
    sin_yaw * laser_extrinsic_.rear_to_laser_x +
    cos_yaw * laser_extrinsic_.rear_to_laser_y;
  snapshot.yaw = sample.yaw;
  // 자세도 함께 남깁니다. 아래 tilt 게이트가 '너무 기운' 스냅샷을 버리긴
  // 하지만, 통과한 스냅샷들도 서로 기울기가 다릅니다 — 그 차이가 뷰별
  // RMS 에 섞여 들어오므로 진단에서 분리해 볼 수 있어야 합니다.
  propagation_->relativeTilt(snapshot.roll_deg, snapshot.pitch_deg);

  // 궤적 정합용 pose 이력은 multi-scan과 독립적으로 쌓습니다.
  if (pose_history_.empty() ||
      std::hypot(snapshot.x - pose_history_.back()[0],
                 snapshot.y - pose_history_.back()[1]) >= 0.3) {
    pose_history_.push_back({snapshot.x, snapshot.y, snapshot.yaw});
    const std::size_t keep =
      static_cast<std::size_t>(trajectory_fit_history_m_ / 0.3);
    while (pose_history_.size() > keep) {
      pose_history_.pop_front();
    }
  }

  // 오염된 스냅샷은 애초에 넣지 않습니다.
  //
  // multi-scan 공동 채점은 '모든' 스캔이 맞아야 통과하므로, 한 장만 오염돼도
  // 모든 후보가 떨어집니다. 그런데 이력은 0.5 m/15도마다 한 장씩만 갱신되어
  // 최대 5.5 m를 달려야 씻겨나갑니다. 실측 icra: 50초경 전복 후 리로컬이
  // 17초 동안 계속 실패하다가 이력이 교체되고 나서야 98 ms 만에 성공했습니다
  // (탐색이 느린 게 아니라 입력이 오염돼 있었습니다).
  //
  // (1) 전복/들림: 라이다 평면이 기울어 바닥·천장을 봅니다. 같은 구간에서
  //     DR도 끊기므로 상대이동 사슬까지 함께 깨집니다.
  if (propagation_->relativeTiltDeg() > ekf_parameters_.dr_tilt_max_deg) {
    return;
  }
  // (2) 유의미한 점을 못 얻은 스캔: 전역 탐색이 어차피 쓰지 못합니다.
  std::size_t valid_points = 0;
  for (const float range : scan.ranges) {
    if (std::isfinite(range) && range >= scan.range_min && range <= scan.range_max) {
      ++valid_points;
    }
  }
  if (valid_points < relocalization_parameters_.minimum_scan_points) {
    return;
  }

  if (!scan_history_.empty()) {
    const ScanSnapshot &last = scan_history_.back();
    const double distance = std::hypot(snapshot.x - last.x, snapshot.y - last.y);
    const double rotation = std::abs(normalizeAngle(snapshot.yaw - last.yaw));
    if (distance < multi_scan_spacing_m_ &&
        rotation < multi_scan_spacing_deg_ * kPi / 180.0) {
      return;
    }
  }
  snapshot.scan = scan;
  scan_history_.push_back(std::move(snapshot));
  // 저장은 사용 장수와 무관하게 조밀 버퍼를 유지한다. 탐색 요청을 만들 때
  // 이 중 기하적으로 퍼진 부분집합을 고른다(selectHistoryViews).
  const std::size_t keep = static_cast<std::size_t>(std::max(
    std::max(1, multi_scan_count_ - 1), multi_scan_buffer_));
  while (scan_history_.size() > keep) {
    scan_history_.pop_front();
  }
}

// EKF 최신 시각의 라이다 pose(파티클과 같은 프레임)를 돌려줍니다.
bool mainNode::currentLaserDrPose(double &x, double &y, double &yaw) const {
  if (!propagation_) {
    return false;
  }
  const auto sample = propagation_->poseAt(propagation_->newestTrajectoryTime());
  if (!sample.valid) {
    return false;
  }
  const double cos_yaw = std::cos(sample.yaw);
  const double sin_yaw = std::sin(sample.yaw);
  x = sample.x +
    cos_yaw * laser_extrinsic_.rear_to_laser_x -
    sin_yaw * laser_extrinsic_.rear_to_laser_y;
  y = sample.y +
    sin_yaw * laser_extrinsic_.rear_to_laser_x +
    cos_yaw * laser_extrinsic_.rear_to_laser_y;
  yaw = sample.yaw;
  return true;
}

bool mainNode::searchPreconditionsMet(
  const sensor_msgs::msg::LaserScan &scan, std::string &reason) {
  // (1) 유효점 비율 — 센서 수준 이상(틸트로 하늘/바닥을 봄, max-range 폭증).
  //     가림은 짧은 리턴이라 유효점을 줄이지 않으므로 정상 가림 씬과 충돌하지
  //     않는다.
  std::size_t valid = 0;
  for (const float range : scan.ranges) {
    if (std::isfinite(range) && range >= scan.range_min && range <= scan.range_max) {
      ++valid;
    }
  }
  const double ratio = scan.ranges.empty() ? 0.0 :
    static_cast<double>(valid) / static_cast<double>(scan.ranges.size());
  if (ratio < search_min_valid_ratio_) {
    reason = "valid ratio " + std::to_string(ratio);
    search_consistency_ok_count_ = 0;
    prev_scan_valid_ = false;
    return false;
  }

  // (2) 자세 — 기울어진 스캔 평면은 2D 정합의 전제를 깬다.
  if (propagation_ && propagation_->relativeTiltDeg() > search_max_tilt_deg_) {
    // 이 에피소드에 자세 이벤트가 있었음을 기록한다 — heading prior가
    // 정당하게 무효가 되므로 플립 조항을 면제해야 한다.
    lost_imu_event_ = true;
    reason = "tilt " + std::to_string(propagation_->relativeTiltDeg());
    search_consistency_ok_count_ = 0;
    prev_scan_valid_ = false;
    return false;
  }

  // (3) 스캔-투-스캔 자기일관성은 매 스캔 갱신되는 스트릭으로 판정한다.
  //     (updateScanConsistency 참조 — Lost 시도 때만 갱신하면 붕괴 직후 첫
  //     검사가 아주 오래된 스캔과 비교되어 무의미한 값이 나온다.)
  if (search_consistency_ok_count_ < search_consistency_frames_) {
    reason = "consistency streak " + std::to_string(search_consistency_ok_count_);
    return false;
  }
  return true;
}

// 매 스캔 호출. 직전 스캔을 DR SE(2)(회전+병진)로 현재 프레임에 옮겨 빔별
// 사거리를 비교하고 연속 일치 프레임 수를 센다. 회전만 보정하면 주행
// 속도에서 병진이 오탐을 낸다. 구르는 동안은 3D 회전을 2D 보정이 못 따라가
// 자연히 무너지는데, 그게 의도된 차단이다.
void mainNode::updateScanConsistency(const sensor_msgs::msg::LaserScan &scan) {
  double dr_x = 0.0;
  double dr_y = 0.0;
  double dr_yaw = 0.0;
  if (!currentLaserDrPose(dr_x, dr_y, dr_yaw)) {
    search_consistency_ok_count_ = 0;
    prev_scan_valid_ = false;
    return;
  }
  bool consistent = false;
  if (prev_scan_valid_ && prev_scan_.ranges.size() == scan.ranges.size() &&
      scan.angle_increment > 0.0) {
    const double delta_x = dr_x - prev_scan_dr_x_;
    const double delta_y = dr_y - prev_scan_dr_y_;
    const double cos_ref = std::cos(-prev_scan_dr_yaw_);
    const double sin_ref = std::sin(-prev_scan_dr_yaw_);
    // 직전 라이다 프레임 기준 이동량.
    const double move_x = cos_ref * delta_x - sin_ref * delta_y;
    const double move_y = sin_ref * delta_x + cos_ref * delta_y;
    const double move_yaw = normalizeAngle(dr_yaw - prev_scan_dr_yaw_);
    std::size_t compared = 0;
    std::size_t agree = 0;
    for (std::size_t i = 0; i < prev_scan_.ranges.size(); ++i) {
      const double r = static_cast<double>(prev_scan_.ranges[i]);
      if (!std::isfinite(r) || r < prev_scan_.range_min || r > prev_scan_.range_max) {
        continue;
      }
      // 직전 스캔의 점을 현재 라이다 프레임으로 옮긴다.
      const double angle = static_cast<double>(prev_scan_.angle_min) +
        static_cast<double>(i) * static_cast<double>(prev_scan_.angle_increment);
      const double px = r * std::cos(angle) - move_x;
      const double py = r * std::sin(angle) - move_y;
      const double cos_m = std::cos(-move_yaw);
      const double sin_m = std::sin(-move_yaw);
      const double qx = cos_m * px - sin_m * py;
      const double qy = sin_m * px + cos_m * py;
      const double predicted_range = std::hypot(qx, qy);
      const double predicted_angle = std::atan2(qy, qx);
      const int index = static_cast<int>(std::llround(
        (predicted_angle - static_cast<double>(scan.angle_min)) /
        static_cast<double>(scan.angle_increment)));
      if (index < 0 || index >= static_cast<int>(scan.ranges.size())) {
        continue;
      }
      const double observed = static_cast<double>(scan.ranges[index]);
      if (!std::isfinite(observed) || observed < scan.range_min ||
          observed > scan.range_max) {
        continue;
      }
      ++compared;
      if (std::abs(observed - predicted_range) < search_consistency_inlier_m_) {
        ++agree;
      }
    }
    if (compared > 0) {
      const double fraction =
        static_cast<double>(agree) / static_cast<double>(compared);
      consistent = fraction >= search_consistency_fraction_;
      last_consistency_fraction_ = fraction;
    }
  }
  prev_scan_ = scan;
  prev_scan_valid_ = true;
  prev_scan_dr_x_ = dr_x;
  prev_scan_dr_y_ = dr_y;
  prev_scan_dr_yaw_ = dr_yaw;

  if (consistent) {
    ++search_consistency_ok_count_;
  } else {
    search_consistency_ok_count_ = 0;
  }
}

double mainNode::scanSignature(const sensor_msgs::msg::LaserScan &scan) {
  double sum = 0.0;
  std::size_t count = 0;
  for (const float range : scan.ranges) {
    if (std::isfinite(range) && range >= scan.range_min && range <= scan.range_max) {
      sum += static_cast<double>(range);
      ++count;
    }
  }
  return count > 0 ? sum / static_cast<double>(count) : 0.0;
}

double mainNode::scanChangeMetric(const sensor_msgs::msg::LaserScan &scan) const {
  if (reloc_attempt_signature_ <= 0.0) {
    return 0.0;
  }
  return std::abs(scanSignature(scan) - reloc_attempt_signature_);
}

// [메인 스레드] 탐색에 필요한 입력을 전부 값으로 복사해 둡니다. 여기서만
// propagation_/scan_history_/pose_history_ 같은 노드 상태를 만집니다.
mainNode::GlobalSearchRequest mainNode::buildGlobalSearchRequest(
  const sensor_msgs::msg::LaserScan &scan) {
  GlobalSearchRequest request;

  // 현재 라이다 DR pose. 궤적 정합과 multi-scan 둘 다 필요합니다.
  double laser_x = 0.0;
  double laser_y = 0.0;
  double laser_yaw = 0.0;
  bool pose_ok = false;
  if (propagation_) {
    propagation_->advanceTo(scan.header.stamp);
    const auto sample = propagation_->poseAt(scan.header.stamp);
    if (sample.valid) {
      const double cos_yaw = std::cos(sample.yaw);
      const double sin_yaw = std::sin(sample.yaw);
      laser_x = sample.x +
        cos_yaw * laser_extrinsic_.rear_to_laser_x -
        sin_yaw * laser_extrinsic_.rear_to_laser_y;
      laser_y = sample.y +
        sin_yaw * laser_extrinsic_.rear_to_laser_x +
        cos_yaw * laser_extrinsic_.rear_to_laser_y;
      laser_yaw = sample.yaw;
      pose_ok = true;
    }
  }
  request.laser_yaw = laser_yaw;

  // 스캔 이력 + 현재 스캔을 relocalizeMultiple/Trajectory 공용 체인으로
  // 접습니다. 연속 라이다 pose a->b의 상대 이동(a 프레임 기준)입니다.
  //
  // 이력은 조밀 버퍼(0.1 m 간격, 최대 multi_scan_buffer장)에서 최대
  // multi_scan_count-1장을 '기하적으로 퍼지게' 골라 씁니다. 판별력은 장수가
  // 아니라 시점 다양성(베이스라인)에서 나옵니다 — 직선 1 m 위의 11장은 서로
  // 상관된 표본이라 한 표나 다름없지만, 코너를 낀 3장은 앨리어스를 강하게
  // 자릅니다. 조밀 저장 덕에 서행에서도 뷰가 빨리 차고(증거 굶주림으로 인한
  // majority escape 소멸), 이동이 쌓이면 선택이 자동으로 넓은 스팬을 복원해
  // 같은 비용(<=11장 채점)으로 양쪽을 다 얻습니다.
  if (pose_ok) {
    // 후보: 현재 pose에서 노후 컷(history_valid_dr_max_m) 이내의 뷰만.
    // 검색 측이 어차피 그 밖의 뷰를 버리므로 슬롯을 낭비하지 않습니다.
    std::vector<std::size_t> eligible;
    eligible.reserve(scan_history_.size());
    const double dr_max = relocalization_parameters_.history_valid_dr_max_m;
    for (std::size_t i = 0; i < scan_history_.size(); ++i) {
      const ScanSnapshot &snapshot = scan_history_[i];
      if (dr_max > 0.0 &&
          std::hypot(snapshot.x - laser_x, snapshot.y - laser_y) > dr_max) {
        continue;
      }
      eligible.push_back(i);
    }

    // greedy farthest-point 선택(SE(2)). 거리 = 평면거리 + 1.8 * |dyaw| —
    // 1.8 m/rad 는 가설 분리 NMS(0.8 m / 25도)와 같은 환산입니다. 현재
    // pose를 고정 앵커로 두고 시작하므로 '지금'과 겹치는 뷰는 밀려납니다.
    const std::size_t want = static_cast<std::size_t>(
      std::max(1, multi_scan_count_ - 1));
    std::vector<std::size_t> selected;
    if (eligible.size() <= want) {
      selected = eligible;
    } else {
      constexpr double kYawWeight = 1.8;
      auto se2_dist = [&](std::size_t a, double bx, double by, double byaw) {
        const ScanSnapshot &va = scan_history_[a];
        return std::hypot(va.x - bx, va.y - by) +
          kYawWeight * std::abs(normalizeAngle(va.yaw - byaw));
      };
      std::vector<double> min_dist(eligible.size());
      for (std::size_t e = 0; e < eligible.size(); ++e) {
        min_dist[e] = se2_dist(eligible[e], laser_x, laser_y, laser_yaw);
      }
      std::vector<bool> taken(eligible.size(), false);
      selected.reserve(want);
      for (std::size_t round = 0; round < want; ++round) {
        std::size_t best = eligible.size();
        double best_dist = -1.0;
        for (std::size_t e = 0; e < eligible.size(); ++e) {
          if (!taken[e] && min_dist[e] > best_dist) {
            best_dist = min_dist[e];
            best = e;
          }
        }
        if (best == eligible.size()) {
          break;
        }
        taken[best] = true;
        selected.push_back(eligible[best]);
        const ScanSnapshot &vb = scan_history_[eligible[best]];
        for (std::size_t e = 0; e < eligible.size(); ++e) {
          if (!taken[e]) {
            min_dist[e] = std::min(
              min_dist[e], se2_dist(eligible[e], vb.x, vb.y, vb.yaw));
          }
        }
      }
      // 체인은 시간 순서를 전제하므로 정렬해 되돌립니다.
      std::sort(selected.begin(), selected.end());
    }

    // 선택 진단: 몇 장을 어떤 스팬으로 쓰는지. 적은 장수로 통과하는지,
    // 정보가 모일 때까지 기다리는지가 여기서 읽힙니다.
    {
      double span = 0.0;
      double yaw_spread = 0.0;
      for (std::size_t a = 0; a < selected.size(); ++a) {
        for (std::size_t b = a + 1; b < selected.size(); ++b) {
          const ScanSnapshot &va = scan_history_[selected[a]];
          const ScanSnapshot &vb = scan_history_[selected[b]];
          span = std::max(span, std::hypot(va.x - vb.x, va.y - vb.y));
          yaw_spread = std::max(yaw_spread,
            std::abs(normalizeAngle(va.yaw - vb.yaw)));
        }
      }
      RCLCPP_INFO(
        this->get_logger(),
        "history views: %zu of %zu buffered (%zu eligible), span %.1f m / "
        "%.0f deg",
        selected.size(), scan_history_.size(), eligible.size(),
        span, yaw_spread * 180.0 / kPi);
    }

    std::vector<std::array<double, 3>> chain;
    chain.reserve(selected.size() + 1);
    reloc_view_tilt_.clear();
    reloc_view_tilt_.reserve(selected.size() + 1);
    for (const std::size_t index : selected) {
      const ScanSnapshot &snapshot = scan_history_[index];
      request.scans.push_back(snapshot.scan);
      chain.push_back({snapshot.x, snapshot.y, snapshot.yaw});
      reloc_view_tilt_.push_back({snapshot.roll_deg, snapshot.pitch_deg});
    }
    request.scans.push_back(scan);
    chain.push_back({laser_x, laser_y, laser_yaw});
    {
      // 마지막 뷰는 현재 스캔이므로 지금 자세를 씁니다.
      double roll_now = 0.0;
      double pitch_now = 0.0;
      propagation_->relativeTilt(roll_now, pitch_now);
      reloc_view_tilt_.push_back({roll_now, pitch_now});
    }
    request.motions.reserve(chain.size() - 1);
    for (std::size_t index = 0; index + 1 < chain.size(); ++index) {
      const double delta_x = chain[index + 1][0] - chain[index][0];
      const double delta_y = chain[index + 1][1] - chain[index][1];
      const double cos_ref = std::cos(-chain[index][2]);
      const double sin_ref = std::sin(-chain[index][2]);
      Relocalization::RelativeMotion motion;
      motion.dx = cos_ref * delta_x - sin_ref * delta_y;
      motion.dy = sin_ref * delta_x + cos_ref * delta_y;
      motion.dyaw = normalizeAngle(chain[index + 1][2] - chain[index][2]);
      request.motions.push_back(motion);
    }
  } else {
    request.scans.push_back(scan);
  }

  // 코너를 포함한 충분한 궤적이 모였으면 궤적-모양 정합이 최우선입니다.
  // 스캔 채점은 낡은 맵/자기유사 트랙에서 앨리어스에 속지만, 주행로
  // 배치는 그대로라 궤적 정합은 진짜 배치를 찾습니다. 루프 위상은
  // multi-scan 공동 채점이 가립니다.
  if (pose_ok && pose_history_.size() >= 8) {
    double distance = 0.0;
    double rotation = 0.0;
    for (std::size_t i = 1; i < pose_history_.size(); ++i) {
      distance += std::hypot(
        pose_history_[i][0] - pose_history_[i - 1][0],
        pose_history_[i][1] - pose_history_[i - 1][1]);
      rotation += std::abs(normalizeAngle(
        pose_history_[i][2] - pose_history_[i - 1][2]));
    }
    if (distance >= trajectory_fit_min_distance_m_ &&
        rotation >= trajectory_fit_min_rotation_deg_ * kPi / 180.0) {
      request.trajectory_points.reserve(pose_history_.size() + 1);
      for (const auto &entry : pose_history_) {
        request.trajectory_points.push_back({entry[0], entry[1]});
      }
      request.trajectory_points.push_back({laser_x, laser_y});
      request.trajectory_distance = distance;
      request.trajectory_rotation = rotation;
    }
  }
  return request;
}

// [워커 스레드] 순수 계산부. 노드 상태를 만지지 않고 relocalization_ 만
// 사용합니다. 동시에 한 건만 도는 것이 호출부에서 보장되고,
// relocalization_ 을 바꾸는 지점(맵 교체/종료)은 먼저 워커를 기다립니다.
// 로거/시계는 여기서 건드리지 않고 결과 구조체로 실어 보냅니다.
mainNode::GlobalSearchResult mainNode::executeGlobalSearch(
  const GlobalSearchRequest &request) {
  GlobalSearchResult result;
  if (!relocalization_ || request.scans.empty()) {
    return result;
  }
  const auto worker_entry = std::chrono::steady_clock::now();
  struct Timer {
    std::chrono::steady_clock::time_point start;
    double *out;
    ~Timer() {
      *out = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    }
  } timer{worker_entry, &result.elapsed_ms};

  if (!request.trajectory_points.empty()) {
    try {
      auto hypotheses = relocalization_->relocalizeTrajectory(
        request.trajectory_points, request.laser_yaw,
        request.scans, request.motions);
      if (!hypotheses.empty()) {
        result.hypotheses = std::move(hypotheses);
        result.from_trajectory = true;
        result.trajectory_distance = request.trajectory_distance;
        result.trajectory_rotation = request.trajectory_rotation;
        result.scan_points = relocalization_->lastDiagnostics().scan_points;
        return result;
      }
    } catch (const std::exception &error) {
      result.error = std::string("trajectory fit: ") + error.what();
    }
  }

  try {
    result.hypotheses = request.scans.size() > 1
      ? relocalization_->relocalizeMultiple(request.scans, request.motions)
      : relocalization_->relocalizeMultiple(request.scans.front());
    result.scan_points = relocalization_->lastDiagnostics().scan_points;
  } catch (const std::exception &error) {
    result.error = error.what();
    result.hypotheses.clear();
  }
  return result;
}

void mainNode::updateScanParticles(double best_main_score, particle *particles,
                                   int32_t particle_count) {
  if (scan_particle_count_ <= 0 || particle_count <= 0) {
    return;
  }
  // 기준점: 이전 프레임의 최종(융합) pose를 이번 사이클 이동량으로 전파.
  const auto &previous = estimation_.last();
  if (!previous.valid) {
    return;
  }
  double base_x = previous.x;
  double base_y = previous.y;
  double base_yaw = previous.yaw;
  const auto motion = propagation_->lastMotionDelta();
  if (motion.valid) {
    const double cos_t = std::cos(base_yaw);
    const double sin_t = std::sin(base_yaw);
    base_x += cos_t * motion.longitudinal - sin_t * motion.lateral;
    base_y += sin_t * motion.longitudinal + cos_t * motion.lateral;
    base_yaw = normalizeAngle(base_yaw + motion.yaw);
  }

  // 고정 범위 균일 샘플로 매 프레임 새로 뿌립니다(무상태 — 진화 없음).
  const std::size_t pool_size = static_cast<std::size_t>(scan_particle_count_);
  scan_particles_.resize(pool_size);
  std::uniform_real_distribution<double> u_pos(
    -scan_particle_spread_pos_, scan_particle_spread_pos_);
  std::uniform_real_distribution<double> u_yaw(
    -scan_particle_spread_yaw_, scan_particle_spread_yaw_);
  for (particle &sp : scan_particles_) {
    sp = particle{};
    sp.x = static_cast<float>(base_x + u_pos(scan_rng_));
    sp.y = static_cast<float>(base_y + u_pos(scan_rng_));
    sp.theta = static_cast<float>(normalizeAngle(base_yaw + u_yaw(scan_rng_)));
  }
  for (particle &sp : scan_particles_) {
    scoring_->scorePrepared(sp);
  }

  // 최고점이 본 필터 best보다 margin 이상 좋으면 본 필터 최하위 교체.
  std::size_t best_index = 0;
  for (std::size_t i = 1; i < pool_size; ++i) {
    if (scan_particles_[i].score[0] > scan_particles_[best_index].score[0]) {
      best_index = i;
    }
  }
  const double pool_best =
    static_cast<double>(scan_particles_[best_index].score[0]);
  if (scan_particle_inject_count_ > 0 &&
      pool_best > best_main_score + scan_particle_inject_margin_) {
    std::vector<std::size_t> order(pool_size);
    for (std::size_t i = 0; i < pool_size; ++i) {
      order[i] = i;
    }
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
      return scan_particles_[a].score[0] > scan_particles_[b].score[0];
    });
    std::vector<int32_t> worst(particle_count);
    for (int32_t i = 0; i < particle_count; ++i) {
      worst[i] = i;
    }
    const int32_t inject = std::min<int32_t>(
      scan_particle_inject_count_, particle_count / 10);
    std::partial_sort(
      worst.begin(), worst.begin() + inject, worst.end(),
      [&](int32_t a, int32_t b) {
        return particles[a].score[0] < particles[b].score[0];
      });
    for (int32_t k = 0; k < inject; ++k) {
      particle &target = particles[worst[k]];
      const particle &src =
        scan_particles_[order[static_cast<std::size_t>(k) %
                              std::max<std::size_t>(1, pool_size / 4)]];
      const uint16_t keep_mode = target.mode;
      target = src;
      target.mode = keep_mode;
    }
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "scan-pool inject: pool best %.2f > main best %.2f (+%.2f), %d particles.",
      pool_best, best_main_score, scan_particle_inject_margin_, inject);
  }

  if (publish_particles_ && scan_particles_pub_->get_subscription_count() > 0) {
    geometry_msgs::msg::PoseArray message;
    message.header.stamp = this->now();
    message.header.frame_id = map_frame_;
    message.poses.reserve(scan_particles_.size());
    for (const particle &sp : scan_particles_) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = sp.x;
      pose.position.y = sp.y;
      pose.orientation = yawToQuaternion(static_cast<double>(sp.theta));
      message.poses.push_back(pose);
    }
    scan_particles_pub_->publish(message);
  }
}

bool mainNode::isFreeCell(double x, double y) const {
  if (map_.data.empty()) {
    return true;
  }
  const double resolution = map_.info.resolution;
  if (!(resolution > 0.0)) {
    return true;
  }
  const int cell_x = static_cast<int>(std::floor(
    (x - map_.info.origin.position.x) / resolution));
  const int cell_y = static_cast<int>(std::floor(
    (y - map_.info.origin.position.y) / resolution));
  if (cell_x < 0 || cell_y < 0 ||
      cell_x >= static_cast<int>(map_.info.width) ||
      cell_y >= static_cast<int>(map_.info.height)) {
    return false;
  }
  const int8_t value = map_.data[
    static_cast<std::size_t>(cell_y) * map_.info.width +
    static_cast<std::size_t>(cell_x)];
  return value >= 0 &&
    value < relocalization_parameters_.occupied_threshold;
}

void mainNode::setState(LocalizationState next) {
  if (state_ == next) {
    return;
  }
  // 이력은 비우지 않습니다.
  //
  // 예전에는 "전제가 틀렸으니 그 위에 쌓인 이력도 버린다"로 폐기했는데,
  // 실측(icra)에서 부작용이 훨씬 컸습니다. 이력은 0.5 m/15도 이동이 있어야
  // 쌓이는데 Lost 직후는 차가 정지·서행하는 구간이라 재구축이 가장 느립니다
  // — 판별력이 가장 필요한 순간과 정확히 겹칩니다. 실제로 붕괴 0.07초 뒤
  // 유효점 266/541짜리 단일 스캔으로 전역 탐색이 돌아 17 m 떨어진 앨리어스에
  // 시드했고(모든 게이트 통과), 이력이 2장 모이는 데 7초가 걸렸습니다.
  //
  // 오염 대응은 폐기가 아니라 (a) 삽입 시점 배제(전복/저품질 스캔은 애초에
  // 안 넣음)와 (b) 탐색 시점 배제(DR 체인이 끊긴 뷰는 그 탐색에서 무효)로
  // 범위를 좁혔습니다.
  if (next == LocalizationState::Lost) {
    // 정전 구간 DR 오차 계측용 앵커.
    const auto &fused = estimation_.last();
    double dr_x = 0.0;
    double dr_y = 0.0;
    double dr_yaw = 0.0;
    lost_imu_event_ = false;
    lost_anchor_valid_ = fused.valid && currentLaserDrPose(dr_x, dr_y, dr_yaw);
    if (lost_anchor_valid_) {
      lost_fused_x_ = fused.x;
      lost_fused_y_ = fused.y;
      lost_fused_yaw_ = fused.yaw;
      lost_dr_x_ = dr_x;
      lost_dr_y_ = dr_y;
      lost_dr_yaw_ = dr_yaw;
      lost_stamp_ = this->now().seconds();
    }
  }
  state_ = next;
  publishState();
}





void mainNode::announceReloc(const std::string &reason, float r, float g, float b) {
  if (!reloc_reason_pub_) {
    return;
  }
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = this->now();
  marker.header.frame_id = map_frame_;
  marker.ns = "reloc_reason";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  // 맵 중앙 상공에 띄웁니다.
  marker.pose.position.x = map_.info.origin.position.x +
    0.5 * map_.info.width * map_.info.resolution;
  marker.pose.position.y = map_.info.origin.position.y +
    0.5 * map_.info.height * map_.info.resolution;
  marker.pose.position.z = 1.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.9;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.95f;
  marker.lifetime = rclcpp::Duration::from_seconds(3.0);
  marker.text = reason;
  reloc_reason_pub_->publish(marker);

  // 누적 이력 보드: 최근 8건을 맵 상단에 상시 표시합니다.
  char line[96];
  std::snprintf(line, sizeof(line), "[%7.1fs] %s",
                this->now().seconds() - node_start_seconds_,
                reason.c_str());
  reloc_history_.push_back(line);
  while (reloc_history_.size() > 8) {
    reloc_history_.pop_front();
  }
  if (reloc_history_pub_) {
    visualization_msgs::msg::Marker board;
    board.header.stamp = this->now();
    board.header.frame_id = map_frame_;
    board.ns = "reloc_history";
    board.id = 0;
    board.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    board.action = visualization_msgs::msg::Marker::ADD;
    board.pose.position.x = map_.info.origin.position.x +
      0.5 * map_.info.width * map_.info.resolution;
    board.pose.position.y = map_.info.origin.position.y +
      map_.info.height * map_.info.resolution + 1.0;
    board.pose.position.z = 0.5;
    board.pose.orientation.w = 1.0;
    board.scale.z = 0.45;
    board.color.r = 0.95f;
    board.color.g = 0.95f;
    board.color.b = 0.95f;
    board.color.a = 0.9f;
    std::string text;
    for (const auto &entry : reloc_history_) {
      text += entry;
      text += "\n";
    }
    board.text = text;
    reloc_history_pub_->publish(board);
  }

  // 파일 기록: 세션 전체 누적(도구/사후 분석용).
  std::ofstream file(
    "/home/shin/Desktop/SLAM_new/docs/eval/reloc_events.log", std::ios::app);
  if (file) {
    file << std::fixed << std::setprecision(1)
         << (this->now().seconds() - node_start_seconds_) << "s  "
         << reason << "\n";
  }
}

void mainNode::publishState() {
  if (!state_pub_) {
    return;
  }
  std_msgs::msg::UInt8 message;
  switch (state_) {
    case LocalizationState::Converging: message.data = 1; break;
    case LocalizationState::Tracking:   message.data = 2; break;
    default:                            message.data = 0; break;
  }
  state_pub_->publish(message);
}

// 준비된 스캔으로 전 파티클을 채점합니다(청크 병렬).
void mainNode::scoreAllParticles(particle *particles, int32_t particle_count) {
  const unsigned scoring_workers = scoring_threads_ > 0 ?
    static_cast<unsigned>(scoring_threads_) :
    std::min(4u, std::max(1u, std::thread::hardware_concurrency()));
  if (scoring_workers <= 1 || particle_count < 256) {
    for (int32_t index = 0; index < particle_count; ++index) {
      scoring_->scorePrepared(particles[index]);
    }
    return;
  }
  std::vector<std::thread> workers;
  workers.reserve(scoring_workers);
  const int32_t chunk =
    (particle_count + static_cast<int32_t>(scoring_workers) - 1) /
    static_cast<int32_t>(scoring_workers);
  for (unsigned w = 0; w < scoring_workers; ++w) {
    const int32_t begin = static_cast<int32_t>(w) * chunk;
    const int32_t end = std::min(particle_count, begin + chunk);
    if (begin >= end) {
      break;
    }
    workers.emplace_back([this, particles, begin, end]() {
      for (int32_t index = begin; index < end; ++index) {
        scoring_->scorePrepared(particles[index]);
      }
    });
  }
  for (auto &worker : workers) {
    worker.join();
  }
}

// 다중 가설 판별 가속.
//
// 가중치는 무기억이다 - normalizeWeights()가 직전 가중치에 곱하지 않고 현재
// 스캔의 score[0]만으로 새로 계산한다. 따라서 같은 스캔을 여러 번 채점해도
// 결과가 완전히 동일하고 정보가 늘지 않는다. 그래서 이 버스트는 "관측 반복"이
// 아니라 "국소 최적화"다:
//
//   지터(점점 축소) -> 채점 -> 정규화 -> 리샘플
//
// 각 모드의 파티클이 자기 주변에서 최선의 정합 위치를 찾아 들어간다. 진짜
// 모드는 실제 pose로 빨려들어가 점수가 크게 오르지만, 가짜 모드는 어디로
// 움직여도 맵과 안 맞으므로 점수가 안 오른다. 그 대비가 모드 질량 차이로
// 나타나 주행 없이도 판별이 진행된다.
//
// 리샘플은 이동량 게이트를 우회한다(정지 중 다양성 손실을 막는 게이트인데,
// 여기서는 지터가 다양성을 계속 다시 넣어주므로 안전하다).
bool mainNode::skipLooksDynamic(double &speed_out, double &ego_out) const {
  speed_out = 0.0;
  ego_out = 0.0;
  // 유효 샘플 두 개 이상이어야 속도를 낼 수 있다.
  std::vector<const SkipCentroidSample *> samples;
  samples.reserve(beamskip_centroids_.size());
  for (const auto &sample : beamskip_centroids_) {
    if (sample.valid) {
      samples.push_back(&sample);
    }
  }
  // 표본이 적거나 시간 baseline이 짧으면 속도 추정이 통째로 잡음이다.
  // 스킵 빔이 있는 프레임만 표본이 되므로 창이 차 있어도 표본은 2개일 수
  // 있고, 그때 dt는 한 스캔(25 ms)이라 20 cm 보정이 8 m/s로 환산된다.
  // 억제는 감지기를 끄는 방향이라 근거가 얇으면 하지 않는다.
  if (static_cast<int>(samples.size()) < dynamic_skip_min_samples_) {
    return false;
  }
  // 창 전체의 순 이동을 경과 시간으로 나눈다. 프레임별 차분의 평균보다
  // 잡음에 둔감하고, 한 방향으로 꾸준히 움직이는 물체를 잘 잡는다.
  const SkipCentroidSample &first = *samples.front();
  const SkipCentroidSample &last = *samples.back();
  const double dt = last.stamp - first.stamp;
  if (!(dt >= dynamic_skip_min_dt_)) {
    return false;
  }
  speed_out = std::hypot(last.x - first.x, last.y - first.y) / dt;
  ego_out = std::hypot(last.ego_x - first.ego_x, last.ego_y - first.ego_y) / dt;
  // 창이 pose 불연속(재시드 등)을 가로지르면 두 속도가 함께 폭발한다.
  // 억제는 감지기를 끄는 방향이라 의심스러우면 하지 않는 쪽이 안전하다.
  if (!std::isfinite(speed_out) || !std::isfinite(ego_out) ||
      ego_out > ekf_parameters_.max_wheel_speed) {
    return false;
  }
  // 상대차가 같은 속도로 앞서가면 중심 속도 == 에고 속도다. 에고 속도에
  // 비례한 기준을 쓰면 대회 속도가 변해도 그대로 성립한다. 정지 중에는
  // 분모가 0이 되므로 절대 하한을 함께 둔다.
  const double threshold =
    std::max(dynamic_skip_speed_mps_, dynamic_skip_ego_ratio_ * ego_out);
  return speed_out > threshold;
}

void mainNode::runFilterCycle(const sensor_msgs::msg::LaserScan &scan) {
  if (!filter_.initialized()) {
    setState(LocalizationState::Lost);
    return;
  }

  // ---- L2 writer: 풀 EKF를 스캔 시각까지 진행 ----
  propagation_->advanceTo(scan.header.stamp);

  // ---- deskew: 빔마다 자기 시각의 라이다 pose로 재투영 ----
  // poseAt은 뒤차축 pose이므로 lever arm을 더해 라이다 pose로 바꿔 넘깁니다.
  const double lever_x = laser_extrinsic_.rear_to_laser_x;
  const double lever_y = laser_extrinsic_.rear_to_laser_y;
  auto pose_lookup = [this, lever_x, lever_y](
    double time_seconds, double &x, double &y, double &yaw) -> bool {
      const auto sample = propagation_->poseAt(time_seconds);
      if (!sample.valid) {
        return false;
      }
      const double cos_yaw = std::cos(sample.yaw);
      const double sin_yaw = std::sin(sample.yaw);
      x = sample.x + cos_yaw * lever_x - sin_yaw * lever_y;
      y = sample.y + sin_yaw * lever_x + cos_yaw * lever_y;
      yaw = sample.yaw;
      return true;
    };

  ScanDeskew::Result deskewed;
  sensor_msgs::msg::LaserScan corrected = scan;
  double reference_time = stampToSeconds(scan.header.stamp);
  if (deskew_.deskew(scan, pose_lookup, deskewed) && deskewed.valid) {
    reference_time = deskewed.reference_time;
    // 기존 LaserScan 소비자를 위해 균일 각도 격자로 되담습니다(근사).
    if (!ScanDeskew::toLaserScan(deskewed, scan, corrected)) {
      corrected = scan;
    }
  }
  const builtin_interfaces::msg::Time reference_stamp = secondsToStamp(reference_time);

  // ---- 예측: 파티클을 deskew 기준 시각으로 전파 ----
  int32_t particle_count = filter_.count();
  propagation_->propagation(filter_.data(), particle_count, reference_stamp);
  const auto motion = propagation_->lastMotionDelta();
  if (motion.valid) {
    accumulated_translation_ += std::hypot(motion.longitudinal, motion.lateral);
    accumulated_rotation_ += std::abs(motion.yaw);
  }

  // ---- 측정: 파티클별 스캔 우도 ----
  // 스캔당 1회 빔 테이블을 만들고(파티클x빔 trig/log 제거) 파티클을
  // 청크로 나눠 병렬 채점합니다. scorePrepared는 읽기 전용이라 안전합니다.
  scoring_->prepareScan(corrected, scoring_beam_stride_);
  particle *particles = filter_.data();

  // ---- beam skipping: 파티클 합의로 동적/미지도 빔 제외 ----
  // 합의 붕괴(제안 스킵 비율 > error_threshold) 프레임이 창 N 중 M개를
  // 넘으면 "빔 다수가 어떤 가설로도 설명 안 됨" = 위치 상실로 판단하고
  // relocalization을 트리거합니다. score 평균은 이 상태에 둔감해서
  // 기존 score 감시가 못 잡던 경로입니다.
  const auto beam_skip = scoring_->computeBeamSkip(particles, particle_count);
  // 출력 타이머가 pose와 같은 stamp로 함께 내보낼 수 있도록 캐시합니다.
  last_skip_fraction_ = beam_skip.proposed_fraction;
  if (beam_skip.applied && skipped_scan_pub_ &&
      skipped_scan_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::LaserScan skipped;
    scoring_->buildSkippedScan(corrected, skipped);
    skipped_scan_pub_->publish(skipped);
  }
  {
    const bool bad = beam_skip.proposed_fraction > beamskip_lost_fraction_;
    beamskip_bad_frames_.push_back(bad);
    if (bad) {
      ++beamskip_bad_count_;
    }
    while (static_cast<int>(beamskip_bad_frames_.size()) > beamskip_lost_window_) {
      if (beamskip_bad_frames_.front()) {
        --beamskip_bad_count_;
      }
      beamskip_bad_frames_.pop_front();
    }

    // 스킵 중심을 맵 프레임으로 옮겨 같은 창 길이로 보관한다.
    //
    // 좌표 기준은 dead reckoning이 아니라 직전 융합 pose(스캔 보정 포함)다.
    // 이 환경은 휠 슬립이 심해 DR로는 정지 물체도 움직이는 것처럼 보인다.
    // 각 샘플이 자기 시점의 보정된 pose를 쓰므로 창 안에 DR 누적이 없다.
    // (감시가 무장된 구간은 아직 미아 확정 전이라 PF pose가 유효하다.)
    SkipCentroidSample sample;
    const auto &fused_pose = estimation_.last();
    if (beam_skip.skip_beams > 0 && fused_pose.valid) {
      const double cos_yaw = std::cos(fused_pose.yaw);
      const double sin_yaw = std::sin(fused_pose.yaw);
      sample.x =
        fused_pose.x + cos_yaw * beam_skip.centroid_x - sin_yaw * beam_skip.centroid_y;
      sample.y =
        fused_pose.y + sin_yaw * beam_skip.centroid_x + cos_yaw * beam_skip.centroid_y;
      sample.ego_x = fused_pose.x;
      sample.ego_y = fused_pose.y;
      sample.stamp = rclcpp::Time(scan.header.stamp).seconds();
      sample.valid = true;
    }
    beamskip_centroids_.push_back(sample);
    while (static_cast<int>(beamskip_centroids_.size()) > beamskip_lost_window_) {
      beamskip_centroids_.pop_front();
    }
    // 다중 가설이 아직 판별 중(Converging & alive>1)에는 발동하지
    // 않습니다 — 뿌린 클러스터가 하나로 뭉치거나 없어질 때까지 다음
    // 리로컬을 막는 사용자 설계. 혼합 pose로 잰 프레임이 재무장 순간
    // 즉발하지 않도록 창도 함께 비웁니다.
    const bool beam_watch_armed =
      state_ == LocalizationState::Tracking ||
      (state_ == LocalizationState::Converging && last_alive_modes_ <= 1);
    if (!beam_watch_armed) {
      beamskip_bad_frames_.clear();
      beamskip_bad_count_ = 0;
      beamskip_centroids_.clear();
    }
    // 창을 채운 미설명 빔이 '움직이는 물체'였다면 미아 증거가 아니다.
    // 같은 속도로 앞서가는 상대차가 정확히 이 경우다.
    double skip_speed = 0.0;
    double ego_speed = 0.0;
    if (beam_watch_armed && dynamic_skip_reject_ &&
        beamskip_bad_count_ >= beamskip_lost_count_ &&
        skipLooksDynamic(skip_speed, ego_speed)) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "beam-skip collapse suppressed: centroid %.2f m/s vs ego %.2f m/s "
        "(%d/%d frames, skip %.2f) — moving object.",
        skip_speed, ego_speed, beamskip_bad_count_, beamskip_lost_window_,
        beam_skip.proposed_fraction);
      beamskip_bad_frames_.clear();
      beamskip_bad_count_ = 0;
    }
    if (beam_watch_armed &&
        beamskip_bad_count_ >= beamskip_lost_count_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Beam-skip consensus collapse: %d/%d frames unexplained "
        "(skip fraction %.2f, threshold %.2f). Lost.",
        beamskip_bad_count_, beamskip_lost_window_,
        beam_skip.proposed_fraction, beamskip_lost_fraction_);
      announceReloc("RELOC: beam consensus", 0.95f, 0.78f, 0.10f);
      setState(LocalizationState::Lost);
      beamskip_bad_frames_.clear();
      beamskip_bad_count_ = 0;
    }
  }
  scoreAllParticles(particles, particle_count);

  // ---- 스캔 탐색 풀 갱신 + 우위 시 주입 ----
  {
    double best_main = -1.0e9;
    for (int32_t index = 0; index < particle_count; ++index) {
      best_main = std::max(
        best_main, static_cast<double>(particles[index].score[0]));
    }
    updateScanParticles(best_main, particles, particle_count);
  }

  // ---- 가중 정규화 + 리샘플링 ----
  filter_.normalizeWeights();
  // 정지 중 리샘플링은 정보 없이 다양성만 깎으므로 이동량으로 한 번 더 거릅니다.
  if (accumulated_translation_ >= resample_min_translation_ ||
      accumulated_rotation_ >= resample_min_rotation_) {
    if (filter_.resampleIfNeeded()) {
      accumulated_translation_ = 0.0;
      accumulated_rotation_ = 0.0;
    }
  }

  // ---- 구름 통계 ----
  // 다봉 상태에서 전체 가중 평균은 모드 사이(벽 안)를 가리킬 수 있으므로
  // pose는 항상 지배 모드의 통계로 냅니다(단일 모드면 전체와 동일).
  const auto mode_summary = filter_.modeSummary();
  // 빔 합의 무장 판정용(다음 사이클에서 사용 — 빔 스킵은 채점 전에 돎).
  last_alive_modes_ = mode_summary.alive_modes;
  const auto cloud = filter_.estimateOfMode(mode_summary.dominant);

  // 지배 모드가 바뀌면 융합 기준을 새 모드로 리베이스합니다. 그대로 두면
  // 융합이 멀리 떨어진 두 모드 사이를 블렌딩한 헛 포즈(벽 안)를 냅니다.
  if (cloud.valid &&
      static_cast<int>(mode_summary.dominant) != last_dominant_mode_) {
    last_dominant_mode_ = static_cast<int>(mode_summary.dominant);
    estimation_.reset(cloud.x, cloud.y, cloud.yaw);
  }

  // ---- 관측성 조회 (교체 지점) ----
  // 라이브 스캔이 아니라 맵 사전계산에서 가져오므로 동적 장애물에 오염되지
  // 않습니다. 더 나은 관측성 지표가 정해지면 이 한 줄만 바꾸면 됩니다.
  PoseEstimation::Observability observability;
  const auto &previous = estimation_.last();
  if (previous.valid) {
    const auto sample = observability_.query(previous.x, previous.y, previous.yaw);
    if (sample.valid) {
      // sigma[m] -> confidence[0,1]. sigma = reference에서 0.5가 되는 포화
      // 사상이라 절대 단위가 유지되고 맵 의존 튜닝이 없습니다.
      const double reference_square =
        observability_reference_sigma_ * observability_reference_sigma_;
      Eigen::Vector2d confidence;
      for (int axis = 0; axis < 2; ++axis) {
        const double sigma = sample.sigma(axis);
        confidence(axis) = reference_square / (reference_square + sigma * sigma);
      }
      observability.valid = true;
      // eigenvectors 열 0 = 강축, 열 1 = 약축(퇴화 방향). confidence 내림차순.
      observability.eigenvectors = sample.eigenvectors;
      observability.confidence = confidence;
      // 다음 사이클 파티클 노이즈도 같은 관측성으로 방향별 확산시킵니다.
      propagation_->setNoiseShaping(sample.eigenvectors, confidence);
    } else {
      // 조회 실패 = "기하를 모름"이지 "스캔이 틀림"이 아닙니다. 예전처럼
      // 스캔을 완전히 끄면(conf 0) dead-reckoning만 남아, 미끄러져 나간
      // 추정을 스캔이 다시 끌어올 수단이 사라지고 표류가 고착됩니다.
      // 등방 중립 신뢰로 물러서서 복구 경로를 열어 둡니다.
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "observability query failed at (%.2f, %.2f); using neutral fallback %.2f.",
        previous.x, previous.y, observability_fallback_confidence_);
      const Eigen::Vector2d fallback(
        observability_fallback_confidence_, observability_fallback_confidence_);
      observability.valid = true;
      observability.eigenvectors = Eigen::Matrix2d::Identity();
      observability.confidence = fallback;
      propagation_->setNoiseShaping(Eigen::Matrix2d::Identity(), fallback);
    }
  }

  // ---- 이방성 융합 ----
  PoseEstimation::ScanEstimate scan_estimate;
  scan_estimate.valid = cloud.valid;
  scan_estimate.x = cloud.x;
  scan_estimate.y = cloud.y;
  scan_estimate.yaw = cloud.yaw;
  scan_estimate.position_covariance = cloud.position_covariance;
  scan_estimate.yaw_variance = cloud.yaw_variance;

  PoseEstimation::OdomDelta odom_delta;
  odom_delta.valid = motion.valid;
  odom_delta.longitudinal = motion.longitudinal;
  odom_delta.lateral = motion.lateral;
  odom_delta.yaw = motion.yaw;

  const auto fused = estimation_.update(scan_estimate, odom_delta, observability);
  if (!fused.valid) {
    return;
  }
  PoseEstimation::annotateParticles(filter_.data(), particle_count, fused);

  // ---- map->odom 갱신 ----
  const Pose2D map_to_base = laserToBase(Pose2D{fused.x, fused.y, fused.yaw});
  const auto odom_sample = propagation_->poseAt(reference_stamp);
  if (odom_sample.valid) {
    map_to_odom_ = computeMapToOdom(
      map_to_base, Pose2D{odom_sample.x, odom_sample.y, odom_sample.yaw});
    map_to_odom_valid_ = true;
  }

  // ---- 순간 슬립 시각화 ----
  // EKF의 횡가속-잔차 기반 슬립 지표[0,1]를 로봇 위 텍스트 마커로 띄웁니다.
  // 색은 초록(0) -> 빨강(1). 수치는 /localization_pf/slip으로도 나갑니다.
  {
    const double slip = propagation_->lastSlipScore();
    std_msgs::msg::Float32 value;
    value.data = static_cast<float>(slip);
    slip_pub_->publish(value);
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->now();
    marker.header.frame_id = map_frame_;
    marker.ns = "slip";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = fused.x;
    marker.pose.position.y = fused.y;
    marker.pose.position.z = 0.6;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.45;
    marker.color.r = static_cast<float>(slip);
    marker.color.g = static_cast<float>(1.0 - slip);
    marker.color.b = 0.1f;
    marker.color.a = 0.95f;
    char text[32];
    std::snprintf(text, sizeof(text), "slip %.2f", slip);
    marker.text = text;
    slip_marker_pub_->publish(marker);
  }

  // ---- 진단 ----
  ++cycle_count_;
  const double now_seconds = stampToSeconds(reference_stamp);
  const double cycle_dt = last_cycle_time_ > 0.0 ? now_seconds - last_cycle_time_ : 0.0;
  last_cycle_time_ = now_seconds;
  double best_score = -1.0e9;
  double mean_score = 0.0;
  int32_t best_index = 0;
  for (int32_t index = 0; index < particle_count; ++index) {
    const double value = static_cast<double>(particles[index].score[0]);
    if (value > best_score) {
      best_score = value;
      best_index = index;
    }
    mean_score += value;
  }
  mean_score /= std::max(1, particle_count);
  // 출력 타이머가 pose와 같은 stamp로 함께 내보낼 수 있도록 캐시합니다.

  // ---- 정합 건강도 -> 다음 사이클 예측 노이즈 ----
  // 두 신호의 min을 씁니다:
  //  (1) best score 선형 사상(느리지만 전체 붕괴를 잡음)
  //  (2) best 파티클의 아웃라이어 빔 '비율'(소수의 큰 불일치에도 즉각 반응
  //      — 평균 로그우도는 여기에 둔감하다는 사용자 관찰 반영).
  {
    const double span = score_good_ - score_fail_threshold_;
    const double score_health = span > 1.0e-9 ?
      std::clamp((best_score - score_fail_threshold_) / span, 0.0, 1.0) : 1.0;
    const double fraction =
      scoring_->outlierFraction(particles[best_index], outlier_prob_);
    const double frac_span = outlier_frac_bad_ - outlier_frac_good_;
    const double beam_health = frac_span > 1.0e-9 ?
      1.0 - std::clamp((fraction - outlier_frac_good_) / frac_span, 0.0, 1.0) : 1.0;
    propagation_->setNoiseHealth(std::min(score_health, beam_health));
    // 신뢰도 토픽으로 내보낼 정규화 지표를 캐시합니다.
    last_score_health_ = score_health;
    last_outlier_fraction_ = fraction;
  }

  // ---- 다중 가설 수렴 판정 ----
  // 정지 중에는 앨리어스를 구분할 수 없으므로(0526-1 초기화 실패의 교훈)
  // 지배 질량 조건을 유지한 채 실제로 주행한 양으로만 수렴을 셉니다.
  //
  // 다만 "얼마나 움직였나"가 아니라 "얼마나 알아냈나"로 세야 합니다. 복도를
  // 따라 직진하면 아무리 가도 앨리어스가 안 풀리지만, 특징이 풍부한 곳에서는
  // 조금만 움직여도 판별이 끝납니다. 그래서 이동량을 그 방향의 기하 관측성
  // 으로 가중해 누적합니다:
  //
  //     c(v)      = (v.e0)^2 c0 + (v.e1)^2 c1      // 방향별 신뢰도
  //     progress += |dp| * c(dp_hat)
  //     rot      += |dpsi| * (c0 + c1) / 2
  //
  // e1(약축)이 곧 퇴화 방향이므로, 그쪽으로 움직이면 진행량이 거의 안 쌓여
  // 자동으로 더 기다리게 됩니다. 관측성을 모르면 가중 1.0(기존 동작).
  //
  // 맵 전체가 퇴화면 영원히 확정 못 하므로 생(raw) 이동량에 상한을 둡니다.
  if (state_ == LocalizationState::Converging) {
    if (mode_summary.dominant_mass >= converge_mass_) {
      if (motion.valid) {
        const double translation =
          std::hypot(motion.longitudinal, motion.lateral);
        const double rotation = std::abs(motion.yaw);
        converge_raw_translation_ += translation;
        converge_raw_rotation_ += rotation;

        double translation_gain = 1.0;
        double rotation_gain = 1.0;
        if (observability.valid && converge_use_observability_) {
          const double c0 = observability.confidence(0);
          const double c1 = observability.confidence(1);
          rotation_gain = 0.5 * (c0 + c1);
          if (translation > 1.0e-9 && previous.valid) {
            // motion은 직전 pose의 body 프레임이므로 map 프레임으로 돌립니다.
            const double cos_yaw = std::cos(previous.yaw);
            const double sin_yaw = std::sin(previous.yaw);
            const Eigen::Vector2d direction{
              (cos_yaw * motion.longitudinal - sin_yaw * motion.lateral) /
                translation,
              (sin_yaw * motion.longitudinal + cos_yaw * motion.lateral) /
                translation};
            const double along0 = direction.dot(observability.eigenvectors.col(0));
            const double along1 = direction.dot(observability.eigenvectors.col(1));
            translation_gain = along0 * along0 * c0 + along1 * along1 * c1;
          } else {
            translation_gain = rotation_gain;
          }
        }
        converge_accumulated_ += translation * translation_gain;
        converge_rotation_accumulated_ += rotation * rotation_gain;
      }
      // 직선 주행만으로는 복도 앨리어스가 판별되지 않으므로(스래싱 교훈)
      // 거리와 함께 코너 통과에 해당하는 누적 회전을 요구합니다.
      const bool informed =
        converge_accumulated_ >= converge_distance_m_ &&
        converge_rotation_accumulated_ >= converge_rotation_deg_ * kPi / 180.0;
      // 퇴화 구간에서 무한 대기하지 않도록 하는 안전 상한입니다.
      const bool capped =
        converge_raw_translation_ >=
          converge_distance_m_ * converge_raw_cap_factor_ &&
        converge_raw_rotation_ >=
          converge_rotation_deg_ * kPi / 180.0 * converge_raw_cap_factor_;
      if (informed || capped) {
        setState(LocalizationState::Tracking);
        RCLCPP_INFO(
          this->get_logger(),
          "Hypotheses converged after %.2f m / %.0f deg informed "
          "(raw %.2f m / %.0f deg%s): mode %u mass %.2f (%d alive). Tracking.",
          converge_accumulated_,
          converge_rotation_accumulated_ * 180.0 / kPi,
          converge_raw_translation_,
          converge_raw_rotation_ * 180.0 / kPi,
          (!informed && capped) ? ", CAP" : "",
          mode_summary.dominant, mode_summary.dominant_mass,
          mode_summary.alive_modes);
      }
    } else {
      converge_accumulated_ = 0.0;
      converge_rotation_accumulated_ = 0.0;
      converge_raw_translation_ = 0.0;
      converge_raw_rotation_ = 0.0;
    }
  }

  // ---- 맵 정합성 감시 ----
  // 로봇은 unknown/벽 위를 달리지 않으므로(운영 전제) fused가 free 아닌
  // 셀에 머무는 비율이 높으면 score가 좋아 보여도 배치가 틀린 것입니다.
  // 닮은꼴 루프 전역 앨리어스는 score 감시로는 안 걸리고 여기서 걸립니다.
  // 다중 가설 판별 중(Converging & alive>1)에는 무장하지 않습니다 —
  // 클러스터가 하나로 뭉치거나 없어질 때까지 다음 리로컬 금지(사용자
  // 설계). 판별 중 EMA도 0으로 유지해 재무장 순간 즉발을 막습니다.
  const bool offmap_watch_armed =
    state_ == LocalizationState::Tracking ||
    (state_ == LocalizationState::Converging &&
     mode_summary.alive_modes <= 1);
  if (offmap_watch_armed) {
    const Pose2D base = laserToBase(Pose2D{fused.x, fused.y, fused.yaw});
    const double sample_dt = std::clamp(cycle_dt, 0.0, 1.0);
    const double alpha =
      std::clamp(sample_dt / map_consistency_window_s_, 0.0, 1.0);
    const double off_track = isFreeCell(base.x, base.y) ? 0.0 : 1.0;
    map_inconsistency_ema_ += alpha * (off_track - map_inconsistency_ema_);
    if (map_inconsistency_ema_ > map_consistency_threshold_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Map consistency failure: pose off drivable cells (ema %.2f > %.2f). Lost.",
        map_inconsistency_ema_, map_consistency_threshold_);
      announceReloc("RELOC: off-map", 0.95f, 0.55f, 0.10f);
      setState(LocalizationState::Lost);
      map_inconsistency_ema_ = 0.0;
    }
  } else {
    map_inconsistency_ema_ = 0.0;
  }

  // 실데이터에서 어느 단계가 깨지는지 보려면 각 단계의 출력을 같이 봐야 합니다.
  // ---- 신뢰도 지표 캐시 (출력 타이머가 pose와 같은 stamp로 발행) ----
  last_pos_sigma_ = std::sqrt(std::max(0.0,
    0.5 * (cloud.position_covariance(0, 0) + cloud.position_covariance(1, 1))));
  last_dominant_mass_ = mode_summary.dominant_mass;

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "fused(%.2f,%.2f,%.1f) pf(%.2f,%.2f,%.1f) d(%.3f,%.3f,%.3f) "
    "sig(%.2f,%.2f) neff=%.0f/%d conf(%.2f,%.2f) a(%.2f,%.2f) "
    "score(best %.2f mean %.2f) st=%d m=%u/%d(%.2f) dt=%.0fms scans=%d/%d "
    "ekf(%.2f,%.2f,%.1f) yawErr=%.1f",
    fused.x, fused.y, fused.yaw * 180.0 / kPi,
    cloud.x, cloud.y, cloud.yaw * 180.0 / kPi,
    motion.longitudinal, motion.lateral, motion.yaw,
    std::sqrt(std::max(0.0, cloud.position_covariance(0, 0))),
    std::sqrt(std::max(0.0, cloud.position_covariance(1, 1))),
    filter_.effectiveSampleSize(), particle_count,
    observability.confidence(0), observability.confidence(1),
    fused.scan_trust(0), fused.scan_trust(1),
    best_score, mean_score,
    static_cast<int>(state_), mode_summary.dominant, mode_summary.alive_modes,
    mode_summary.dominant_mass,
    cycle_dt * 1000.0, cycle_count_, scan_count_,
    odom_sample.x, odom_sample.y, odom_sample.yaw * 180.0 / kPi,
    normalizeAngle(odom_sample.yaw - raw_gyro_yaw_) * 180.0 / kPi);

  if (publish_particles_) {
    publishParticles(this->now());
  }
}

void mainNode::ensureLaserExtrinsic() {
  if (laser_extrinsic_ready_ || !tf_buffer_) {
    return;
  }
  // 추후 장착 위치가 바뀔 수 있으므로 TF에서 한 번만 읽어 반영합니다.
  try {
    const auto transform = tf_buffer_->lookupTransform(
      base_frame_, laser_frame_, tf2::TimePointZero);
    laser_extrinsic_.rear_to_laser_x = transform.transform.translation.x;
    laser_extrinsic_.rear_to_laser_y = transform.transform.translation.y;
    propagation_->setLaserExtrinsic(laser_extrinsic_);
    laser_extrinsic_ready_ = true;
    RCLCPP_INFO(
      this->get_logger(), "Laser extrinsic from TF: (%.3f, %.3f)",
      laser_extrinsic_.rear_to_laser_x, laser_extrinsic_.rear_to_laser_y);
  } catch (const std::exception &) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 10000,
      "base_link->laser TF unavailable; using configured fallback (%.3f, %.3f).",
      laser_extrinsic_.rear_to_laser_x, laser_extrinsic_.rear_to_laser_y);
  }

  // laser 프레임이 TF 트리에 항상 존재하도록 우리 extrinsic으로 static
  // 발행합니다. tf_static이 비어 있는 bag(icra)에서는 이게 유일한
  // base_link->laser 공급원이고, 이미 있는 경우엔 같은 값의 무해한 중복입니다.
  if (publish_laser_tf_ && static_broadcaster_ && !laser_tf_published_) {
    geometry_msgs::msg::TransformStamped laser_tf;
    laser_tf.header.stamp = this->now();
    laser_tf.header.frame_id = base_frame_;
    laser_tf.child_frame_id = laser_frame_;
    laser_tf.transform.translation.x = laser_extrinsic_.rear_to_laser_x;
    laser_tf.transform.translation.y = laser_extrinsic_.rear_to_laser_y;
    laser_tf.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(laser_tf);
    laser_tf_published_ = true;
  }
}

mainNode::Pose2D mainNode::laserToBase(const Pose2D &laser) const {
  // base = laser - R(yaw) * d
  const double cos_yaw = std::cos(laser.yaw);
  const double sin_yaw = std::sin(laser.yaw);
  Pose2D base;
  base.x = laser.x -
    (cos_yaw * laser_extrinsic_.rear_to_laser_x -
     sin_yaw * laser_extrinsic_.rear_to_laser_y);
  base.y = laser.y -
    (sin_yaw * laser_extrinsic_.rear_to_laser_x +
     cos_yaw * laser_extrinsic_.rear_to_laser_y);
  base.yaw = laser.yaw;
  return base;
}

mainNode::Pose2D mainNode::computeMapToOdom(
  const Pose2D &map_to_base, const Pose2D &odom_to_base) const {
  // T_mo = T_mb * inverse(T_ob)
  Pose2D result;
  result.yaw = normalizeAngle(map_to_base.yaw - odom_to_base.yaw);
  const double cos_yaw = std::cos(result.yaw);
  const double sin_yaw = std::sin(result.yaw);
  result.x = map_to_base.x - (cos_yaw * odom_to_base.x - sin_yaw * odom_to_base.y);
  result.y = map_to_base.y - (sin_yaw * odom_to_base.x + cos_yaw * odom_to_base.y);
  return result;
}

void mainNode::output_timer_callback() {
  // 궤적 버퍼를 읽기만 하는 순수 reader 경로입니다.
  const rclcpp::Time stamp = this->now();
  publishTransforms(stamp);
  publishPose(stamp);

  // 벤치마크용: 보간 레이턴시(지금 시각 - 궤적 최신 샘플 시각)와 직전 PF
  // 처리 시간을 함께 발행합니다. 출력이 비동기 보간이므로 이 합이 체감
  // 레이턴시에 해당합니다.
  const double newest = propagation_ ? propagation_->newestTrajectoryTime() : -1.0;
  if (latency_pub_ && newest > 0.0) {
    std_msgs::msg::Float32MultiArray message;
    const double interp_ms =
      std::max(0.0, (stamp.seconds() - newest)) * 1000.0;
    message.data = {static_cast<float>(interp_ms),
                    static_cast<float>(last_processing_ms_)};
    latency_pub_->publish(message);
  }
}

void mainNode::publishTransforms(const rclcpp::Time &stamp) {
  if (!propagation_ || !tf_broadcaster_) {
    return;
  }

  const auto sample = propagation_->poseAt(stamp.seconds());
  if (sample.valid) {
    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header.stamp = stamp;
    odom_to_base.header.frame_id = odom_frame_;
    odom_to_base.child_frame_id = base_frame_;
    odom_to_base.transform.translation.x = sample.x;
    odom_to_base.transform.translation.y = sample.y;
    odom_to_base.transform.rotation = yawToQuaternion(sample.yaw);
    tf_broadcaster_->sendTransform(odom_to_base);
  }

  if (map_to_odom_valid_) {
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = stamp;
    map_to_odom.header.frame_id = map_frame_;
    map_to_odom.child_frame_id = odom_frame_;
    map_to_odom.transform.translation.x = map_to_odom_.x;
    map_to_odom.transform.translation.y = map_to_odom_.y;
    map_to_odom.transform.rotation = yawToQuaternion(map_to_odom_.yaw);
    tf_broadcaster_->sendTransform(map_to_odom);
  }
}

// 로컬라이제이션을 얼마나 믿을 수 있는지를 [0,1] 하나로 요약합니다.
// 약한 고리(min) 방식 — 어느 한 축이 무너지면 전체가 내려갑니다. 로그우도
// 하나로는 판별이 안 되므로(맵/빔 수에 따라 절대값이 달라짐) 정규화된 정합도,
// 불일치 빔 비율, 구름 퍼짐, 가설 확정도를 함께 봅니다.
double mainNode::localizationConfidence() const {
  // (1) 스캔 정합도 — 이미 [0,1]로 정규화된 값
  const double score_term = last_score_health_;

  // (2) 불일치 빔 비율 — good 이하면 1, bad 이상이면 0
  const double frac_span = outlier_frac_bad_ - outlier_frac_good_;
  const double outlier_term = frac_span > 1.0e-9 ?
    1.0 - std::clamp(
      (last_outlier_fraction_ - outlier_frac_good_) / frac_span, 0.0, 1.0) : 1.0;

  // (3) beam skip 제안 비율 — 위치 상실 문턱에 가까울수록 0으로
  const double skip_term = beamskip_lost_fraction_ > 1.0e-9 ?
    1.0 - std::clamp(last_skip_fraction_ / beamskip_lost_fraction_, 0.0, 1.0) : 1.0;

  // (4) 파티클 구름 퍼짐 — 0.3 m에서 0.5가 되도록 사상
  constexpr double kSigmaRef = 0.3;
  const double spread_term = kSigmaRef * kSigmaRef /
    (kSigmaRef * kSigmaRef + last_pos_sigma_ * last_pos_sigma_);

  // (5) 지배 가설 질량 — 여러 가설이 살아 있으면 그만큼 불확실
  const double mass_term = std::clamp(last_dominant_mass_, 0.0, 1.0);

  const double weakest = std::min(
    {score_term, outlier_term, skip_term, spread_term, mass_term});

  // 상태로 최종 배율: Lost면 0, 판별 중이면 절반.
  double state_factor = 1.0;
  if (state_ == LocalizationState::Lost) {
    state_factor = 0.0;
  } else if (state_ == LocalizationState::Converging) {
    state_factor = 0.5;
  }
  return std::clamp(state_factor * weakest, 0.0, 1.0);
}

void mainNode::publishPose(const rclcpp::Time &stamp) {
  if (!map_to_odom_valid_ || !propagation_ || !pose_pub_) {
    return;
  }
  const auto sample = propagation_->poseAt(stamp.seconds());
  if (!sample.valid) {
    return;
  }

  // map->base = map->odom * odom->base
  const double cos_yaw = std::cos(map_to_odom_.yaw);
  const double sin_yaw = std::sin(map_to_odom_.yaw);
  geometry_msgs::msg::PoseWithCovarianceStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = map_frame_;
  message.pose.pose.position.x =
    map_to_odom_.x + cos_yaw * sample.x - sin_yaw * sample.y;
  message.pose.pose.position.y =
    map_to_odom_.y + sin_yaw * sample.x + cos_yaw * sample.y;
  message.pose.pose.orientation =
    yawToQuaternion(normalizeAngle(map_to_odom_.yaw + sample.yaw));

  const auto &fused = estimation_.last();
  if (fused.valid) {
    message.pose.covariance[0] = fused.position_covariance(0, 0);
    message.pose.covariance[1] = fused.position_covariance(0, 1);
    message.pose.covariance[6] = fused.position_covariance(1, 0);
    message.pose.covariance[7] = fused.position_covariance(1, 1);
    message.pose.covariance[35] = fused.yaw_variance;
  }
  pose_pub_->publish(message);

  // ---- nav_msgs/Odometry 로도 같은 결과를 발행 ----
  // frame_id = map, child_frame_id = base_link 이므로 pose는 map 기준 base_link
  // 절대 위치입니다(관례적인 휠 오돔과 달리 전역 보정이 반영된 값).
  if (!odom_pub_) {
    return;
  }
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = map_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose = message.pose;

  // twist는 child frame(base_link) 기준입니다. 속도 접근자가 없으므로 연속한
  // 출력 사이의 map 기준 이동을 body frame으로 돌려 유한차분으로 만듭니다.
  const double now_seconds = stamp.seconds();
  const Pose2D current{
    message.pose.pose.position.x,
    message.pose.pose.position.y,
    normalizeAngle(map_to_odom_.yaw + sample.yaw)};
  const double dt = now_seconds - odom_prev_time_;
  if (odom_prev_time_ > 0.0 && dt > 1.0e-4 && dt < 0.5) {
    const double dx = current.x - odom_prev_pose_.x;
    const double dy = current.y - odom_prev_pose_.y;
    const double cos_b = std::cos(current.yaw);
    const double sin_b = std::sin(current.yaw);
    odom.twist.twist.linear.x = (cos_b * dx + sin_b * dy) / dt;
    odom.twist.twist.linear.y = (-sin_b * dx + cos_b * dy) / dt;
    odom.twist.twist.angular.z =
      normalizeAngle(current.yaw - odom_prev_pose_.yaw) / dt;
  }
  odom_prev_time_ = now_seconds;
  odom_prev_pose_ = current;
  odom_pub_->publish(odom);

  // ---- 신뢰도 진단 (pose/odom과 동일한 stamp) ----
  // std_msgs 에는 header가 없으므로 stamp를 배열 앞 두 칸에 담아, 상위에서
  // pose와 정확히 짝지을 수 있게 합니다. 배열 의미는 layout.dim 라벨 참고.
  if (confidence_pub_) {
    std_msgs::msg::Float64MultiArray conf;
    conf.layout.dim.resize(3);
    const char *labels[3] = {"stamp_sec", "stamp_nanosec", "confidence"};
    for (std::size_t i = 0; i < 3; ++i) {
      conf.layout.dim[i].label = labels[i];
      conf.layout.dim[i].size = 1;
      conf.layout.dim[i].stride = 1;
    }
    const int64_t total_ns = stamp.nanoseconds();
    conf.data = {
      static_cast<double>(total_ns / 1000000000LL),
      static_cast<double>(total_ns % 1000000000LL),
      localizationConfidence()};
    confidence_pub_->publish(conf);
  }
}

void mainNode::publishParticles(const rclcpp::Time &stamp) {
  // 구독자가 없으면(실차에 RViz 없음) 1000포즈 직렬화를 통째로 생략합니다.
  if (!particles_pub_ || filter_.count() <= 0 ||
      particles_pub_->get_subscription_count() == 0) {
    return;
  }
  geometry_msgs::msg::PoseArray message;
  message.header.stamp = stamp;
  message.header.frame_id = map_frame_;
  message.poses.reserve(static_cast<std::size_t>(filter_.count()));
  const particle *particles = filter_.data();
  for (int32_t index = 0; index < filter_.count(); ++index) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = particles[index].x;
    pose.position.y = particles[index].y;
    pose.orientation = yawToQuaternion(particles[index].theta);
    message.poses.push_back(pose);
  }
  particles_pub_->publish(message);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mainNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
