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
  imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu");
  map_topic_ = this->declare_parameter<std::string>("map_topic", "/map");
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
  // 자체 맵 로더. map_dir는 보통 launch가 패키지 map 폴더로 주입합니다.
  map_loader_enabled_ = this->declare_parameter<bool>("map_loader.enabled", true);
  map_loader_dir_ = this->declare_parameter<std::string>("map_loader.map_dir", "");
  map_loader_name_ = this->declare_parameter<std::string>("map_loader.map_name", "map");
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
  relocalization_parameters_.hypothesis_verify_fraction = std::clamp(
    this->declare_parameter<double>(
      "relocalization.hypothesis_verify_fraction", 0.7), 0.0, 1.0);
  relocalization_parameters_.hypothesis_verify_inlier_m = std::max(0.05,
    this->declare_parameter<double>(
      "relocalization.hypothesis_verify_inlier_m", 0.2));
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
  score_fail_threshold_ = this->declare_parameter<double>(
    "tracking.score_fail_threshold", -9.0);
  multi_scan_count_ = std::max<int>(2, this->declare_parameter<int>(
    "relocalization.multi_scan_count", 12));
  multi_scan_spacing_m_ = std::max(0.05, this->declare_parameter<double>(
    "relocalization.multi_scan_spacing_m", 0.5));
  multi_scan_spacing_deg_ = std::max(1.0, this->declare_parameter<double>(
    "relocalization.multi_scan_spacing_deg", 15.0));
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

// P5(binary) PGM을 읽습니다. 주석 허용, maxval<256 가정.
bool loadPgm(const std::string &path, int &width, int &height,
             std::vector<uint8_t> &pixels) {
  std::ifstream file(path, std::ios::binary);
  if (!file) return false;
  std::string magic;
  file >> magic;
  if (magic != "P5") return false;
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
  if (width <= 0 || height <= 0 || maxval <= 0 || maxval > 255) return false;
  pixels.resize(static_cast<std::size_t>(width) * height);
  file.read(reinterpret_cast<char *>(pixels.data()), pixels.size());
  return static_cast<std::size_t>(file.gcount()) == pixels.size();
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
  if (!loadPgm(image_path, width, height, pixels)) {
    RCLCPP_ERROR(this->get_logger(), "map loader: cannot read PGM %s (P5 only)",
                 image_path.c_str());
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

  if (state_ == LocalizationState::Lost) {
    // 전역 탐색은 수백 ms가 걸리므로 매 scan마다 시도하지 않습니다.
    const rclcpp::Time now = this->now();
    if ((now - last_relocalize_attempt_).seconds() < relocalize_period_s_) {
      return;
    }
    last_relocalize_attempt_ = now;
    if (tryRelocalize(*msg)) {
      // 정지 한 스캔의 최적해는 복도 앨리어스일 수 있으므로 바로 확정하지
      // 않고, 주행으로 가설이 판별될 때까지 Converging에 머뭅니다.
      setState(LocalizationState::Converging);
      announceReloc("RELOC: global search", 0.15f, 0.75f, 0.30f);
      RCLCPP_INFO(this->get_logger(), "Global localization seeded. Converging.");
    }
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

bool mainNode::tryRelocalize(const sensor_msgs::msg::LaserScan &scan) {
  try {
    // EKF를 스캔 시각까지 진행시켜 이후 추적이 같은 기준에서 시작하게 합니다.
    propagation_->advanceTo(scan.header.stamp);

    const auto hypotheses = runGlobalSearch(scan);
    if (hypotheses.empty()) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "global search: no hypotheses");
      return false;
    }
    std::vector<ParticleFilter::ModeSeed> seeds;
    seeds.reserve(hypotheses.size());
    for (std::size_t index = 0; index < hypotheses.size(); ++index) {
      const auto &h = hypotheses[index];
      seeds.push_back(ParticleFilter::ModeSeed{h.x, h.y, h.yaw});
      RCLCPP_INFO(
        this->get_logger(), "reloc hypo[%zu](%.2f,%.2f,%.1f) score=%.4f%s",
        index, h.x, h.y, h.yaw * 180.0 / kPi, h.score,
        index == 0 ? " <- dominant seed" : "");
    }
    RCLCPP_INFO(
      this->get_logger(), "reloc %zu hypotheses, pts=%zu",
      hypotheses.size(), relocalization_->lastDiagnostics().scan_points);

    seedFilter(seeds);
    return true;
  } catch (const std::exception &error) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Relocalization failed: %s", error.what());
    return false;
  }
}

void mainNode::seedFilter(const std::vector<ParticleFilter::ModeSeed> &seeds) {
  filter_.initializeMultiple(seeds);
  // 파티클을 다시 뿌렸으므로 오래된 기준 pose와의 큰 차이를 적용하지 않도록
  // propagation 기준을 반드시 초기화합니다.
  propagation_->resetPropagationReference();
  // 융합 기준은 첫 번째(최고 점수) 가설에서 시작합니다.
  estimation_.reset(seeds.front().x, seeds.front().y, seeds.front().yaw);
  accumulated_translation_ = 0.0;
  accumulated_rotation_ = 0.0;
  converge_accumulated_ = 0.0;
  converge_rotation_accumulated_ = 0.0;
  map_inconsistency_ema_ = 0.0;
  beamskip_bad_frames_.clear();
  beamskip_bad_count_ = 0;
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
  const std::size_t keep = static_cast<std::size_t>(
    std::max(1, multi_scan_count_ - 1));
  while (scan_history_.size() > keep) {
    scan_history_.pop_front();
  }
}

std::vector<Relocalization::Hypothesis> mainNode::runGlobalSearch(
  const sensor_msgs::msg::LaserScan &scan) {
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

  // 스캔 이력 + 현재 스캔을 relocalizeMultiple/Trajectory 공용 체인으로
  // 접습니다. 연속 라이다 pose a->b의 상대 이동(a 프레임 기준)입니다.
  std::vector<sensor_msgs::msg::LaserScan> scans;
  std::vector<Relocalization::RelativeMotion> motions;
  if (pose_ok) {
    std::vector<std::array<double, 3>> chain;
    chain.reserve(scan_history_.size() + 1);
    for (const ScanSnapshot &snapshot : scan_history_) {
      scans.push_back(snapshot.scan);
      chain.push_back({snapshot.x, snapshot.y, snapshot.yaw});
    }
    scans.push_back(scan);
    chain.push_back({laser_x, laser_y, laser_yaw});
    motions.reserve(chain.size() - 1);
    for (std::size_t index = 0; index + 1 < chain.size(); ++index) {
      const double delta_x = chain[index + 1][0] - chain[index][0];
      const double delta_y = chain[index + 1][1] - chain[index][1];
      const double cos_ref = std::cos(-chain[index][2]);
      const double sin_ref = std::sin(-chain[index][2]);
      Relocalization::RelativeMotion motion;
      motion.dx = cos_ref * delta_x - sin_ref * delta_y;
      motion.dy = sin_ref * delta_x + cos_ref * delta_y;
      motion.dyaw = normalizeAngle(chain[index + 1][2] - chain[index][2]);
      motions.push_back(motion);
    }
  }

  // 코너를 포함한 충분한 궤적이 모였으면 궤적-모양 정합이 최우선입니다.
  // 스캔 채점은 낡은 맵/자기유사 트랙에서 앨리어스에 속지만, 주행로
  // 배치는 그대로라 궤적 정합은 진짜 배치를 찾습니다. 루프 위상은
  // 반환 직전 multi-scan 공동 채점이 가립니다.
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
      std::vector<std::array<double, 2>> points;
      points.reserve(pose_history_.size() + 1);
      for (const auto &entry : pose_history_) {
        points.push_back({entry[0], entry[1]});
      }
      points.push_back({laser_x, laser_y});
      try {
        auto hypotheses = relocalization_->relocalizeTrajectory(
          points, laser_yaw, scans, motions);
        if (!hypotheses.empty()) {
          RCLCPP_INFO(
            this->get_logger(),
            "trajectory fit: %zu hypotheses over %.1f m / %.0f deg, "
            "top(%.2f,%.2f,%.1f) joint=%.4f",
            hypotheses.size(), distance, rotation * 180.0 / kPi,
            hypotheses.front().x, hypotheses.front().y,
            hypotheses.front().yaw * 180.0 / kPi,
            hypotheses.front().score);
          return hypotheses;
        }
      } catch (const std::exception &error) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 10000,
          "Trajectory fit failed: %s", error.what());
      }
    }
  }
  if (scans.size() > 1) {
    return relocalization_->relocalizeMultiple(scans, motions);
  }
  return relocalization_->relocalizeMultiple(scan);
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
    }
    if (beam_watch_armed &&
        beamskip_bad_count_ >= beamskip_lost_count_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Beam-skip consensus collapse: %d/%d frames unexplained "
        "(latest skip fraction %.2f). Lost.",
        beamskip_bad_count_, beamskip_lost_window_,
        beam_skip.proposed_fraction);
      announceReloc("RELOC: beam consensus", 0.95f, 0.78f, 0.10f);
      setState(LocalizationState::Lost);
      beamskip_bad_frames_.clear();
      beamskip_bad_count_ = 0;
    }
  }
  const unsigned scoring_workers = scoring_threads_ > 0 ?
    static_cast<unsigned>(scoring_threads_) :
    std::min(4u, std::max(1u, std::thread::hardware_concurrency()));
  if (scoring_workers <= 1 || particle_count < 256) {
    for (int32_t index = 0; index < particle_count; ++index) {
      scoring_->scorePrepared(particles[index]);
    }
  } else {
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
  }

  // ---- 다중 가설 수렴 판정 ----
  // 정지 중에는 앨리어스를 구분할 수 없으므로(0526-1 초기화 실패의 교훈)
  // 지배 질량 조건을 유지한 채 실제로 주행한 거리로만 수렴을 셉니다.
  if (state_ == LocalizationState::Converging) {
    if (mode_summary.dominant_mass >= converge_mass_) {
      if (motion.valid) {
        converge_accumulated_ += std::hypot(motion.longitudinal, motion.lateral);
        converge_rotation_accumulated_ += std::abs(motion.yaw);
      }
      // 직선 주행만으로는 복도 앨리어스가 판별되지 않으므로(스래싱 교훈)
      // 거리와 함께 코너 통과에 해당하는 누적 회전을 요구합니다.
      if (converge_accumulated_ >= converge_distance_m_ &&
          converge_rotation_accumulated_ >=
            converge_rotation_deg_ * kPi / 180.0) {
        setState(LocalizationState::Tracking);
        RCLCPP_INFO(
          this->get_logger(),
          "Hypotheses converged after %.2f m / %.0f deg: mode %u mass %.2f "
          "(%d alive). Tracking.",
          converge_accumulated_,
          converge_rotation_accumulated_ * 180.0 / kPi,
          mode_summary.dominant, mode_summary.dominant_mass,
          mode_summary.alive_modes);
      }
    } else {
      converge_accumulated_ = 0.0;
      converge_rotation_accumulated_ = 0.0;
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
