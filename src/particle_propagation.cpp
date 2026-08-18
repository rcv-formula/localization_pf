#include "particle_propagation.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

#include "tf2/LinearMath/Quaternion.h"

namespace {
constexpr double kGravity = 9.80665;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

// EKF 구현에서 반복해서 쓰는 작은 수학/시간 보조 함수들입니다.
double square(double value) {
    return value * value;
}

double timeToSeconds(const builtin_interfaces::msg::Time &time) {
    return static_cast<double>(time.sec) + static_cast<double>(time.nanosec) * 1.0e-9;
}

builtin_interfaces::msg::Time secondsToTime(double seconds) {
    builtin_interfaces::msg::Time time;
    if (seconds <= 0.0) {
        return time;
    }

    time.sec = static_cast<int32_t>(std::floor(seconds));
    time.nanosec =
        static_cast<uint32_t>((seconds - static_cast<double>(time.sec)) * 1.0e9);
    if (time.nanosec >= 1000000000U) {
        time.sec += 1;
        time.nanosec -= 1000000000U;
    }
    return time;
}

double normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double angleDifference(double lhs, double rhs) {
    return normalizeAngle(lhs - rhs);
}

double positiveVariance(double variance) {
    return std::max(variance, 1.0e-12);
}

double gaussianConfidence(double value, double sigma) {
    sigma = std::max(std::abs(sigma), 1.0e-12);
    return std::exp(-square(value / sigma));
}

std::string trim(std::string value) {
    const auto begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

bool parseKeyValue(const std::string &line, std::string *key, double *value) {
    const auto delimiter = line.find(':');
    if (delimiter == std::string::npos) {
        return false;
    }

    *key = trim(line.substr(0, delimiter));
    std::stringstream stream(trim(line.substr(delimiter + 1)));
    stream >> *value;
    return !key->empty() && !stream.fail();
}
}  // 익명 namespace

// 기본 rear-to-IMU x 오프셋을 휠베이스로 두고 운동 추정기를 초기화합니다.
// 호출부는 이후 setImuExtrinsic()으로 이 값을 덮어쓸 수 있습니다.
particlePropagation::particlePropagation(float wheel_base, float motor_speed_gain)
    : motor_speed_gain(motor_speed_gain),
      wheel_base(wheel_base) {
    imu_extrinsic_.rear_to_imu_x = static_cast<double>(wheel_base);
    imu_extrinsic_.rear_to_imu_y = 0.0;
    imu_extrinsic_.rear_to_imu_z = 0.0;
    last_wheel_variance_ = ekf_params_.wheel_velocity_variance;

    ekf_state_.setZero();
    ekf_covariance_.setIdentity();
}

// 목표시각 보간과 시작 시 중력 보정에 사용할 IMU 원본 샘플을 저장합니다.
// 이 샘플은 이미 차량/body 좌표계로 변환되어 들어온다고 가정합니다.
void particlePropagation::imuGetter(sensor_msgs::msg::Imu current_imu){
    TimedImu imu_sample{current_imu, imu_extrinsic_};
    imu_history_.push_back(imu_sample);
    updateStartupGravityCalibration(imu_sample);
    // 궤적은 IMU 레이트로 채워야 deskew와 scan 시각 정합이 정확해집니다.
    integrateDeadReckoning(imu_sample);
    pruneSensorHistory(timeToSeconds(current_imu.header.stamp));
}

void particlePropagation::imuGetter(
    sensor_msgs::msg::Imu current_imu,
    ImuExtrinsic imu_extrinsic) {
    TimedImu imu_sample{current_imu, imu_extrinsic};
    imu_history_.push_back(imu_sample);
    updateStartupGravityCalibration(imu_sample);
    integrateDeadReckoning(imu_sample);
    pruneSensorHistory(timeToSeconds(current_imu.header.stamp));
}

// 경량 dead-reckoning: 행렬 없이 yaw만 적분하고 EKF의 bias/속도를 가져다 씁니다.
// 무거운 EKF는 scan 레이트로만 돌고, 궤적 해상도는 여기서 확보합니다.
void particlePropagation::integrateDeadReckoning(const TimedImu &imu_sample) {
    const double stamp = timeToSeconds(imu_sample.msg.header.stamp);
    if (!std::isfinite(stamp) || stamp <= 0.0) {
        return;
    }

    if (!dead_reckoning_ready_) {
        dead_reckoning_time_ = stamp;
        dead_reckoning_x_ = ekf_state_(kX);
        dead_reckoning_y_ = ekf_state_(kY);
        dead_reckoning_yaw_ = ekf_state_(kYaw);
        dead_reckoning_ready_ = true;
        appendTrajectory(stamp, dead_reckoning_x_, dead_reckoning_y_, dead_reckoning_yaw_);
        return;
    }

    const double dt = stamp - dead_reckoning_time_;
    if (dt <= ekf_params_.min_dt) {
        return;
    }

    // 속도는 EKF가 휠 측정으로 유지하는 추정치를 씁니다. EKF 갱신 사이에는
    // 상수로 두므로 급가감속 구간에서 mm 수준 오차가 남습니다.
    const double gyro_z = imu_sample.msg.angular_velocity.z - ekf_state_(kGyroZBias);
    const double velocity = ekf_state_(kVelocity);

    // 기울기 보정. 궤적 버퍼는 2D 지도 평면의 pose이므로, 차가 기울면
    //   (a) heading rate는 gyro z가 아니다  - IMU z축이 연직이 아니므로
    //   (b) 이동거리는 v*dt가 아니다        - 경사면 이동을 수평에 투영해야
    // ZYX 오일러 기구학:
    //   psi_dot   = (w_y sin(phi) + w_z cos(phi)) / cos(theta)
    //   ds_horiz  = v cos(theta) dt
    // theta,phi -> 0에서 기존 평지 식과 정확히 일치합니다.
    double yaw_rate = gyro_z;
    double horizontal_velocity = velocity;
    double tilt_roll = 0.0;
    double tilt_pitch = 0.0;
    if (chassisTiltFromImu(imu_sample, tilt_roll, tilt_pitch)) {
        const double cos_pitch = std::cos(tilt_pitch);
        // dr_tilt_max_deg로 이미 걸러지므로 cos_pitch는 0에서 충분히 멉니다.
        yaw_rate = (imu_sample.msg.angular_velocity.y * std::sin(tilt_roll) +
                    gyro_z * std::cos(tilt_roll)) / cos_pitch;
        horizontal_velocity = velocity * cos_pitch;
    }

    // 중점(midpoint) 적분이라 선회 중에도 1차 오차가 상쇄됩니다.
    const double mid_yaw = dead_reckoning_yaw_ + 0.5 * yaw_rate * dt;
    dead_reckoning_x_ += horizontal_velocity * dt * std::cos(mid_yaw);
    dead_reckoning_y_ += horizontal_velocity * dt * std::sin(mid_yaw);
    dead_reckoning_yaw_ = normalizeAngle(dead_reckoning_yaw_ + yaw_rate * dt);
    dead_reckoning_time_ = stamp;

    appendTrajectory(stamp, dead_reckoning_x_, dead_reckoning_y_, dead_reckoning_yaw_);
}

void particlePropagation::resyncDeadReckoning() {
    if (!ekf_initialized_) {
        return;
    }
    const double stamp = timeToSeconds(last_imu_time_);
    const double ekf_x = ekf_state_(kX);
    const double ekf_y = ekf_state_(kY);
    const double ekf_yaw = ekf_state_(kYaw);

    // 궤적이 아직 EKF 시각을 넘지 않았으면 기준점을 새로 잡기만 하면 됩니다.
    const auto anchor = poseAt(stamp);
    if (!dead_reckoning_ready_ || trajectory_.empty() || !anchor.valid ||
        trajectory_.back().time <= stamp + ekf_params_.min_dt) {
        dead_reckoning_time_ = stamp;
        dead_reckoning_x_ = ekf_x;
        dead_reckoning_y_ = ekf_y;
        dead_reckoning_yaw_ = ekf_yaw;
        dead_reckoning_ready_ = true;
        appendTrajectory(stamp, ekf_x, ekf_y, ekf_yaw);
        return;
    }

    // EKF는 scan 시각까지만 전진하는데, 그 시점에 궤적은 이미 더 최신 IMU까지
    // 쌓여 있습니다. 여기서 EKF 시각에 샘플을 append하면 appendTrajectory가
    // 그보다 최신인 꼬리를 전부 버리고, 다음 IMU 한 샘플의 각속도로 그 구간
    // 전체를 다시 영차유지 적분하게 됩니다. 매 스캔 반복되면 회전이 계속
    // 과소적분됩니다.
    //
    // 그래서 꼬리를 버리지 않고, EKF 해와 DR 해의 차이를 강체 변환으로
    // 이후 구간 전체에 적용합니다. IMU 레이트 상대운동은 그대로 보존됩니다.
    const double delta_yaw = normalizeAngle(ekf_yaw - anchor.yaw);
    const double cos_delta = std::cos(delta_yaw);
    const double sin_delta = std::sin(delta_yaw);
    const auto rebase = [&](double &x, double &y, double &yaw) {
        const double relative_x = x - anchor.x;
        const double relative_y = y - anchor.y;
        x = ekf_x + cos_delta * relative_x - sin_delta * relative_y;
        y = ekf_y + sin_delta * relative_x + cos_delta * relative_y;
        yaw = normalizeAngle(yaw + delta_yaw);
    };

    for (auto &sample : trajectory_) {
        if (sample.time <= stamp - ekf_params_.min_dt) {
            continue;
        }
        rebase(sample.x, sample.y, sample.yaw);
    }
    rebase(dead_reckoning_x_, dead_reckoning_y_, dead_reckoning_yaw_);
    dead_reckoning_ready_ = true;
}

// 같은 시각이 다시 들어오면 최신값으로 덮어써 단조 증가를 유지합니다.
void particlePropagation::appendTrajectory(
    double time, double x, double y, double yaw) {
    while (!trajectory_.empty() && trajectory_.back().time >= time - ekf_params_.min_dt) {
        trajectory_.pop_back();
    }
    trajectory_.push_back(TrajectorySample{time, x, y, yaw});
}

// L2 writer: 풀 EKF를 목표 시각까지 진행시키고 dead-reckoning 기준을 되맞춥니다.
void particlePropagation::advanceTo(builtin_interfaces::msg::Time target_time) {
    ekfOdom(target_time);
    resyncDeadReckoning();
}

particlePropagation::TrajectoryPose particlePropagation::poseAt(
    builtin_interfaces::msg::Time target_time) const {
    return poseAt(timeToSeconds(target_time));
}

// L2 reader: 궤적 링버퍼를 보간합니다. 상태를 바꾸지 않으므로 출력 경로가
// 아무리 자주 조회해도 PF가 보는 과거 시각 pose가 오염되지 않습니다.
particlePropagation::TrajectoryPose particlePropagation::poseAt(
    double target_seconds) const {
    TrajectoryPose result;
    if (trajectory_.empty() || !std::isfinite(target_seconds)) {
        return result;
    }

    const TrajectorySample &oldest = trajectory_.front();
    const TrajectorySample &newest = trajectory_.back();

    // 버퍼보다 과거는 되살릴 수 없습니다(호출부가 지연을 줄이거나 버퍼를 늘려야 함).
    if (target_seconds < oldest.time - ekf_params_.min_dt) {
        return result;
    }

    // 최신보다 미래면 마지막 구간 속도로 짧게 외삽합니다. 100Hz 출력이 IMU
    // 샘플 사이를 메우는 용도이며, 허용 폭을 넘으면 유효하지 않다고 알립니다.
    if (target_seconds > newest.time) {
        const double ahead = target_seconds - newest.time;
        if (ahead > ekf_params_.max_imu_extrapolation) {
            return result;
        }
        double vx = 0.0, vy = 0.0, vyaw = 0.0;
        if (trajectory_.size() >= 2) {
            const TrajectorySample &previous = trajectory_[trajectory_.size() - 2];
            const double span = newest.time - previous.time;
            if (span > ekf_params_.min_dt) {
                vx = (newest.x - previous.x) / span;
                vy = (newest.y - previous.y) / span;
                vyaw = angleDifference(newest.yaw, previous.yaw) / span;
            }
        }
        result.valid = true;
        result.time = target_seconds;
        result.x = newest.x + vx * ahead;
        result.y = newest.y + vy * ahead;
        result.yaw = normalizeAngle(newest.yaw + vyaw * ahead);
        return result;
    }

    // 버퍼 안이면 목표 시각을 감싸는 두 샘플을 찾아 선형 보간합니다.
    // deskew가 빔마다(수백 회) 호출하므로 선형 탐색 대신 이진 탐색을
    // 씁니다(버퍼는 시간 오름차순 — 결과는 동일).
    const auto it = std::lower_bound(
        trajectory_.begin(), trajectory_.end(), target_seconds,
        [](const TrajectorySample &sample, double value) {
            return sample.time < value;
        });
    std::size_t upper = it == trajectory_.end() ?
        trajectory_.size() - 1 :
        static_cast<std::size_t>(it - trajectory_.begin());
    if (upper == 0) {
        result.valid = true;
        result.time = trajectory_[0].time;
        result.x = trajectory_[0].x;
        result.y = trajectory_[0].y;
        result.yaw = trajectory_[0].yaw;
        return result;
    }

    const TrajectorySample &before = trajectory_[upper - 1];
    const TrajectorySample &after = trajectory_[upper];
    const double span = after.time - before.time;
    const double ratio = span > ekf_params_.min_dt ?
        std::clamp((target_seconds - before.time) / span, 0.0, 1.0) : 0.0;

    result.valid = true;
    result.time = target_seconds;
    result.x = before.x + (after.x - before.x) * ratio;
    result.y = before.y + (after.y - before.y) * ratio;
    // yaw는 wrap을 넘을 수 있으므로 각도 차이로 보간합니다.
    result.yaw = normalizeAngle(
        before.yaw + angleDifference(after.yaw, before.yaw) * ratio);
    return result;
}

// VESC 상태를 누적 주행거리[m]로 바꿔 히스토리에 저장합니다.
// EKF는 이후 보간된 누적 거리를 미분해서 속도 측정값으로 사용합니다.
// 산출 경로는 두 가지이며(ekf.wheel_use_displacement), 어느 쪽이든 아래
// 다운스트림(보간 -> 차분 -> 속도 관측)은 동일합니다.
void particlePropagation::wheelGetter(vesc_msgs::msg::VescStateStamped current_vesc){
    builtin_interfaces::msg::Time cur_time;
    cur_time.sec = current_vesc.header.stamp.sec;
    cur_time.nanosec = current_vesc.header.stamp.nanosec;
    const double stamp = timeToSeconds(cur_time);

    double trav_distance = 0.0;
    if (ekf_params_.wheel_use_displacement) {
        // (구 경로) 타코미터 카운트 -> 미터.
        const double tacometer =
            static_cast<double>(current_vesc.state.displacement) / 6.0;
        trav_distance = tacometer / static_cast<double>(motor_speed_gain) *
            ekf_params_.wheel_scale;
    } else {
        // (기본) vesc_to_odom과 동일하게 ERPM -> 속도[m/s]로 변환 후 적분.
        //   v = (state.speed - speed_to_erpm_offset) / speed_to_erpm_gain
        const double gain = ekf_params_.speed_to_erpm_gain;
        double speed = 0.0;
        if (std::isfinite(gain) && std::abs(gain) > 1.0e-9) {
            speed = (static_cast<double>(current_vesc.state.speed) -
                     ekf_params_.speed_to_erpm_offset) / gain;
        }
        // vesc_to_odom과 같은 저속 데드밴드.
        if (std::abs(speed) < ekf_params_.erpm_speed_deadband) {
            speed = 0.0;
        }
        speed *= ekf_params_.wheel_scale;
        if (erpm_prev_stamp_ > 0.0) {
            const double dt = stamp - erpm_prev_stamp_;
            // 비정상 간격(역행/과대 점프)은 적분하지 않고 시각만 갱신합니다.
            if (dt > ekf_params_.min_dt && dt < 1.0) {
                erpm_cumulative_distance_ += speed * dt;
            }
        }
        erpm_prev_stamp_ = stamp;
        trav_distance = erpm_cumulative_distance_;
    }
    wheel_history_.push_back(TimedWheel{cur_time, stamp, trav_distance});
    pruneSensorHistory(stamp);

    if (ekf_params_.enable_startup_gravity_calibration &&
        gravity_calibration_state_ == GravityCalibrationState::WaitingForStability) {
        gravity_wheel_history_.push_back(
            WheelStabilitySample{stamp, trav_distance});
    }
}

// 이후 particle propagation 경로에서 사용할 scan을 저장합니다.
void particlePropagation::scanGetter(sensor_msgs::msg::LaserScan currnet_scan){
    scan_queue.push(currnet_scan);
}

// imuGetter(msg)에서 사용할 기본 IMU extrinsic을 갱신합니다.
void particlePropagation::setImuExtrinsic(ImuExtrinsic imu_extrinsic) {
    imu_extrinsic_ = imu_extrinsic;
}

void particlePropagation::setMotionNoise(MotionNoise motion_noise) {
    motion_noise_.trans_per_trans = std::max(0.0, motion_noise.trans_per_trans);
    motion_noise_.lateral_per_trans = std::max(0.0, motion_noise.lateral_per_trans);
    motion_noise_.rot_per_rot = std::max(0.0, motion_noise.rot_per_rot);
    motion_noise_.rot_per_trans = std::max(0.0, motion_noise.rot_per_trans);
    motion_noise_.degeneracy_gain = std::max(0.0, motion_noise.degeneracy_gain);
    motion_noise_.recovery_gain = std::max(0.0, motion_noise.recovery_gain);
    motion_noise_.min_trans_sigma = std::max(0.0, motion_noise.min_trans_sigma);
    motion_noise_.min_yaw_sigma = std::max(0.0, motion_noise.min_yaw_sigma);
}

void particlePropagation::setLaserExtrinsic(LaserExtrinsic laser_extrinsic) {
    laser_extrinsic_ = laser_extrinsic;
}

void particlePropagation::setNoiseHealth(double health) {
    noise_health_ = std::clamp(health, 0.0, 1.0);
}

void particlePropagation::setNoiseShaping(
    const Eigen::Matrix2d &eigenvectors,
    const Eigen::Vector2d &confidence) {
    noise_shaping_vectors_ = eigenvectors;
    noise_shaping_confidence_ = confidence.cwiseMax(0.0).cwiseMin(1.0);
    noise_shaping_valid_ = confidence.allFinite() && eigenvectors.allFinite();
}

void particlePropagation::clearNoiseShaping() {
    noise_shaping_valid_ = false;
}

void particlePropagation::resetPropagationReference() {
    has_propagation_reference_ = false;
}

particlePropagation::EkfDiagnostics particlePropagation::ekfDiagnostics() const {
    EkfDiagnostics diag;
    if (!ekf_initialized_) {
        return diag;
    }
    diag.velocity = ekf_state_(kVelocity);
    diag.velocity_std =
        std::sqrt(std::max(0.0, ekf_covariance_(kVelocity, kVelocity)));
    diag.gyro_z_bias = ekf_state_(kGyroZBias);
    diag.accel_x_bias = ekf_state_(kAccelXBias);
    diag.accel_y_bias = ekf_state_(kAccelYBias);
    diag.roll_deg = ekf_state_(kRoll) / kDegToRad;
    diag.pitch_deg = ekf_state_(kPitch) / kDegToRad;
    diag.wheel_innov_sq_sum = wheel_innov_sq_sum_;
    diag.wheel_innov_count = wheel_innov_count_;
    return diag;
}

void particlePropagation::resetDynamicState() {
    if (!ekf_initialized_) {
        return;
    }
    // initializeEkf와 같은 값으로 되돌리되 pose(kX,kY,kYaw)는 건드리지
    // 않습니다. 속도는 0으로 두어도 다음 휠 샘플이 수십 ms 안에 복원합니다
    // (초기 공분산 0.30^2 이 측정을 즉시 지배).
    const int32_t reset_states[] = {
        kVelocity, kGyroZBias, kAccelXBias, kAccelYBias,
        kPitch, kPitchRate, kRoll, kRollRate};
    for (const int32_t index : reset_states) {
        ekf_state_(index) = 0.0;
        // 오염된 상태와 pose 사이의 교차 상관도 더는 유효하지 않습니다.
        ekf_covariance_.row(index).setZero();
        ekf_covariance_.col(index).setZero();
    }
    // 시동 정지 보정의 자이로 bias는 손으로 움직여도 참값이 변하지 않으므로
    // 이어받습니다(온라인으로 오염된 추정치를 버리고 보정값으로 복귀).
    if (gravity_calibration_.valid) {
        ekf_state_(kGyroZBias) = gravity_calibration_.gyro_z_bias;
    }
    ekf_covariance_(kVelocity, kVelocity) = square(0.30);
    ekf_covariance_(kGyroZBias, kGyroZBias) =
        ekf_params_.process_gyro_bias_std > 0.0 ? square(0.03) : 0.0;
    ekf_covariance_(kAccelXBias, kAccelXBias) = square(0.30);
    ekf_covariance_(kAccelYBias, kAccelYBias) = square(0.30);
    ekf_covariance_(kPitch, kPitch) = square(0.05);
    ekf_covariance_(kPitchRate, kPitchRate) = square(0.10);
    ekf_covariance_(kRoll, kRoll) = square(0.05);
    ekf_covariance_(kRollRate, kRollRate) = square(0.10);
    // 샘플 파생 캐시도 함께 비웁니다 — 들고 흔든 마지막 샘플의 각가속/가속이
    // 다음 예측의 초기값으로 쓰이면 리셋 직후 한 스텝이 또 오염됩니다.
    last_yaw_rate_ = 0.0;
    last_yaw_accel_ = 0.0;
    last_rear_accel_x_ = 0.0;
}

// 최신 센서 시각으로 전파합니다. 시간 정합이 필요한 경로는 target_time을 주는
// 오버로드를 사용하세요.
// 편의 오버로드입니다. writer(advanceTo)를 직접 수행한 뒤 reader에 위임합니다.
// ekf_initialized_는 advanceTo 경로에서만 참이 되므로 여기서 미리 걸러내면
// EKF가 영원히 초기화되지 않습니다.
void particlePropagation::propagation(particle *p_ptr, int32_t &particle_count) {
    const builtin_interfaces::msg::Time latest = ekfOdom().header.stamp;
    advanceTo(latest);
    propagation(p_ptr, particle_count, latest);
}

void particlePropagation::propagation(
    particle *p_ptr,
    int32_t &particle_count,
    builtin_interfaces::msg::Time target_time) {
    if (p_ptr == nullptr || particle_count <= 0) {
        return;
    }

    // 궤적 버퍼만 읽는 순수 reader입니다. 상태 전진은 호출부가 advanceTo()로
    // 미리 수행해야 하며, 덕분에 출력 경로가 앞서 나가도 여기서 보는 scan
    // 시각 pose가 흔들리지 않습니다.
    const TrajectoryPose sample = poseAt(target_time);
    if (!sample.valid) {
        return;
    }

    const double current_x = sample.x;
    const double current_y = sample.y;
    const double current_yaw = sample.yaw;

    // 첫 호출이나 relocalization 직후에는 기준만 잡고 파티클을 건드리지 않습니다.
    if (!has_propagation_reference_) {
        propagation_reference_x_ = current_x;
        propagation_reference_y_ = current_y;
        propagation_reference_yaw_ = current_yaw;
        has_propagation_reference_ = true;
        last_motion_delta_ = MotionDelta{};
        return;
    }

    // odom 프레임 이동량을 직전 body 프레임 성분으로 분해합니다. 전진 성분은
    // 부호를 유지하므로 후진에서도 각도 특이점이 생기지 않습니다.
    const double delta_x = current_x - propagation_reference_x_;
    const double delta_y = current_y - propagation_reference_y_;
    const double reference_cos = std::cos(propagation_reference_yaw_);
    const double reference_sin = std::sin(propagation_reference_yaw_);
    const double rear_longitudinal = reference_cos * delta_x + reference_sin * delta_y;
    const double rear_lateral = -reference_sin * delta_x + reference_cos * delta_y;
    const double delta_yaw = angleDifference(current_yaw, propagation_reference_yaw_);

    // EKF는 뒤차축 pose를 추정하지만 파티클은 라이다 pose입니다. 같은 body의
    // 서로 다른 점이라 프레임 오프셋이 증분에서 상쇄되지 않으므로,
    // 회전이 만드는 lever arm 이동 (R(delta_yaw) - I) * d 를 더해줍니다.
    const double yaw_cos = std::cos(delta_yaw);
    const double yaw_sin = std::sin(delta_yaw);
    const double lever_x = laser_extrinsic_.rear_to_laser_x;
    const double lever_y = laser_extrinsic_.rear_to_laser_y;
    const double longitudinal = rear_longitudinal +
        (yaw_cos - 1.0) * lever_x - yaw_sin * lever_y;
    const double lateral = rear_lateral +
        yaw_sin * lever_x + (yaw_cos - 1.0) * lever_y;

    propagation_reference_x_ = current_x;
    propagation_reference_y_ = current_y;
    propagation_reference_yaw_ = current_yaw;

    // estimation 모듈이 같은 델타를 다시 구하지 않도록 보관합니다.
    last_motion_delta_.valid = true;
    last_motion_delta_.longitudinal = longitudinal;
    last_motion_delta_.lateral = lateral;
    last_motion_delta_.yaw = delta_yaw;

    // 노이즈 크기는 전부 이동량에 비례합니다. 정지 중에는 0으로 수렴하므로
    // 별도 임계값 없이도 파티클 구름이 퍼지지 않습니다.
    // 병진 노이즈는 휠 슬립이 원인이므로 lever arm이 더해진 라이다 이동량이 아니라
    // 실제 측정 대상인 뒤차축 이동량에 비례시킵니다.
    const double travel = std::abs(rear_longitudinal);
    const double turn = std::abs(delta_yaw);
    // 스캔 정합이 무너질수록(health->0) 등방으로 노이즈를 키워 재포착
    // 반경을 확보합니다. 이동량 비례는 유지되므로 정지 중엔 여전히 0입니다.
    const double recovery_scale =
        1.0 + motion_noise_.recovery_gain * (1.0 - noise_health_);
    const double longitudinal_sigma = std::max(
        motion_noise_.min_trans_sigma,
        motion_noise_.trans_per_trans * travel) * recovery_scale;
    const double lateral_sigma = std::max(
        motion_noise_.min_trans_sigma,
        motion_noise_.lateral_per_trans * travel) * recovery_scale;
    const double yaw_sigma = std::max(
        motion_noise_.min_yaw_sigma,
        motion_noise_.rot_per_rot * turn + motion_noise_.rot_per_trans * travel) *
        recovery_scale;

    // 맵 기반 관측성으로 방향별 확산 배율을 만듭니다.
    // s(v) = 1 + c * (1 - confidence(v)) : 잘 관측되는 축은 1,
    // 퇴화 축은 1+c까지 연속적으로 커집니다(임계값 없음).
    // 관측성 조회가 실패한 경우 confidence는 0으로 남으므로 양쪽 모두
    // 최대로 확산됩니다. 이전에는 그 경로에서 확산이 오히려 꺼졌습니다.
    double scale_major = 1.0;
    double scale_minor = 1.0;
    if (motion_noise_.degeneracy_gain > 0.0) {
        const double confidence_major =
            noise_shaping_valid_ ? noise_shaping_confidence_(0) : 0.0;
        const double confidence_minor =
            noise_shaping_valid_ ? noise_shaping_confidence_(1) : 0.0;
        scale_major = 1.0 + motion_noise_.degeneracy_gain * (1.0 - confidence_major);
        scale_minor = 1.0 + motion_noise_.degeneracy_gain * (1.0 - confidence_minor);
    }

    std::normal_distribution<double> unit_normal(0.0, 1.0);
    for (int32_t index = 0; index < particle_count; ++index) {
        particle &current = p_ptr[index];
        const double particle_cos = std::cos(static_cast<double>(current.theta));
        const double particle_sin = std::sin(static_cast<double>(current.theta));

        // 이동량은 각 파티클 자신의 heading으로 회전시켜 map 프레임에 적용합니다.
        double move_x = particle_cos * longitudinal - particle_sin * lateral;
        double move_y = particle_sin * longitudinal + particle_cos * lateral;

        // 노이즈는 body 프레임에서 뽑아 파티클 heading으로 회전시킨 뒤,
        // 관측성 고유기저에서 방향별 배율을 적용합니다. 선형 변환이라
        // 파티클마다 Cholesky를 다시 구할 필요가 없습니다.
        const double body_noise_x = longitudinal_sigma * unit_normal(noise_rng_);
        const double body_noise_y = lateral_sigma * unit_normal(noise_rng_);
        double noise_x = particle_cos * body_noise_x - particle_sin * body_noise_y;
        double noise_y = particle_sin * body_noise_x + particle_cos * body_noise_y;

        if (noise_shaping_valid_) {
            const Eigen::Vector2d noise_map(noise_x, noise_y);
            const Eigen::Vector2d in_eigen_basis =
                noise_shaping_vectors_.transpose() * noise_map;
            const Eigen::Vector2d scaled(
                in_eigen_basis(0) * scale_major,
                in_eigen_basis(1) * scale_minor);
            const Eigen::Vector2d shaped = noise_shaping_vectors_ * scaled;
            noise_x = shaped(0);
            noise_y = shaped(1);
        } else {
            // 관측성을 모르면 방향을 고를 수 없으므로 등방으로 최대 확산합니다
            // (이때 scale_major == scale_minor 이므로 기저 회전이 불필요).
            noise_x *= scale_major;
            noise_y *= scale_major;
        }

        current.x = static_cast<float>(
            static_cast<double>(current.x) + move_x + noise_x);
        current.y = static_cast<float>(
            static_cast<double>(current.y) + move_y + noise_y);
        current.theta = static_cast<float>(normalizeAngle(
            static_cast<double>(current.theta) + delta_yaw +
            yaw_sigma * unit_normal(noise_rng_)));
    }
}

// 사용자가 넘긴 파라미터를 사용하기 전에 안전한 수치 범위로 제한합니다.
void particlePropagation::setEkfParameters(EkfParameters params) {
    params.yaw_rate_filter_window = std::max(2, params.yaw_rate_filter_window);
    params.yaw_rate_filter_order = std::clamp(params.yaw_rate_filter_order, 1, 4);
    params.min_dt = std::max(params.min_dt, 1.0e-6);
    params.history_keep_time = std::max(params.history_keep_time, params.min_dt);
    params.max_imu_extrapolation = std::max(0.0, params.max_imu_extrapolation);
    params.max_wheel_extrapolation = std::max(0.0, params.max_wheel_extrapolation);
    params.wheel_velocity_variance = positiveVariance(params.wheel_velocity_variance);
    params.gyro_roll_rate_variance = positiveVariance(params.gyro_roll_rate_variance);
    params.gyro_pitch_rate_variance = positiveVariance(params.gyro_pitch_rate_variance);
    params.lateral_accel_variance = positiveVariance(params.lateral_accel_variance);
    params.slip_reference_variance = positiveVariance(params.slip_reference_variance);
    params.max_wheel_variance_scale = std::max(1.0, params.max_wheel_variance_scale);
    params.gravity_startup_timeout = std::max(0.0, params.gravity_startup_timeout);
    params.gravity_stability_window =
        std::max(params.gravity_stability_window, params.min_dt);
    params.gravity_stability_score_min =
        std::clamp(params.gravity_stability_score_min, 0.0, 1.0);
    params.gravity_min_imu_samples = std::max(2, params.gravity_min_imu_samples);
    params.gravity_gyro_mean_sigma =
        std::max(std::abs(params.gravity_gyro_mean_sigma), 1.0e-12);
    params.gravity_gyro_std_sigma =
        std::max(std::abs(params.gravity_gyro_std_sigma), 1.0e-12);
    params.gravity_accel_std_sigma =
        std::max(std::abs(params.gravity_accel_std_sigma), 1.0e-12);
    params.gravity_accel_norm_sigma =
        std::max(std::abs(params.gravity_accel_norm_sigma), 1.0e-12);
    params.gravity_wheel_delta_sigma =
        std::max(std::abs(params.gravity_wheel_delta_sigma), 1.0e-12);
    params.gravity_wheel_velocity_sigma =
        std::max(std::abs(params.gravity_wheel_velocity_sigma), 1.0e-12);
    params.accel_gravity_sign = params.accel_gravity_sign >= 0.0 ? 1.0 : -1.0;
    params.dr_tilt_max_deg = std::clamp(params.dr_tilt_max_deg, 1.0, 80.0);
    ekf_params_ = params;
}

// scan 기반 propagation 튜닝을 위한 임시 entropy 보조 함수입니다.
scan_entropy particlePropagation::calcScanEntropy(sensor_msgs::msg::LaserScan scan){
    scan_entropy entropy{};
    entropy.theta = scan.angle_min;
    entropy.min = scan.range_min;
    entropy.max = scan.range_max;
    return entropy;
}

// 현재 보유한 센서 중 가장 최신 timestamp의 오도메트리 추정값을 반환합니다.
nav_msgs::msg::Odometry particlePropagation::ekfOdom() {
    builtin_interfaces::msg::Time target_time = last_imu_time_;
    double target_seconds = ekf_initialized_ ? timeToSeconds(last_imu_time_) : 0.0;

    // 동기화를 위해 기다리지 않고 IMU/휠 히스토리 중 가장 최신 timestamp를 고릅니다.
    if (!imu_history_.empty() && imu_history_.back().msg.header.stamp.sec != 0) {
        const double imu_time = timeToSeconds(imu_history_.back().msg.header.stamp);
        if (imu_time >= target_seconds) {
            target_seconds = imu_time;
            target_time = imu_history_.back().msg.header.stamp;
        }
    }
    if (!wheel_history_.empty()) {
        const double wheel_time = wheel_history_.back().time;
        if (wheel_time >= target_seconds) {
            target_seconds = wheel_time;
            target_time = wheel_history_.back().stamp;
        }
    }

    return ekfOdom(target_time);
}

// 호출부가 지정한 timestamp의 오도메트리 추정값을 반환합니다.
// 센서값을 보간하거나 아주 짧게 외삽해 EKF 지연을 최소화합니다.
nav_msgs::msg::Odometry particlePropagation::ekfOdom(
    builtin_interfaces::msg::Time target_time) {
    const double target_seconds = timeToSeconds(target_time);
    TimedImu target_imu;
    // 예측 단계에는 목표시각 IMU 샘플이 필요합니다. 요청 시각이 짧은 외삽
    // 허용 범위를 벗어나면 마지막 유효 추정값을 유지합니다.
    if (!interpolateImu(target_seconds, &target_imu)) {
        return buildOdomMsg(last_imu_time_);
    }

    if (ekf_initialized_) {
        const double state_time = timeToSeconds(last_imu_time_);
        // 이 구현은 causal/저지연 경로라서, 과거 측정값 적용을 위해 상태를
        // 되감지 않습니다.
        if (target_seconds < state_time - ekf_params_.min_dt) {
            return buildOdomMsg(last_imu_time_);
        }
        // 호출부가 현재 상태 시각을 요청한 경우, 같은 timestamp에서 휠 속도
        // 업데이트만 새로 반영합니다.
        if (target_seconds <= state_time + ekf_params_.min_dt) {
            double wheel_distance = 0.0;
            if (interpolateWheelDistance(target_seconds, &wheel_distance)) {
                applyWheelMeasurementAtTime(
                    target_seconds,
                    target_time,
                    wheel_distance,
                    last_wheel_variance_);
            }
            return buildOdomMsg(target_time);
        }

        // 직전 상태 시각과 목표 시각 사이에 쌓인 IMU 샘플을 순서대로 모두
        // 반영합니다. 이게 없으면 목표시각 샘플 하나의 각속도로 구간 전체를
        // 영차유지 적분하게 되어, IMU(수십~수백 Hz)보다 성긴 scan 주기에서
        // 회전이 크게 과소적분됩니다. 데이터셋 실측에서 EKF yaw가 정답의
        // 53%까지 떨어졌고, 그 오차가 propagation의 lateral 성분을 오염시켜
        // 파티클을 맵 밖으로 끌고 갔습니다.
        for (const TimedImu &sample : imu_history_) {
            const double sample_time = timeToSeconds(sample.msg.header.stamp);
            if (sample_time <= state_time + ekf_params_.min_dt) {
                continue;
            }
            if (sample_time >= target_seconds - ekf_params_.min_dt) {
                break;
            }
            predictWithImu(sample);
        }
    }

    // 남은 구간은 보간된 IMU 샘플로 목표시각까지 마저 예측합니다.
    target_imu.msg.header.stamp = target_time;
    predictWithImu(target_imu);

    // 목표시각의 누적 휠 이동거리로 뒤차축 속도를 업데이트합니다.
    double wheel_distance = 0.0;
    if (interpolateWheelDistance(target_seconds, &wheel_distance)) {
        applyWheelMeasurementAtTime(
            target_seconds,
            target_time,
            wheel_distance,
            last_wheel_variance_);
    }

    return buildOdomMsg(target_time);
}

// 첫 유효 IMU timestamp에서 필터를 초기화합니다.
void particlePropagation::initializeEkf(double stamp) {
    (void)stamp;
    ekf_initialized_ = true;
    ekf_state_.setZero();
    ekf_covariance_.setZero();

    // 시작 정지 보정이 이미 끝났다면 그 자이로 bias를 그대로 이어받습니다.
    if (gravity_calibration_.valid) {
        ekf_state_(kGyroZBias) = gravity_calibration_.gyro_z_bias;
    }

    ekf_covariance_(kX, kX) = square(0.10);
    ekf_covariance_(kY, kY) = square(0.10);
    ekf_covariance_(kYaw, kYaw) = square(0.10);
    ekf_covariance_(kVelocity, kVelocity) = square(0.30);
    // yaw-rate bias를 얼릴 때는 공분산도 0으로 둡니다. 그래야 어떤 측정
    // 업데이트도 (P * H^T의 해당 성분이 0이므로) bias를 움직이지 못합니다.
    // bias 행은 random walk라 F도 이 0을 그대로 보존합니다.
    ekf_covariance_(kGyroZBias, kGyroZBias) =
        ekf_params_.process_gyro_bias_std > 0.0 ? square(0.03) : 0.0;
    ekf_covariance_(kAccelXBias, kAccelXBias) = square(0.30);
    ekf_covariance_(kAccelYBias, kAccelYBias) = square(0.30);
    ekf_covariance_(kPitch, kPitch) = square(0.05);
    ekf_covariance_(kPitchRate, kPitchRate) = square(0.10);
    ekf_covariance_(kRoll, kRoll) = square(0.05);
    ekf_covariance_(kRollRate, kRollRate) = square(0.10);
}

// 목표시각 IMU 샘플을 사용해 EKF 예측/업데이트 한 사이클을 수행합니다.
void particlePropagation::predictWithImu(const TimedImu &imu_sample) {
    const double stamp = timeToSeconds(imu_sample.msg.header.stamp);
    const double yaw_rate = imu_sample.msg.angular_velocity.z - ekf_state_(kGyroZBias);

    // 첫 IMU 샘플은 필터 시계와 미분 히스토리의 기준점으로만 사용합니다.
    if (!ekf_initialized_) {
        initializeEkf(stamp);
        last_imu_time_ = imu_sample.msg.header.stamp;
        last_yaw_rate_ = yaw_rate;
        last_yaw_accel_ = updateYawRateDerivative(stamp, yaw_rate);
        return;
    }

    // propagation 간격을 계산하고 yaw 가속도 추정값을 갱신합니다.
    const double last_stamp = timeToSeconds(last_imu_time_);
    const double dt = stamp - last_stamp;
    last_yaw_rate_ = yaw_rate;
    last_yaw_accel_ = updateYawRateDerivative(stamp, yaw_rate);

    if (dt <= ekf_params_.min_dt) {
        last_imu_time_ = imu_sample.msg.header.stamp;
        return;
    }

    // EKF 예측 단계: 차량 동역학으로 상태를 전파하고, 수치 process
    // 야코비안으로 공분산도 함께 전파합니다.
    const StateMatrix process_jacobian =
        numericalProcessJacobian(ekf_state_, imu_sample, dt, last_yaw_accel_);
    ekf_state_ = predictState(ekf_state_, imu_sample, dt, last_yaw_accel_);
    ekf_state_(kYaw) = normalizeAngle(ekf_state_(kYaw));

    // 대각 process noise입니다. pose, 속도, bias, suspension 상태가 얼마나
    // 빠르게 drift할 수 있는지를 이 파라미터들이 조절합니다.
    StateMatrix process_noise = StateMatrix::Zero();
    process_noise(kX, kX) = square(ekf_params_.process_position_std);
    process_noise(kY, kY) = square(ekf_params_.process_position_std);
    process_noise(kYaw, kYaw) = square(ekf_params_.process_yaw_std);
    process_noise(kVelocity, kVelocity) = square(ekf_params_.process_velocity_std);
    process_noise(kGyroZBias, kGyroZBias) = square(ekf_params_.process_gyro_bias_std);
    process_noise(kAccelXBias, kAccelXBias) = square(ekf_params_.process_accel_bias_std);
    process_noise(kAccelYBias, kAccelYBias) = square(ekf_params_.process_accel_bias_std);
    process_noise(kPitch, kPitch) = square(ekf_params_.process_tilt_std);
    process_noise(kPitchRate, kPitchRate) = square(ekf_params_.process_tilt_rate_std);
    process_noise(kRoll, kRoll) = square(ekf_params_.process_tilt_std);
    process_noise(kRollRate, kRollRate) = square(ekf_params_.process_tilt_rate_std);

    ekf_covariance_ =
        process_jacobian * ekf_covariance_ * process_jacobian.transpose() +
        process_noise * dt;
    ekf_covariance_ = 0.5 * (ekf_covariance_ + ekf_covariance_.transpose());

    // gyro x/y는 roll_rate와 pitch_rate에 대한 직접 rate 관측값으로 사용합니다.
    StateVector jacobian = StateVector::Zero();
    jacobian(kRollRate) = 1.0;
    applyScalarUpdate(
        imu_sample.msg.angular_velocity.x - ekf_state_(kRollRate),
        jacobian,
        ekf_params_.gyro_roll_rate_variance);

    jacobian.setZero();
    jacobian(kPitchRate) = 1.0;
    applyScalarUpdate(
        imu_sample.msg.angular_velocity.y - ekf_state_(kPitchRate),
        jacobian,
        ekf_params_.gyro_pitch_rate_variance);

    // 횡가속도는 약한 EKF 측정값이면서, 휠 속도 신뢰도를 조절하는 연속적인
    // slip/load 지표로도 사용합니다.
    const double predicted_ay =
        predictLateralAccelMeasurement(ekf_state_, imu_sample, last_yaw_accel_);
    const double lateral_residual =
        imu_sample.msg.linear_acceleration.y - predicted_ay;
    const double normalized_lateral_error =
        square(lateral_residual) / ekf_params_.slip_reference_variance;
    last_slip_score_ =
        normalized_lateral_error / (1.0 + normalized_lateral_error);
    last_slip_score_ = std::clamp(last_slip_score_, 0.0, 1.0);

    jacobian = numericalLateralJacobian(ekf_state_, imu_sample, last_yaw_accel_);
    const double lateral_variance =
        ekf_params_.lateral_accel_variance *
        (1.0 + ekf_params_.lateral_update_gain * ekf_params_.slip_gain *
            std::pow(last_slip_score_, ekf_params_.slip_power));
    applyScalarUpdate(lateral_residual, jacobian, lateral_variance);

    // EKF 업데이트 이후 뒤차축 가속도를 재구성해 진단값과 적응형 휠 공분산
    // 스케일링에 사용합니다.
    const double yaw_rate_after_update =
        imu_sample.msg.angular_velocity.z - ekf_state_(kGyroZBias);
    const Eigen::Vector3d gravity_body = calcGravityBody(ekf_state_);
    last_rear_accel_x_ =
        imu_sample.msg.linear_acceleration.x -
        ekf_state_(kAccelXBias) -
        gravity_body.x() +
        last_yaw_accel_ * imu_sample.extrinsic.rear_to_imu_y +
        square(yaw_rate_after_update) * imu_sample.extrinsic.rear_to_imu_x;
    last_rear_accel_y_ =
        imu_sample.msg.linear_acceleration.y -
        ekf_state_(kAccelYBias) -
        gravity_body.y() -
        last_yaw_accel_ * imu_sample.extrinsic.rear_to_imu_x +
        square(yaw_rate_after_update) * imu_sample.extrinsic.rear_to_imu_y;

    // 횡방향 일관성이나 suspension/load 지표가 slip 또는 낮은 휠 신뢰도를
    // 시사하면 휠 속도 분산을 부드럽게 키웁니다.
    const double load_score =
        ekf_params_.load_ax_gain * std::abs(last_rear_accel_x_) +
        ekf_params_.load_ay_gain * std::abs(last_rear_accel_y_) +
        ekf_params_.load_pitch_gain * std::abs(ekf_state_(kPitch)) +
        ekf_params_.load_roll_gain * std::abs(ekf_state_(kRoll));
    double wheel_scale =
        1.0 + ekf_params_.slip_gain *
        std::pow(last_slip_score_, ekf_params_.slip_power);
    wheel_scale *= 1.0 + ekf_params_.load_gain * load_score;
    wheel_scale = std::clamp(
        wheel_scale,
        1.0,
        ekf_params_.max_wheel_variance_scale);
    last_wheel_variance_ = ekf_params_.wheel_velocity_variance * wheel_scale;

    last_imu_time_ = imu_sample.msg.header.stamp;
}

// 호환성을 위해 남겨둔 기존 queue 기반 휠 업데이트 경로입니다. 현재 활성
// 저지연 경로는 보간 히스토리와 applyWheelMeasurementAtTime()을 사용합니다.
void particlePropagation::processWheelMeasurementsUpTo(
    double stamp,
    double wheel_variance) {
    while (!wheel_queue.empty() && !wheel_time_queue.empty()) {
        // 요청 timestamp까지의 휠 메시지를 모두 소비합니다.
        const builtin_interfaces::msg::Time wheel_time = wheel_time_queue.front();
        const double wheel_stamp = timeToSeconds(wheel_time);
        if (wheel_stamp > stamp + ekf_params_.min_dt) {
            break;
        }

        const double wheel_distance = wheel_queue.front();
        wheel_queue.pop();
        wheel_time_queue.pop();

        // 첫 누적 샘플은 기준 누적거리만 설정합니다.
        if (!has_previous_wheel_) {
            has_previous_wheel_ = true;
            previous_wheel_distance_ = wheel_distance;
            previous_wheel_time_ = wheel_time;
            continue;
        }

        const double wheel_dt = wheel_stamp - timeToSeconds(previous_wheel_time_);
        const double wheel_delta = wheel_distance - previous_wheel_distance_;
        previous_wheel_distance_ = wheel_distance;
        previous_wheel_time_ = wheel_time;

        // 누적거리 차이를 평균 속도로 변환합니다.
        if (wheel_dt <= ekf_params_.min_dt) {
            continue;
        }

        const double measured_velocity = wheel_delta / wheel_dt;
        if (!std::isfinite(measured_velocity) ||
            std::abs(measured_velocity) > ekf_params_.max_wheel_speed) {
            continue;
        }

        // 휠 측정 모델: z = v_R.
        StateVector jacobian = StateVector::Zero();
        jacobian(kVelocity) = 1.0;

        double velocity_residual = measured_velocity - ekf_state_(kVelocity);
        wheel_innov_sq_sum_ += square(velocity_residual);
        ++wheel_innov_count_;
        if (std::abs(ekf_state_(kVelocity)) < ekf_params_.velocity_deadzone &&
            std::abs(measured_velocity) < ekf_params_.velocity_deadzone) {
            velocity_residual = 0.0;
        }

        applyScalarUpdate(
            velocity_residual,
            jacobian,
            positiveVariance(wheel_variance));
    }
}

// 범용 1차원 EKF 측정 업데이트입니다.
void particlePropagation::applyScalarUpdate(
    double residual,
    const StateVector &jacobian,
    double variance) {
    if (!std::isfinite(residual)) {
        return;
    }

    // 스칼라 측정 z = h(x)에 대한 innovation 공분산과 Kalman gain입니다.
    variance = positiveVariance(variance);
    const double innovation_variance =
        (jacobian.transpose() * ekf_covariance_ * jacobian)(0, 0) + variance;
    if (!std::isfinite(innovation_variance) || innovation_variance <= 0.0) {
        return;
    }

    const StateVector kalman_gain =
        ekf_covariance_ * jacobian / innovation_variance;
    ekf_state_ += kalman_gain * residual;
    ekf_state_(kYaw) = normalizeAngle(ekf_state_(kYaw));

    // Joseph form 공분산 업데이트는 P = (I-KH)P만 쓰는 것보다 공분산 대칭성과
    // 수치 안정성을 더 잘 유지합니다.
    const StateMatrix identity = StateMatrix::Identity();
    const StateMatrix update = identity - kalman_gain * jacobian.transpose();
    ekf_covariance_ =
        update * ekf_covariance_ * update.transpose() +
        kalman_gain * variance * kalman_gain.transpose();
    ekf_covariance_ = 0.5 * (ekf_covariance_ + ekf_covariance_.transpose());
}

// 뒤차축 기준 차량 상태에 대한 비선형 process 모델입니다.
particlePropagation::StateVector particlePropagation::predictState(
    const StateVector &state,
    const TimedImu &imu_sample,
    double dt,
    double yaw_accel) const {
    StateVector predicted = state;
    const double yaw_rate = imu_sample.msg.angular_velocity.z - state(kGyroZBias);
    const Eigen::Vector3d gravity_body = calcGravityBody(state);

    // 오프셋 위치의 IMU 가속도를 뒤차축 가속도로 변환합니다.
    // r_x/r_y 항은 구심 가속도와 각가속도에 의한 영향을 보상합니다.
    const double rear_accel_x =
        imu_sample.msg.linear_acceleration.x -
        state(kAccelXBias) -
        gravity_body.x() +
        yaw_accel * imu_sample.extrinsic.rear_to_imu_y +
        square(yaw_rate) * imu_sample.extrinsic.rear_to_imu_x;
    const double rear_accel_y =
        imu_sample.msg.linear_acceleration.y -
        state(kAccelYBias) -
        gravity_body.y() -
        yaw_accel * imu_sample.extrinsic.rear_to_imu_x +
        square(yaw_rate) * imu_sample.extrinsic.rear_to_imu_y;

    // 중간 heading/속도를 사용해 평면상의 뒤차축 운동을 적분합니다.
    const double midpoint_yaw = state(kYaw) + 0.5 * yaw_rate * dt;
    const double midpoint_velocity = state(kVelocity) + 0.5 * rear_accel_x * dt;

    predicted(kX) =
        state(kX) + midpoint_velocity * std::cos(midpoint_yaw) * dt;
    predicted(kY) =
        state(kY) + midpoint_velocity * std::sin(midpoint_yaw) * dt;
    predicted(kYaw) = normalizeAngle(state(kYaw) + yaw_rate * dt);
    predicted(kVelocity) = state(kVelocity) + rear_accel_x * dt;

    // 종가속도로 구동되는 2차 pitch suspension 모델입니다.
    const double pitch_w = ekf_params_.pitch_natural_frequency;
    const double pitch_ddot =
        -2.0 * ekf_params_.pitch_damping_ratio * pitch_w * state(kPitchRate) -
        square(pitch_w) * state(kPitch) +
        ekf_params_.pitch_accel_gain * rear_accel_x;
    predicted(kPitchRate) = state(kPitchRate) + pitch_ddot * dt;
    predicted(kPitch) = state(kPitch) + predicted(kPitchRate) * dt;

    // 횡가속도로 구동되는 2차 roll suspension 모델입니다.
    const double roll_w = ekf_params_.roll_natural_frequency;
    const double roll_ddot =
        -2.0 * ekf_params_.roll_damping_ratio * roll_w * state(kRollRate) -
        square(roll_w) * state(kRoll) +
        ekf_params_.roll_accel_gain * rear_accel_y;
    predicted(kRollRate) = state(kRollRate) + roll_ddot * dt;
    predicted(kRoll) = state(kRoll) + predicted(kRollRate) * dt;

    return predicted;
}

// 수치 process 야코비안 F = d f(x,u) / d x 입니다.
particlePropagation::StateMatrix particlePropagation::numericalProcessJacobian(
    const StateVector &state,
    const TimedImu &imu_sample,
    double dt,
    double yaw_accel) const {
    StateMatrix jacobian = StateMatrix::Zero();

    for (int32_t i = 0; i < kStateSize; ++i) {
        // 각 상태 차원마다 중앙차분을 적용합니다. yaw 행은 +/-pi 불연속을
        // 피하기 위해 wrap된 각도 차이를 사용합니다.
        const double epsilon = 1.0e-6 * std::max(1.0, std::abs(state(i)));
        StateVector plus = state;
        StateVector minus = state;
        plus(i) += epsilon;
        minus(i) -= epsilon;

        const StateVector predicted_plus =
            predictState(plus, imu_sample, dt, yaw_accel);
        const StateVector predicted_minus =
            predictState(minus, imu_sample, dt, yaw_accel);
        StateVector diff = (predicted_plus - predicted_minus) / (2.0 * epsilon);
        diff(kYaw) =
            angleDifference(predicted_plus(kYaw), predicted_minus(kYaw)) /
            (2.0 * epsilon);
        jacobian.col(i) = diff;
    }

    return jacobian;
}

// 현재 상태에서 IMU y축 가속도를 예측합니다. 이 값은 slip 검출과 약한 bias
// 보정에 사용하는 횡방향 일관성 모델입니다.
double particlePropagation::predictLateralAccelMeasurement(
    const StateVector &state,
    const TimedImu &imu_sample,
    double yaw_accel) const {
    const double yaw_rate = imu_sample.msg.angular_velocity.z - state(kGyroZBias);
    const Eigen::Vector3d gravity_body = calcGravityBody(state);

    return state(kVelocity) * yaw_rate +
           yaw_accel * imu_sample.extrinsic.rear_to_imu_x -
           square(yaw_rate) * imu_sample.extrinsic.rear_to_imu_y +
           gravity_body.y() +
           state(kAccelYBias);
}

// 횡가속도 측정 모델의 수치 야코비안입니다.
particlePropagation::StateVector particlePropagation::numericalLateralJacobian(
    const StateVector &state,
    const TimedImu &imu_sample,
    double yaw_accel) const {
    StateVector jacobian = StateVector::Zero();

    for (int32_t i = 0; i < kStateSize; ++i) {
        // 모델이 계속 변하는 동안에도 견고하게 유지하려고 중앙차분을 씁니다.
        // 이렇게 하면 해석적인 H를 직접 계속 관리하지 않아도 됩니다.
        const double epsilon = 1.0e-6 * std::max(1.0, std::abs(state(i)));
        StateVector plus = state;
        StateVector minus = state;
        plus(i) += epsilon;
        minus(i) -= epsilon;

        const double predicted_plus =
            predictLateralAccelMeasurement(plus, imu_sample, yaw_accel);
        const double predicted_minus =
            predictLateralAccelMeasurement(minus, imu_sample, yaw_accel);
        jacobian(i) = (predicted_plus - predicted_minus) / (2.0 * epsilon);
    }

    return jacobian;
}

// 1회성 시작 보정 상태기계입니다. 먼저 안정된 IMU와 정지한 휠 샘플로
// 중력을 측정하고, 실패하면 저장된 파일값으로 fallback합니다.
void particlePropagation::updateStartupGravityCalibration(const TimedImu &imu_sample) {
    if (!ekf_params_.enable_startup_gravity_calibration ||
        gravity_calibration_state_ != GravityCalibrationState::WaitingForStability) {
        return;
    }

    // 첫 IMU 샘플 시각부터 10초 보정 timeout을 시작합니다.
    const double stamp = timeToSeconds(imu_sample.msg.header.stamp);
    if (!gravity_startup_timer_started_) {
        gravity_startup_timer_started_ = true;
        gravity_startup_time_ = stamp;
    }

    pushGravityStabilitySample(imu_sample);

    // IMU와 휠이 모두 안정 상태라면 새 시작 보정값을 우선 사용합니다.
    GravityCalibrationEstimate estimate;
    if (estimateStartupGravityCalibration(&estimate)) {
        applyGravityCalibration(
            estimate,
            GravityCalibrationState::CalibratedFromStartupImu);
        saveGravityCalibration(estimate);
        return;
    }

    // 시작 안정 상태가 끝내 나오지 않으면 가장 최근 저장된 중력 벡터를
    // 불러오고, 오도메트리를 막지 않은 채 계속 진행합니다.
    if (stamp - gravity_startup_time_ >= ekf_params_.gravity_startup_timeout) {
        if (loadGravityCalibration(&estimate)) {
            applyGravityCalibration(estimate, GravityCalibrationState::LoadedFromFile);
        } else {
            gravity_calibration_state_ = GravityCalibrationState::Unavailable;
            gravity_calibration_.valid = false;
            gravity_imu_history_.clear();
            gravity_wheel_history_.clear();
        }
    }
}

// 시작 안정 상태 검출에 사용하는 IMU/휠 rolling window를 관리합니다.
void particlePropagation::pushGravityStabilitySample(const TimedImu &imu_sample) {
    const double stamp = timeToSeconds(imu_sample.msg.header.stamp);
    gravity_imu_history_.push_back(GravityStabilitySample{
        stamp,
        Eigen::Vector3d(
            imu_sample.msg.linear_acceleration.x,
            imu_sample.msg.linear_acceleration.y,
            imu_sample.msg.linear_acceleration.z),
        Eigen::Vector3d(
            imu_sample.msg.angular_velocity.x,
            imu_sample.msg.angular_velocity.y,
            imu_sample.msg.angular_velocity.z)});

    // 최근 안정성 window에 해당하는 샘플만 유지합니다.
    const double oldest_allowed = stamp - ekf_params_.gravity_stability_window;
    while (!gravity_imu_history_.empty() &&
           gravity_imu_history_.front().time < oldest_allowed) {
        gravity_imu_history_.pop_front();
    }
    while (!gravity_wheel_history_.empty() &&
           gravity_wheel_history_.front().time < oldest_allowed) {
        gravity_wheel_history_.pop_front();
    }
}

// IMU가 안정되고 휠 이동거리도 변하지 않을 때만 시작 중력을 추정합니다.
// 모든 안정 조건이 만족되기 전까지는 false를 반환합니다.
bool particlePropagation::estimateStartupGravityCalibration(
    GravityCalibrationEstimate *estimate) const {
    if (estimate == nullptr ||
        static_cast<int>(gravity_imu_history_.size()) <
            ekf_params_.gravity_min_imu_samples ||
        gravity_wheel_history_.size() < 2) {
        return false;
    }

    // 짧고 조용한 순간만으로 보정되지 않도록 충분한 시간 범위를 요구합니다.
    const double imu_duration =
        gravity_imu_history_.back().time - gravity_imu_history_.front().time;
    if (imu_duration < 0.9 * ekf_params_.gravity_stability_window) {
        return false;
    }

    Eigen::Vector3d accel_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
    // 안정성 window 내 가속도와 gyro를 평균냅니다.
    for (const auto &sample : gravity_imu_history_) {
        accel_mean += sample.accel;
        gyro_mean += sample.gyro;
    }
    accel_mean /= static_cast<double>(gravity_imu_history_.size());
    gyro_mean /= static_cast<double>(gravity_imu_history_.size());

    double accel_variance_sum = 0.0;
    double gyro_variance_sum = 0.0;
    // 벡터 표준편차를 간결한 안정성 지표로 사용합니다.
    for (const auto &sample : gravity_imu_history_) {
        accel_variance_sum += (sample.accel - accel_mean).squaredNorm();
        gyro_variance_sum += (sample.gyro - gyro_mean).squaredNorm();
    }
    const double accel_std =
        std::sqrt(accel_variance_sum / static_cast<double>(gravity_imu_history_.size()));
    const double gyro_std =
        std::sqrt(gyro_variance_sum / static_cast<double>(gravity_imu_history_.size()));
    const double accel_norm = accel_mean.norm();
    if (accel_norm < 0.5 * kGravity) {
        return false;
    }

    // 같은 window 동안 휠도 정지 상태여야 합니다.
    const double wheel_delta =
        std::abs(
            gravity_wheel_history_.back().cumulative_distance -
            gravity_wheel_history_.front().cumulative_distance);
    double wheel_velocity_square_sum = 0.0;
    int wheel_velocity_count = 0;
    // 순 이동량이 작더라도 작은 진동이 있으면 RMS 휠 속도로 잡아냅니다.
    for (size_t i = 1; i < gravity_wheel_history_.size(); ++i) {
        const double dt =
            gravity_wheel_history_[i].time - gravity_wheel_history_[i - 1].time;
        if (dt <= ekf_params_.min_dt) {
            continue;
        }
        const double ds =
            gravity_wheel_history_[i].cumulative_distance -
            gravity_wheel_history_[i - 1].cumulative_distance;
        const double velocity = ds / dt;
        wheel_velocity_square_sum += velocity * velocity;
        wheel_velocity_count++;
    }
    if (wheel_velocity_count == 0) {
        return false;
    }

    const double wheel_velocity_rms =
        std::sqrt(wheel_velocity_square_sum / static_cast<double>(wheel_velocity_count));
    const double accel_norm_error = std::abs(accel_norm - kGravity);

    // 연속 confidence score입니다. 어떤 항목 하나라도 나쁘면 곱이 작아지므로,
    // 모든 항목이 충분히 안정적일 때만 보정이 수행됩니다.
    const double score =
        gaussianConfidence(gyro_mean.norm(), ekf_params_.gravity_gyro_mean_sigma) *
        gaussianConfidence(gyro_std, ekf_params_.gravity_gyro_std_sigma) *
        gaussianConfidence(accel_std, ekf_params_.gravity_accel_std_sigma) *
        gaussianConfidence(accel_norm_error, ekf_params_.gravity_accel_norm_sigma) *
        gaussianConfidence(wheel_delta, ekf_params_.gravity_wheel_delta_sigma) *
        gaussianConfidence(wheel_velocity_rms, ekf_params_.gravity_wheel_velocity_sigma);

    if (score < ekf_params_.gravity_stability_score_min) {
        return false;
    }

    // 중력은 rear/body 좌표계에 저장합니다. accel_gravity_sign은 정지 시 +g 또는
    // -g로 보고하는 IMU 드라이버 차이를 처리합니다.
    estimate->valid = true;
    estimate->score = score;
    estimate->gravity_body =
        ekf_params_.accel_gravity_sign * kGravity * accel_mean / accel_norm;
    estimate->static_pitch = std::atan2(
        -estimate->gravity_body.x(),
        std::hypot(estimate->gravity_body.y(), estimate->gravity_body.z()));
    estimate->static_roll = std::atan2(
        estimate->gravity_body.y(),
        estimate->gravity_body.z());
    // 정지 window의 자이로 z 평균이 곧 yaw-rate bias입니다.
    estimate->gyro_z_bias = gyro_mean.z();
    estimate->sample_count = static_cast<int>(gravity_imu_history_.size());
    estimate->window_duration = imu_duration;
    estimate->accel_norm_mean = accel_norm;
    estimate->gyro_norm_mean = gyro_mean.norm();
    estimate->accel_std = accel_std;
    estimate->gyro_std = gyro_std;
    estimate->wheel_delta = wheel_delta;
    estimate->wheel_velocity_rms = wheel_velocity_rms;
    return true;
}

// 시작 중력 보정값을 작은 YAML 유사 key-value 파일로 저장합니다.
bool particlePropagation::saveGravityCalibration(
    const GravityCalibrationEstimate &estimate) const {
    if (!estimate.valid) {
        return false;
    }

    // 깨끗한 시스템에서도 필터가 동작하도록 출력 디렉터리를 필요할 때 생성합니다.
    const std::filesystem::path path(gravityCalibrationPath());
    std::error_code error;
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path(), error);
        if (error) {
            return false;
        }
    }

    std::ofstream file(path);
    if (!file.is_open()) {
        return false;
    }

    // 별도 의존성이 없도록 단순한 "key: value" 줄 형식을 유지합니다.
    file << "version: 1\n";
    file << "gravity_body_x: " << estimate.gravity_body.x() << "\n";
    file << "gravity_body_y: " << estimate.gravity_body.y() << "\n";
    file << "gravity_body_z: " << estimate.gravity_body.z() << "\n";
    file << "static_pitch: " << estimate.static_pitch << "\n";
    file << "static_roll: " << estimate.static_roll << "\n";
    file << "gyro_z_bias: " << estimate.gyro_z_bias << "\n";
    file << "score: " << estimate.score << "\n";
    file << "sample_count: " << estimate.sample_count << "\n";
    file << "window_duration: " << estimate.window_duration << "\n";
    file << "accel_norm_mean: " << estimate.accel_norm_mean << "\n";
    file << "gyro_norm_mean: " << estimate.gyro_norm_mean << "\n";
    file << "accel_std: " << estimate.accel_std << "\n";
    file << "gyro_std: " << estimate.gyro_std << "\n";
    file << "wheel_delta: " << estimate.wheel_delta << "\n";
    file << "wheel_velocity_rms: " << estimate.wheel_velocity_rms << "\n";
    return file.good();
}

// 시작 보정이 timeout되면 마지막으로 저장된 중력 보정값을 불러옵니다.
bool particlePropagation::loadGravityCalibration(
    GravityCalibrationEstimate *estimate) const {
    if (estimate == nullptr) {
        return false;
    }

    std::ifstream file(gravityCalibrationPath());
    if (!file.is_open()) {
        return false;
    }

    bool has_x = false;
    bool has_y = false;
    bool has_z = false;
    bool has_pitch = false;
    bool has_roll = false;
    GravityCalibrationEstimate loaded;
    std::string line;
    // 알고 있는 숫자 필드만 파싱합니다. 향후 호환성을 위해 모르는 key는 무시합니다.
    while (std::getline(file, line)) {
        std::string key;
        double value = 0.0;
        if (!parseKeyValue(line, &key, &value)) {
            continue;
        }

        if (key == "gravity_body_x") {
            loaded.gravity_body.x() = value;
            has_x = true;
        } else if (key == "gravity_body_y") {
            loaded.gravity_body.y() = value;
            has_y = true;
        } else if (key == "gravity_body_z") {
            loaded.gravity_body.z() = value;
            has_z = true;
        } else if (key == "static_pitch") {
            loaded.static_pitch = value;
            has_pitch = true;
        } else if (key == "static_roll") {
            loaded.static_roll = value;
        } else if (key == "gyro_z_bias") {
            loaded.gyro_z_bias = value;
            has_roll = true;
        } else if (key == "score") {
            loaded.score = value;
        } else if (key == "sample_count") {
            loaded.sample_count = static_cast<int>(value);
        } else if (key == "window_duration") {
            loaded.window_duration = value;
        } else if (key == "accel_norm_mean") {
            loaded.accel_norm_mean = value;
        } else if (key == "gyro_norm_mean") {
            loaded.gyro_norm_mean = value;
        } else if (key == "accel_std") {
            loaded.accel_std = value;
        } else if (key == "gyro_std") {
            loaded.gyro_std = value;
        } else if (key == "wheel_delta") {
            loaded.wheel_delta = value;
        } else if (key == "wheel_velocity_rms") {
            loaded.wheel_velocity_rms = value;
        }
    }

    // 파일을 받아들이기 전에 그럴듯한 중력 벡터인지 확인합니다.
    if (!has_x || !has_y || !has_z ||
        !std::isfinite(loaded.gravity_body.norm()) ||
        loaded.gravity_body.norm() < 0.5 * kGravity) {
        return false;
    }

    // 예전 파일에는 pitch/roll이 없을 수 있으므로 중력 벡터에서 유도합니다.
    if (!has_pitch) {
        loaded.static_pitch = std::atan2(
            -loaded.gravity_body.x(),
            std::hypot(loaded.gravity_body.y(), loaded.gravity_body.z()));
    }
    if (!has_roll) {
        loaded.static_roll = std::atan2(
            loaded.gravity_body.y(),
            loaded.gravity_body.z());
    }
    if (loaded.score <= 0.0) {
        loaded.score = 1.0;
    }
    loaded.valid = true;
    *estimate = loaded;
    return true;
}

// 중력 보정값을 활성화하고 시작 보정 버퍼를 비웁니다.
void particlePropagation::applyGravityCalibration(
    const GravityCalibrationEstimate &estimate,
    GravityCalibrationState state) {
    if (!estimate.valid) {
        return;
    }

    gravity_calibration_ = estimate;
    gravity_calibration_state_ = state;
    gravity_imu_history_.clear();
    gravity_wheel_history_.clear();

    // 새 시작 보정값은 정적 tilt 기준을 정의합니다. 동적 suspension tilt는
    // 이 보정 기준 주변의 0에서 시작하도록 초기화합니다.
    if (state == GravityCalibrationState::CalibratedFromStartupImu &&
        ekf_initialized_) {
        ekf_state_(kPitch) = 0.0;
        ekf_state_(kPitchRate) = 0.0;
        ekf_state_(kRoll) = 0.0;
        ekf_state_(kRollRate) = 0.0;
        // 경량 DR의 기울기 기준도 같은 순간으로 맞춥니다.
        if (dr_tilt_reference_valid_) {
            dr_tilt_reference_roll_ = last_dr_tilt_roll_;
            dr_tilt_reference_pitch_ = last_dr_tilt_pitch_;
        }
    }
    // 정지 상태에서 잰 자이로 z 평균을 yaw-rate bias로 확정합니다.
    if (ekf_initialized_) {
        ekf_state_(kGyroZBias) = estimate.gyro_z_bias;
    }
}

// IMU AHRS quaternion에서 차대 기울기를 뽑습니다.
//
// AHRS는 중력 기준 절대 자세를 주지만 우리가 원하는 것은 "노면에 대한 차대의
// 기울기"이므로, IMU 장착 기울기를 빼야 합니다. 시동 시점 자세를 기준으로
// 잡아 그 대비 증분을 씁니다(평지에서 시동한다는 전제 - 시작 중력 보정과
// 같은 전제입니다). 기준은 중력 보정이 확정될 때 다시 맞춥니다.
bool particlePropagation::chassisTiltFromImu(
    const TimedImu &imu_sample, double &roll, double &pitch) {
    last_dr_tilt_valid_ = false;
    if (!ekf_params_.dr_use_imu_tilt) {
        return false;
    }

    const auto &q = imu_sample.msg.orientation;
    const double norm_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    // 자세를 주지 않는 IMU는 quaternion을 0으로 채워 보냅니다.
    if (!std::isfinite(norm_sq) || norm_sq < 0.5) {
        return false;
    }
    const double inv = 1.0 / std::sqrt(norm_sq);
    const double qw = q.w * inv;
    const double qx = q.x * inv;
    const double qy = q.y * inv;
    const double qz = q.z * inv;

    const double abs_roll = std::atan2(
        2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    const double abs_pitch =
        std::asin(std::clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0));
    if (!std::isfinite(abs_roll) || !std::isfinite(abs_pitch)) {
        return false;
    }

    last_dr_tilt_roll_ = abs_roll;
    last_dr_tilt_pitch_ = abs_pitch;
    if (!dr_tilt_reference_valid_) {
        dr_tilt_reference_roll_ = abs_roll;
        dr_tilt_reference_pitch_ = abs_pitch;
        dr_tilt_reference_valid_ = true;
        return false;
    }

    const double rel_roll = normalizeAngle(abs_roll - dr_tilt_reference_roll_);
    const double rel_pitch = normalizeAngle(abs_pitch - dr_tilt_reference_pitch_);
    const double limit = ekf_params_.dr_tilt_max_deg * kDegToRad;
    if (std::abs(rel_roll) > limit || std::abs(rel_pitch) > limit) {
        return false;
    }

    roll = rel_roll;
    pitch = rel_pitch;
    last_dr_tilt_valid_ = true;
    return true;
}

void particlePropagation::relativeTilt(double &roll_deg, double &pitch_deg) const {
    if (!dr_tilt_reference_valid_) {
        roll_deg = 0.0;
        pitch_deg = 0.0;
        return;
    }
    roll_deg =
        normalizeAngle(last_dr_tilt_roll_ - dr_tilt_reference_roll_) / kDegToRad;
    pitch_deg =
        normalizeAngle(last_dr_tilt_pitch_ - dr_tilt_reference_pitch_) / kDegToRad;
}

double particlePropagation::relativeTiltDeg() const {
    if (!dr_tilt_reference_valid_) {
        return 0.0;
    }
    const double rel_roll =
        normalizeAngle(last_dr_tilt_roll_ - dr_tilt_reference_roll_);
    const double rel_pitch =
        normalizeAngle(last_dr_tilt_pitch_ - dr_tilt_reference_pitch_);
    return std::max(std::abs(rel_roll), std::abs(rel_pitch)) / kDegToRad;
}

// 정적 시작 중력값과 동적 pitch/roll 중력 성분을 합칩니다.
Eigen::Vector3d particlePropagation::calcGravityBody(
    const StateVector &state) const {
    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
    if (gravity_calibration_.valid) {
        gravity = gravity_calibration_.gravity_body;
    }

    gravity.x() += ekf_params_.gravity_pitch_sign * kGravity * state(kPitch);
    gravity.y() += ekf_params_.gravity_roll_sign * kGravity * state(kRoll);
    return gravity;
}

// 호출부가 명시적인 경로를 주지 않았을 때 사용할 보정 파일 기본 위치입니다.
std::string particlePropagation::gravityCalibrationPath() const {
    if (!ekf_params_.gravity_calibration_path.empty()) {
        return ekf_params_.gravity_calibration_path;
    }

    const char *home = std::getenv("HOME");
    if (home != nullptr && std::string(home).size() > 0) {
        return (std::filesystem::path(home) /
                ".ros" /
                "localization_pf" /
                "imu_gravity_calibration.yaml")
            .string();
    }

    return "imu_gravity_calibration.yaml";
}

// target_time의 IMU 샘플을 만듭니다. 앞뒤 샘플이 있으면 보간하고,
// 아주 최신 시각이면 제한된 hold 외삽을 사용합니다.
bool particlePropagation::interpolateImu(
    double target_time,
    TimedImu *imu_sample) const {
    if (imu_sample == nullptr || imu_history_.empty() || !std::isfinite(target_time)) {
        return false;
    }

    // geometry_msgs::Vector3 계열 필드 보간용 보조 함수입니다.
    const auto interpolate_vector3 = [](const auto &lhs, const auto &rhs, double alpha) {
        auto result = lhs;
        result.x = lhs.x + (rhs.x - lhs.x) * alpha;
        result.y = lhs.y + (rhs.y - lhs.y) * alpha;
        result.z = lhs.z + (rhs.z - lhs.z) * alpha;
        return result;
    };

    // 샘플이 하나뿐이면 짧은 외삽 window 안에서만 사용합니다.
    if (imu_history_.size() == 1) {
        const TimedImu &only = imu_history_.front();
        const double only_time = timeToSeconds(only.msg.header.stamp);
        if (std::abs(target_time - only_time) > ekf_params_.max_imu_extrapolation) {
            return false;
        }
        *imu_sample = only;
        imu_sample->msg.header.stamp = secondsToTime(target_time);
        return true;
    }

    // target이 보관 중인 첫 샘플보다 아주 조금 앞이면 가장 이른 값을 유지합니다.
    // 더 크게 벗어난 요청은 너무 오래된 것으로 보고 거부합니다.
    const double first_time = timeToSeconds(imu_history_.front().msg.header.stamp);
    if (target_time < first_time) {
        if (first_time - target_time > ekf_params_.max_imu_extrapolation) {
            return false;
        }
        *imu_sample = imu_history_.front();
        imu_sample->msg.header.stamp = secondsToTime(target_time);
        return true;
    }

    // target_time을 사이에 두는 두 IMU 샘플을 찾습니다.
    for (size_t i = 1; i < imu_history_.size(); ++i) {
        const TimedImu &prev = imu_history_[i - 1];
        const TimedImu &next = imu_history_[i];
        const double prev_time = timeToSeconds(prev.msg.header.stamp);
        const double next_time = timeToSeconds(next.msg.header.stamp);
        if (target_time < prev_time || target_time > next_time) {
            continue;
        }

        // timestamp가 사실상 같은 샘플 쌍이면 더 최신 샘플을 우선합니다.
        if (next_time - prev_time <= ekf_params_.min_dt) {
            *imu_sample = next;
            imu_sample->msg.header.stamp = secondsToTime(target_time);
            return true;
        }

        // 가속도, gyro, IMU extrinsic을 선형 보간합니다.
        const double alpha =
            std::clamp((target_time - prev_time) / (next_time - prev_time), 0.0, 1.0);
        *imu_sample = next;
        imu_sample->msg.header.stamp = secondsToTime(target_time);
        imu_sample->msg.linear_acceleration =
            interpolate_vector3(prev.msg.linear_acceleration, next.msg.linear_acceleration, alpha);
        imu_sample->msg.angular_velocity =
            interpolate_vector3(prev.msg.angular_velocity, next.msg.angular_velocity, alpha);
        imu_sample->extrinsic.rear_to_imu_x =
            prev.extrinsic.rear_to_imu_x +
            (next.extrinsic.rear_to_imu_x - prev.extrinsic.rear_to_imu_x) * alpha;
        imu_sample->extrinsic.rear_to_imu_y =
            prev.extrinsic.rear_to_imu_y +
            (next.extrinsic.rear_to_imu_y - prev.extrinsic.rear_to_imu_y) * alpha;
        imu_sample->extrinsic.rear_to_imu_z =
            prev.extrinsic.rear_to_imu_z +
            (next.extrinsic.rear_to_imu_z - prev.extrinsic.rear_to_imu_z) * alpha;
        return true;
    }

    // target이 히스토리보다 최신인 경우입니다. 필터에 blocking delay를 추가하지
    // 않도록 아주 짧은 구간에서만 최신 IMU 값을 유지합니다.
    const TimedImu &latest = imu_history_.back();
    const double latest_time = timeToSeconds(latest.msg.header.stamp);
    if (target_time - latest_time > ekf_params_.max_imu_extrapolation) {
        return false;
    }

    *imu_sample = latest;
    imu_sample->msg.header.stamp = secondsToTime(target_time);
    return true;
}

// target_time의 누적 휠 이동거리를 만듭니다. 휠 입력이 누적값이므로,
// 보간은 거리 공간에서 수행합니다.
bool particlePropagation::interpolateWheelDistance(
    double target_time,
    double *cumulative_distance) const {
    if (cumulative_distance == nullptr ||
        wheel_history_.empty() ||
        !std::isfinite(target_time)) {
        return false;
    }

    // 샘플이 하나뿐이면 해당 timestamp 근처의 기준 거리로만 사용할 수 있습니다.
    if (wheel_history_.size() == 1) {
        const TimedWheel &only = wheel_history_.front();
        if (std::abs(target_time - only.time) > ekf_params_.max_wheel_extrapolation) {
            return false;
        }
        *cumulative_distance = only.cumulative_distance;
        return true;
    }

    // 보관 중인 첫 휠 샘플보다 아주 조금 앞선 시각은 hold를 허용합니다.
    if (target_time < wheel_history_.front().time) {
        if (wheel_history_.front().time - target_time > ekf_params_.max_wheel_extrapolation) {
            return false;
        }
        *cumulative_distance = wheel_history_.front().cumulative_distance;
        return true;
    }

    // target_time을 사이에 두는 휠 샘플 사이에서 누적거리를 보간합니다.
    for (size_t i = 1; i < wheel_history_.size(); ++i) {
        const TimedWheel &prev = wheel_history_[i - 1];
        const TimedWheel &next = wheel_history_[i];
        if (target_time < prev.time || target_time > next.time) {
            continue;
        }

        // timestamp가 사실상 같은 샘플 쌍이면 더 최신 누적거리를 우선합니다.
        if (next.time - prev.time <= ekf_params_.min_dt) {
            *cumulative_distance = next.cumulative_distance;
            return true;
        }

        // 거리 선형 보간은 짧은 샘플 구간에서 휠 속도가 일정하다고 보는 것과 같습니다.
        const double alpha =
            std::clamp((target_time - prev.time) / (next.time - prev.time), 0.0, 1.0);
        *cumulative_distance =
            prev.cumulative_distance +
            (next.cumulative_distance - prev.cumulative_distance) * alpha;
        return true;
    }

    // target이 최신 휠 샘플보다 뒤에 있는 경우입니다. 가장 최근 휠 속도만 사용해,
    // 제한된 저지연 구간에서만 외삽합니다.
    const TimedWheel &latest = wheel_history_.back();
    const TimedWheel &prev = wheel_history_[wheel_history_.size() - 2];
    if (target_time - latest.time > ekf_params_.max_wheel_extrapolation) {
        return false;
    }

    const double wheel_dt = latest.time - prev.time;
    if (wheel_dt <= ekf_params_.min_dt) {
        *cumulative_distance = latest.cumulative_distance;
        return true;
    }

    const double latest_velocity =
        (latest.cumulative_distance - prev.cumulative_distance) / wheel_dt;
    *cumulative_distance =
        latest.cumulative_distance + latest_velocity * (target_time - latest.time);
    return true;
}

// 보간된 누적거리로부터 target_time의 휠 속도 측정값을 적용합니다.
// 측정 모델은 z_v = rear_velocity 입니다.
void particlePropagation::applyWheelMeasurementAtTime(
    double target_time,
    const builtin_interfaces::msg::Time &stamp,
    double cumulative_distance,
    double wheel_variance) {
    if (!ekf_initialized_ || !std::isfinite(cumulative_distance)) {
        return;
    }

    // 첫 휠 값은 누적거리 원점만 설정합니다.
    if (!has_previous_wheel_) {
        has_previous_wheel_ = true;
        previous_wheel_distance_ = cumulative_distance;
        previous_wheel_time_ = stamp;
        return;
    }

    const double previous_time = timeToSeconds(previous_wheel_time_);
    if (target_time <= previous_time + ekf_params_.min_dt) {
        return;
    }

    // 이전 휠 업데이트 이후 target_time까지의 거리 차이를 평균 속도로 변환합니다.
    const double wheel_dt = target_time - previous_time;
    const double wheel_delta = cumulative_distance - previous_wheel_distance_;
    previous_wheel_distance_ = cumulative_distance;
    previous_wheel_time_ = stamp;

    const double measured_velocity = wheel_delta / wheel_dt;
    if (!std::isfinite(measured_velocity) ||
        std::abs(measured_velocity) > ekf_params_.max_wheel_speed) {
        return;
    }

    // 스칼라 EKF 업데이트: residual = 측정 뒤차축 속도 - 추정 속도.
    StateVector jacobian = StateVector::Zero();
    jacobian(kVelocity) = 1.0;

    double velocity_residual = measured_velocity - ekf_state_(kVelocity);
    wheel_innov_sq_sum_ += square(velocity_residual);
    ++wheel_innov_count_;
    if (std::abs(ekf_state_(kVelocity)) < ekf_params_.velocity_deadzone &&
        std::abs(measured_velocity) < ekf_params_.velocity_deadzone) {
        velocity_residual = 0.0;
    }

    applyScalarUpdate(
        velocity_residual,
        jacobian,
        positiveVariance(wheel_variance));
}

// 보간/외삽에 필요한 최소 두 샘플은 남기면서 센서 히스토리 메모리를 제한합니다.
void particlePropagation::pruneSensorHistory(double newest_time) {
    if (!std::isfinite(newest_time)) {
        return;
    }

    const double oldest_allowed = newest_time - ekf_params_.history_keep_time;
    while (imu_history_.size() > 2 &&
           timeToSeconds(imu_history_.front().msg.header.stamp) < oldest_allowed) {
        imu_history_.pop_front();
    }
    while (wheel_history_.size() > 2 &&
           wheel_history_.front().time < oldest_allowed) {
        wheel_history_.pop_front();
    }
    // 궤적은 PF가 직전 scan 시각을 되돌아볼 수 있을 만큼은 남겨 둡니다.
    while (trajectory_.size() > 2 &&
           trajectory_.front().time < oldest_allowed) {
        trajectory_.pop_front();
    }
}

// 최근 yaw-rate 샘플에 작은 Savitzky-Golay 유사 최소제곱 다항 미분을 적용해
// yaw 가속도를 추정합니다.
double particlePropagation::updateYawRateDerivative(double stamp, double yaw_rate) {
    if (!yaw_rate_history_.empty() &&
        stamp <= yaw_rate_history_.back().time + ekf_params_.min_dt) {
        return last_yaw_accel_;
    }

    // yaw-rate fitting window를 유지합니다.
    yaw_rate_history_.push_back(YawRateSample{stamp, yaw_rate});
    while (static_cast<int32_t>(yaw_rate_history_.size()) >
           ekf_params_.yaw_rate_filter_window) {
        yaw_rate_history_.pop_front();
    }

    if (yaw_rate_history_.size() < 2) {
        return 0.0;
    }

    // window가 짧거나 fitting 조건이 나쁠 때 사용할 fallback 미분값입니다.
    const YawRateSample &last = yaw_rate_history_.back();
    const YawRateSample &prev = yaw_rate_history_[yaw_rate_history_.size() - 2];
    const double finite_dt = last.time - prev.time;
    const double finite_difference =
        finite_dt > ekf_params_.min_dt
            ? (last.yaw_rate - prev.yaw_rate) / finite_dt
            : 0.0;

    if (yaw_rate_history_.size() < 3) {
        return finite_difference;
    }

    // 최신 timestamp 주변에서 yaw_rate(tau) = c0 + c1*tau + ... 를 fitting합니다.
    // tau=0에서의 미분값은 c1입니다.
    const int32_t sample_count = static_cast<int32_t>(yaw_rate_history_.size());
    const int32_t order = std::min(
        ekf_params_.yaw_rate_filter_order,
        sample_count - 1);
    Eigen::MatrixXd design(sample_count, order + 1);
    Eigen::VectorXd target(sample_count);
    const double t0 = yaw_rate_history_.back().time;

    for (int32_t row = 0; row < sample_count; ++row) {
        const double tau = yaw_rate_history_[row].time - t0;
        double basis = 1.0;
        for (int32_t col = 0; col <= order; ++col) {
            design(row, col) = basis;
            basis *= tau;
        }
        target(row) = yaw_rate_history_[row].yaw_rate;
    }

    const Eigen::MatrixXd normal_matrix = design.transpose() * design;
    const Eigen::VectorXd rhs = design.transpose() * target;
    const Eigen::VectorXd coefficients =
        normal_matrix.ldlt().solve(rhs);

    if (coefficients.size() < 2 || !std::isfinite(coefficients(1))) {
        return finite_difference;
    }
    return coefficients(1);
}

// 내부 EKF 상태를 publish하지 않고 Odometry 메시지로 변환합니다.
nav_msgs::msg::Odometry particlePropagation::buildOdomMsg(
    const builtin_interfaces::msg::Time &stamp) const {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    if (!ekf_initialized_) {
        return odom;
    }

    // pose는 odom 프레임에서의 뒤차축/base pose 추정값입니다.
    odom.pose.pose.position.x = ekf_state_(kX);
    odom.pose.pose.position.y = ekf_state_(kY);
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion orientation;
    orientation.setRPY(ekf_state_(kRoll), ekf_state_(kPitch), ekf_state_(kYaw));
    orientation.normalize();
    odom.pose.pose.orientation.x = orientation.x();
    odom.pose.pose.orientation.y = orientation.y();
    odom.pose.pose.orientation.z = orientation.z();
    odom.pose.pose.orientation.w = orientation.w();

    // twist에는 뒤차축 종방향 속도와 각속도 추정값을 담습니다.
    odom.twist.twist.linear.x = ekf_state_(kVelocity);
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.x = ekf_state_(kRollRate);
    odom.twist.twist.angular.y = ekf_state_(kPitchRate);
    odom.twist.twist.angular.z = last_yaw_rate_;

    // EKF 공분산 중 관련 항목을 ROS pose/twist 공분산 배열에 매핑합니다.
    odom.pose.covariance[0] = ekf_covariance_(kX, kX);
    odom.pose.covariance[7] = ekf_covariance_(kY, kY);
    odom.pose.covariance[14] = square(0.01);
    odom.pose.covariance[21] = ekf_covariance_(kRoll, kRoll);
    odom.pose.covariance[28] = ekf_covariance_(kPitch, kPitch);
    odom.pose.covariance[35] = ekf_covariance_(kYaw, kYaw);

    odom.twist.covariance[0] = ekf_covariance_(kVelocity, kVelocity);
    odom.twist.covariance[7] = square(0.01);
    odom.twist.covariance[14] = square(0.01);
    odom.twist.covariance[21] = ekf_covariance_(kRollRate, kRollRate);
    odom.twist.covariance[28] = ekf_covariance_(kPitchRate, kPitchRate);
    odom.twist.covariance[35] = square(0.05);

    return odom;
}
