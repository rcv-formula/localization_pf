#include "pose_estimation.h"

#include <algorithm>
#include <cmath>

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

}  // namespace

PoseEstimation::PoseEstimation(const Parameters &parameters) {
    setParameters(parameters);
}

void PoseEstimation::setParameters(const Parameters &parameters) {
    parameters_ = parameters;
    parameters_.scan_trust = std::clamp(parameters_.scan_trust, 0.0, 1.0);
    parameters_.yaw_scan_trust = std::clamp(parameters_.yaw_scan_trust, 0.0, 1.0);
    parameters_.odom_position_growth = std::max(0.0, parameters_.odom_position_growth);
    parameters_.odom_yaw_growth = std::max(0.0, parameters_.odom_yaw_growth);
    parameters_.odom_yaw_growth_per_meter =
        std::max(0.0, parameters_.odom_yaw_growth_per_meter);
    parameters_.min_variance = std::max(1.0e-12, parameters_.min_variance);
}

void PoseEstimation::reset(double x, double y, double yaw) {
    reset(x, y, yaw, Eigen::Matrix2d::Identity() * 0.25, 0.25);
}

void PoseEstimation::reset(
    double x, double y, double yaw,
    const Eigen::Matrix2d &position_covariance,
    double yaw_variance) {
    x_ = x;
    y_ = y;
    yaw_ = normalizeAngle(yaw);
    position_covariance_ = position_covariance;
    yaw_variance_ = std::max(parameters_.min_variance, yaw_variance);
    initialized_ = true;

    last_ = Result{};
    last_.valid = true;
    last_.x = x_;
    last_.y = y_;
    last_.yaw = yaw_;
    last_.position_covariance = position_covariance_;
    last_.yaw_variance = yaw_variance_;
}

void PoseEstimation::invalidate() {
    initialized_ = false;
    last_ = Result{};
}

PoseEstimation::Result PoseEstimation::update(
    const ScanEstimate &scan_estimate,
    const OdomDelta &odom_delta,
    const Observability &observability) {
    // 아직 기준이 없으면 스캔 추정으로 시작합니다.
    if (!initialized_) {
        if (!scan_estimate.valid) {
            return last_;
        }
        reset(
            scan_estimate.x, scan_estimate.y, scan_estimate.yaw,
            scan_estimate.position_covariance,
            scan_estimate.yaw_variance);
        return last_;
    }

    // ---- 예측: 직전 융합 pose에 body frame 이동량을 적용합니다. ----
    double predicted_x = x_;
    double predicted_y = y_;
    double predicted_yaw = yaw_;
    Eigen::Matrix2d predicted_covariance = position_covariance_;
    double predicted_yaw_variance = yaw_variance_;

    double travel = 0.0;
    if (odom_delta.valid) {
        const double cos_yaw = std::cos(yaw_);
        const double sin_yaw = std::sin(yaw_);
        predicted_x += cos_yaw * odom_delta.longitudinal - sin_yaw * odom_delta.lateral;
        predicted_y += sin_yaw * odom_delta.longitudinal + cos_yaw * odom_delta.lateral;
        predicted_yaw = normalizeAngle(yaw_ + odom_delta.yaw);
        travel = std::hypot(odom_delta.longitudinal, odom_delta.lateral);

        // 이동량에 비례해 예측 불확실성이 커집니다(정지 중에는 커지지 않음).
        const double position_growth = parameters_.odom_position_growth * travel;
        predicted_covariance(0, 0) += position_growth * position_growth;
        predicted_covariance(1, 1) += position_growth * position_growth;
        const double yaw_growth =
            parameters_.odom_yaw_growth * std::abs(odom_delta.yaw) +
            parameters_.odom_yaw_growth_per_meter * travel;
        predicted_yaw_variance += yaw_growth * yaw_growth;
    }

    // 스캔 추정이 없으면 예측만으로 진행합니다.
    if (!scan_estimate.valid) {
        x_ = predicted_x;
        y_ = predicted_y;
        yaw_ = predicted_yaw;
        position_covariance_ = predicted_covariance;
        yaw_variance_ = predicted_yaw_variance;

        last_ = Result{};
        last_.valid = true;
        last_.x = x_;
        last_.y = y_;
        last_.yaw = yaw_;
        last_.position_covariance = position_covariance_;
        last_.yaw_variance = yaw_variance_;
        last_.scan_trust = Eigen::Vector2d::Zero();
        last_.yaw_scan_trust = 0.0;
        return last_;
    }

    // ---- 방향 기저와 축별 신뢰도 ----
    // 관측성 조회 자체가 실패하면(맵 미준비, 맵 밖) 등방으로 물러서되
    // 스캔은 믿지 않습니다. 예전에는 이 경로가 등방 + 신뢰도 1.0이라
    // 관측성이 가장 나쁜 곳에서 스캔을 가장 믿는 fail-open이었습니다.
    Eigen::Matrix2d basis = Eigen::Matrix2d::Identity();
    Eigen::Vector2d observability_confidence = Eigen::Vector2d::Zero();
    if (observability.valid) {
        basis = observability.eigenvectors;
        observability_confidence(0) =
            std::clamp(observability.confidence(0), 0.0, 1.0);
        observability_confidence(1) =
            std::clamp(observability.confidence(1), 0.0, 1.0);
    }

    // ---- 축별 스캔 가중 alpha ----
    // 형태 (라): alpha는 오직 기하 관측성에서만 나옵니다. 공분산은 아래에서
    // 보고용으로만 갱신되고 alpha에 되먹임되지 않습니다. 이 되먹임이 있으면
    // 융합 공분산이 0으로 붕괴하면서 alpha도 함께 0으로 가라앉습니다.
    Eigen::Vector2d alpha = Eigen::Vector2d::Zero();
    Eigen::Vector2d fused_variance = Eigen::Vector2d::Zero();
    for (int axis = 0; axis < 2; ++axis) {
        alpha(axis) = std::clamp(
            parameters_.scan_trust * observability_confidence(axis), 0.0, 1.0);

        const Eigen::Vector2d direction = basis.col(axis);
        const double scan_variance = std::max(
            parameters_.min_variance,
            direction.dot(scan_estimate.position_covariance * direction));
        const double odom_variance = std::max(
            parameters_.min_variance,
            direction.dot(predicted_covariance * direction));

        // x = (1-a) * x_pred + a * x_scan 의 공분산입니다. a < 1 이면 예측
        // 성분이 남으므로 0으로 붕괴하지 않고, a = 0 이면 dead-reckoning
        // 불확실성이 그대로 자라납니다.
        const double weight = 1.0 - alpha(axis);
        fused_variance(axis) =
            weight * weight * odom_variance +
            alpha(axis) * alpha(axis) * scan_variance;
    }

    // ---- 융합: 예측에서 스캔 쪽으로 방향별 alpha 만큼만 이동합니다. ----
    const Eigen::Vector2d innovation(
        scan_estimate.x - predicted_x,
        scan_estimate.y - predicted_y);
    const Eigen::Vector2d innovation_in_basis = basis.transpose() * innovation;
    const Eigen::Vector2d correction_in_basis(
        alpha(0) * innovation_in_basis(0),
        alpha(1) * innovation_in_basis(1));
    const Eigen::Vector2d correction = basis * correction_in_basis;

    x_ = predicted_x + correction(0);
    y_ = predicted_y + correction(1);
    position_covariance_ =
        basis * fused_variance.asDiagonal() * basis.transpose();

    // yaw는 복도에서도 벽이 방향을 잘 구속하므로 기하 게이팅 없이 섞습니다.
    // 주의: 이 가정은 아직 검증되지 않았습니다. 원형 홀이나 회전대칭 구역에서는
    // 거짓이고, 현재 2x2 병진 정보행렬로는 yaw 관측성을 볼 수단 자체가 없습니다.
    // 3x3 point-to-plane Hessian + Schur 주변화를 넣을 때 함께 게이팅할 것.
    const double yaw_alpha = std::clamp(parameters_.yaw_scan_trust, 0.0, 1.0);
    yaw_ = normalizeAngle(
        predicted_yaw +
        yaw_alpha * normalizeAngle(scan_estimate.yaw - predicted_yaw));
    const double yaw_weight = 1.0 - yaw_alpha;
    yaw_variance_ =
        yaw_weight * yaw_weight * predicted_yaw_variance +
        yaw_alpha * yaw_alpha *
        std::max(parameters_.min_variance, scan_estimate.yaw_variance);

    last_ = Result{};
    last_.valid = true;
    last_.x = x_;
    last_.y = y_;
    last_.yaw = yaw_;
    last_.position_covariance = position_covariance_;
    last_.yaw_variance = yaw_variance_;
    last_.scan_trust = alpha;
    last_.eigenvectors = basis;
    last_.yaw_scan_trust = yaw_alpha;
    return last_;
}

void PoseEstimation::annotateParticles(
    particle *particles,
    int32_t particle_count,
    const Result &result) {
    if (particles == nullptr || particle_count <= 0 || !result.valid) {
        return;
    }
    // eigenvectors는 내림차순이므로 col(0)이 강축, col(1)이 약축(퇴화 방향)입니다.
    const Eigen::Vector2d strong_axis = result.eigenvectors.col(0);
    const Eigen::Vector2d weak_axis = result.eigenvectors.col(1);
    for (int32_t index = 0; index < particle_count; ++index) {
        particle &current = particles[index];
        const Eigen::Vector2d offset(
            static_cast<double>(current.x) - result.x,
            static_cast<double>(current.y) - result.y);
        current.score[3] = static_cast<float>(offset.dot(weak_axis));
        current.score[4] = static_cast<float>(offset.dot(strong_axis));
    }
}
