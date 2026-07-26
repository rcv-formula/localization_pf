#include "scan_deskew.h"

#include <algorithm>
#include <cmath>
#include <limits>

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

double stampSeconds(const builtin_interfaces::msg::Time &stamp) {
    return static_cast<double>(stamp.sec) +
        static_cast<double>(stamp.nanosec) * 1.0e-9;
}

}  // namespace

ScanDeskew::ScanDeskew(const Parameters &parameters)
    : parameters_(parameters) {}

void ScanDeskew::setParameters(const Parameters &parameters) {
    parameters_ = parameters;
}

double ScanDeskew::beamTime(
    const sensor_msgs::msg::LaserScan &scan,
    std::size_t index,
    double start_seconds,
    double increment) const {
    (void)scan;
    return start_seconds + static_cast<double>(index) * increment;
}

bool ScanDeskew::deskew(
    const sensor_msgs::msg::LaserScan &scan,
    const PoseLookup &pose_at,
    Result &result) const {
    result = Result{};
    const std::size_t beam_count = scan.ranges.size();
    if (beam_count == 0) {
        return false;
    }

    const double start_seconds = stampSeconds(scan.header.stamp);

    // 빔 간 시간 간격을 정합니다. 센서가 time_increment를 채우지 않으면
    // 파라미터의 대체 스윕 길이를 사용합니다.
    double increment = static_cast<double>(scan.time_increment);
    if (!std::isfinite(increment) || increment <= 0.0) {
        increment = parameters_.fallback_sweep_duration > 0.0 && beam_count > 1 ?
            parameters_.fallback_sweep_duration / static_cast<double>(beam_count - 1) :
            0.0;
    }

    const double end_seconds =
        start_seconds + increment * static_cast<double>(beam_count - 1);
    const double reference_time =
        parameters_.reference == Reference::SweepEnd ? end_seconds : start_seconds;
    result.reference_time = reference_time;

    result.xs.assign(beam_count, 0.0f);
    result.ys.assign(beam_count, 0.0f);
    result.ranges.assign(beam_count, std::numeric_limits<float>::quiet_NaN());
    result.bearings.assign(beam_count, 0.0f);

    // 보정을 끄거나 빔 시각을 알 수 없으면 원본을 그대로 담아 돌려줍니다.
    const bool passthrough = !parameters_.enabled || increment <= 0.0;

    double reference_x = 0.0, reference_y = 0.0, reference_yaw = 0.0;
    bool reference_ok = false;
    if (!passthrough && pose_at) {
        reference_ok = pose_at(reference_time, reference_x, reference_y, reference_yaw);
    }
    if (!passthrough && !reference_ok) {
        // 기준 시각 pose가 없으면 보정 자체가 불가능합니다.
        return false;
    }

    const double reference_cos = std::cos(reference_yaw);
    const double reference_sin = std::sin(reference_yaw);
    std::size_t failures = 0;

    for (std::size_t index = 0; index < beam_count; ++index) {
        const double range = static_cast<double>(scan.ranges[index]);
        const double bearing =
            static_cast<double>(scan.angle_min) +
            static_cast<double>(index) * static_cast<double>(scan.angle_increment);

        const bool usable = std::isfinite(range) &&
            range >= static_cast<double>(scan.range_min) &&
            range <= static_cast<double>(scan.range_max) &&
            range > 0.0;
        if (!usable) {
            result.ranges[index] = scan.ranges[index];
            result.bearings[index] = static_cast<float>(bearing);
            continue;
        }

        // 빔 자신의 시각 pose가 있어야 재투영할 수 있습니다.
        double beam_x = 0.0, beam_y = 0.0, beam_yaw = 0.0;
        bool beam_ok = false;
        if (!passthrough && pose_at) {
            beam_ok = pose_at(
                beamTime(scan, index, start_seconds, increment),
                beam_x, beam_y, beam_yaw);
        }

        if (passthrough || !beam_ok) {
            // 보정 불가 빔은 원본 좌표를 그대로 둡니다.
            if (!passthrough) {
                ++failures;
                ++result.uncorrected_count;
            }
            const double local_x = range * std::cos(bearing);
            const double local_y = range * std::sin(bearing);
            result.xs[index] = static_cast<float>(local_x);
            result.ys[index] = static_cast<float>(local_y);
            result.ranges[index] = static_cast<float>(range);
            result.bearings[index] = static_cast<float>(bearing);
            continue;
        }

        // 빔 시각의 센서 프레임 -> 고정 프레임 -> 기준 시각의 센서 프레임.
        const double local_x = range * std::cos(bearing);
        const double local_y = range * std::sin(bearing);
        const double beam_cos = std::cos(beam_yaw);
        const double beam_sin = std::sin(beam_yaw);
        const double world_x = beam_x + beam_cos * local_x - beam_sin * local_y;
        const double world_y = beam_y + beam_sin * local_x + beam_cos * local_y;

        const double offset_x = world_x - reference_x;
        const double offset_y = world_y - reference_y;
        const double corrected_x = reference_cos * offset_x + reference_sin * offset_y;
        const double corrected_y = -reference_sin * offset_x + reference_cos * offset_y;

        result.xs[index] = static_cast<float>(corrected_x);
        result.ys[index] = static_cast<float>(corrected_y);
        result.ranges[index] = static_cast<float>(std::hypot(corrected_x, corrected_y));
        result.bearings[index] =
            static_cast<float>(std::atan2(corrected_y, corrected_x));
    }

    // 조회 실패가 많으면 결과를 신뢰할 수 없으므로 실패로 처리합니다.
    if (!passthrough && beam_count > 0) {
        const double failure_ratio =
            static_cast<double>(failures) / static_cast<double>(beam_count);
        if (failure_ratio > parameters_.max_lookup_failure_ratio) {
            result.valid = false;
            return false;
        }
    }

    result.valid = true;
    return true;
}

bool ScanDeskew::toLaserScan(
    const Result &result,
    const sensor_msgs::msg::LaserScan &reference_scan,
    sensor_msgs::msg::LaserScan &out) {
    if (!result.valid || result.ranges.empty()) {
        return false;
    }

    out = reference_scan;
    const double angle_min = static_cast<double>(reference_scan.angle_min);
    const double angle_increment =
        static_cast<double>(reference_scan.angle_increment);
    if (!std::isfinite(angle_increment) || angle_increment == 0.0) {
        return false;
    }

    const std::size_t bin_count = reference_scan.ranges.size();
    out.ranges.assign(bin_count, std::numeric_limits<float>::infinity());

    // 재투영 후 방위가 비균일해지므로 원본 각도 격자에 다시 담습니다.
    // 같은 칸에 여러 점이 오면 더 가까운 값을 남깁니다.
    for (std::size_t index = 0; index < result.ranges.size(); ++index) {
        const float range = result.ranges[index];
        if (!std::isfinite(range) || range <= 0.0f) {
            continue;
        }
        const double bearing = static_cast<double>(result.bearings[index]);
        const long long bin = std::llround((bearing - angle_min) / angle_increment);
        if (bin < 0 || static_cast<std::size_t>(bin) >= bin_count) {
            continue;
        }
        float &slot = out.ranges[static_cast<std::size_t>(bin)];
        if (!std::isfinite(slot) || range < slot) {
            slot = range;
        }
    }
    return true;
}
