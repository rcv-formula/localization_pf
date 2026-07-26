#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"

// 주행 중 스윕되는 LiDAR의 모션 왜곡을 보정합니다.
//
// 40Hz 스캔이면 한 주기 25ms 동안 15m/s 플랫폼은 0.375m를 이동하므로, 스윕
// 시작 빔과 끝 빔이 서로 다른 좌표계에서 찍힙니다. 이 클래스는 빔마다 그
// 순간의 pose를 조회해 하나의 기준 시각 좌표계로 재투영합니다.
//
// 오도메트리 구현에 의존하지 않도록 pose 조회를 std::function으로 주입받습니다.
// particlePropagation::poseAt()을 감싸 넘겨도 되고, 이후 다른 소스(외부 odom,
// LIO 등)로 교체해도 이 파일은 그대로 씁니다.
class ScanDeskew {
public:
    // 조회 시각[s]에 대한 pose를 map/odom 등 하나의 고정 프레임에서 돌려줍니다.
    // 조회 불가면 false를 반환해야 합니다.
    using PoseLookup = std::function<bool(
        double time_seconds, double &x, double &y, double &yaw)>;

    enum class Reference {
        SweepStart,  // scan.header.stamp 기준으로 정렬
        SweepEnd     // 스윕 마지막 빔 시각 기준으로 정렬(가장 최신 상태)
    };

    struct Parameters {
        bool enabled{true};
        // 정렬 기준 시각입니다. PF는 가장 최신 시각 기준이 유리합니다.
        Reference reference{Reference::SweepEnd};
        // scan.time_increment가 0으로 오는 센서를 위한 대체 스윕 길이[s]입니다.
        // 0 이하이면 대체를 쓰지 않습니다.
        double fallback_sweep_duration{0.0};
        // 빔 pose 조회가 이 비율 이상 실패하면 보정을 포기합니다.
        double max_lookup_failure_ratio{0.2};
    };

    // 보정 결과입니다. 재투영하면 방위가 더 이상 균일하지 않으므로, 점 좌표를
    // 1급 출력으로 두고 LaserScan 변환은 별도 헬퍼로 제공합니다.
    struct Result {
        bool valid{false};
        // 이 시각의 센서 프레임 기준으로 정렬되어 있습니다. PF는 파티클을
        // 반드시 이 시각으로 전파해야 합니다.
        double reference_time{0.0};
        // reference_time 센서 프레임에서의 점 좌표입니다.
        std::vector<float> xs;
        std::vector<float> ys;
        // 위 좌표에서 다시 계산한 거리와 방위입니다(방위는 비균일).
        std::vector<float> ranges;
        std::vector<float> bearings;
        // 보정에 사용하지 못해 원본 그대로 둔 빔 수입니다(진단용).
        std::size_t uncorrected_count{0};
    };

    ScanDeskew() = default;
    explicit ScanDeskew(const Parameters &parameters);

    void setParameters(const Parameters &parameters);
    const Parameters &parameters() const { return parameters_; }

    // scan의 각 빔을 자기 시각의 pose로 펴서 reference 시각 프레임에 모읍니다.
    // enabled=false면 원본을 그대로 담아 valid=true로 반환합니다.
    bool deskew(
        const sensor_msgs::msg::LaserScan &scan,
        const PoseLookup &pose_at,
        Result &result) const;

    // 기존 LaserScan 소비자(scanScoring, Relocalization)를 위해 균일 각도
    // 격자로 다시 담습니다. 같은 칸에 여러 점이 들어오면 가까운 쪽을 남깁니다.
    // 재비닝이므로 근사이며, 정확도가 중요한 소비자는 Result를 직접 쓰세요.
    static bool toLaserScan(
        const Result &result,
        const sensor_msgs::msg::LaserScan &reference_scan,
        sensor_msgs::msg::LaserScan &out);

private:
    // 빔 인덱스별 시각을 계산합니다. time_increment가 없으면 대체값을 씁니다.
    double beamTime(
        const sensor_msgs::msg::LaserScan &scan,
        std::size_t index,
        double start_seconds,
        double increment) const;

    Parameters parameters_{};
};
