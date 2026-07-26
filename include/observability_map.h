#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "relocalization.h"

// 맵 기반 기하 퇴화 사전계산 — 미끄러짐 민감도(slip sensitivity).
//
// 질문을 정확히 하나로 고정한다:
//   "이 (지점, heading)에서 pose를 방향 v로 밀면, 스캔과 맵의 평균 불일치가
//    얼마나 빨리 자라는가. 뒤집어서, 평균 불일치가 허용치 ε에 도달하려면
//    몇 미터를 밀어야 하는가."
//
// 유도
//   빔이 법선 n인 벽을 맞혔을 때, pose를 v로 delta 밀면 그 빔의 점-벽
//   불일치는 (n . v) * delta 다 (점-직선 거리의 1차 근사). 스캔 전체의
//   RMS 불일치 민감도는
//
//     s(v)^2 = (1/N_total) * sum_hit (n . v)^2 = v^T M v
//     M      = (1/N_total) * sum_hit n n^T          (2x2)
//
//   여기서 N_total은 miss를 포함한 전체 빔 수다. 그래서
//     - 고유값은 [0, 1]. 1이면 "모든 빔이 그 방향을 정면으로 구속".
//     - trace(M) = 적중률. 개활지처럼 맞는 빔 자체가 적으면 모든 방향의
//       고유값이 함께 내려간다. 별도의 유효성 게이트가 필요 없다.
//     - 빔 개수, 센서 노이즈와 무관한 순수 기하량이다. CRLB(sigma/sqrt(N))
//       계열은 빔 수가 지배해 mm 단위가 나오므로 퇴화 지표로 부적합했다.
//
//   미터 단위 변환:
//     sigma(v) = slip_tolerance_m / sqrt(lambda(v)),  max_sigma_m 로 포화.
//   "방향 v로 sigma(v)만큼 밀면 평균 불일치가 slip_tolerance_m에 도달한다."
//
// 입력은 오직 "그 지점, 그 heading에서 쏜 raycast" 하나다. 블러도, 응답면도,
// 최근접 스냅도 없다.
class ObservabilityMap {
public:
    // 법선 추정 방식입니다.
    enum class NormalMethod {
        // 인접 빔 끝점 차분. 0.5도/0.05m에서 r < 5.7m면 끝점 간격이 격자보다
        // 작아 접선이 격자 방향으로 스냅됩니다(실측: 직선 복도의 복도축이
        // 수백 배 과대 구속). 비교용으로만 두세요.
        AdjacentDifference,
        // 벽 위 목표 호길이만큼 창을 넓혀 TLS 적합.
        ChordWindowTls,
        // 스캔을 선분으로 분할(split-and-merge/IEPF)하고 선분별 법선을
        // 지지 빔 수로 가중. 짧은 덩어리(맵 노이즈)는 선분을 못 이루어
        // 자연히 떨어집니다. 기본값.
        LineDetection
    };

    struct Parameters {
        // ----- 사전계산 격자 -----
        double grid_step_m{0.30};
        int heading_bins{32};
        double clearance_m{0.15};
        // 격자점을 unknown 셀에도 놓을지. 실주행 맵에서 로봇은 unknown에
        // 들어가지 않으므로 노드에서는 false(기본)입니다. 시각화로 맵
        // 전체를 훑을 때만 켭니다.
        bool include_unknown{false};
        // 관측성용 raycast 사거리[m]. 실효 반사거리 기준이며 relocalization의
        // max_range(전역 탐색용, 원거리)와 별개입니다. unknown은 벽이 아니라
        // 통과로 취급합니다.
        double max_range_m{5.0};

        // ----- 지표 -----
        // 평균 불일치 허용치[m]. sigma(v) = 이 값 / sqrt(lambda(v)).
        // "그 방향으로 sigma만큼 밀리면 스캔이 평균 이만큼 어긋난다"의 기준.
        // 스코어링 likelihood field의 sigma(0.1~0.2m)나 맵 해상도 수준이 자연스럽다.
        double slip_tolerance_m{0.10};
        // 관측 불가 방향의 sigma 포화값[m].
        double max_sigma_m{10.0};

        // ----- 법선 추정 -----
        NormalMethod normal_method{NormalMethod::LineDetection};
        double normal_chord_m{0.30};
        int normal_max_half_window{40};
        double normal_residual_limit_m{0.06};

        // ----- 선분 추출(LineDetection) -----
        // IEPF 분할 기준: 끝점을 잇는 직선에서 이 수직거리[m]를 넘는 점이
        // 있으면 그 점에서 쪼갭니다.
        double line_split_m{0.10};
        // 선분 후보의 최소 지지 빔 수(3 미만은 TLS 자체가 불가)와 최소
        // 길이[m]. 하드 게이트는 최소한으로 두고 실제 신뢰도는 아래 각도
        // 스미어링이 연속적으로 처리합니다.
        int line_min_beams{3};
        double line_min_length_m{0.10};

        // ----- 가중·신뢰 모델 (조사 결론) -----
        // 가중치는 빔 수가 아니라 등가 길이 w = L / (r_ref * ray_step) 입니다.
        // (Shi-Tomasi 구조텐서 / Fisher 정보 가법성: 정보는 독립 샘플 수
        // = 길이에 비례. 빔 수 가중은 같은 벽도 거리에 따라 값이 변하는
        // 시점 의존성을 만들었습니다.) r_ref는 등가화 기준 거리입니다.
        double reference_range_m{3.0};
        // 선분에 수직 방향 점 잡음[m]. 법선 각도 불확실성
        //   sigma_alpha = sigma_perp * sqrt(12 / b) / L      (Pfister 2003)
        // 로 전파되어, 기여를 폐형식
        //   M_i = w [ e^(-2 sa^2) n n^T + (1 - e^(-2 sa^2)) I/2 ]
        // 로 스미어링합니다. 짧거나 빔이 적은 선분은 방향 스파이크가 아니라
        // 넓게 퍼진 소량이 되어, 문턱 이진성이 만들던 계단 점프가 사라집니다.
        double perpendicular_noise_m{0.05};
        // 조회 시 방향별 lambda를 이웃 entry들의 최소값으로 취합니다(보수적).
        // 한계 특징이 격자 사이에서 나타났다 사라졌다 하며 만드는 계단
        // 점프를 억제합니다. 끄면 순수 bilinear 평균.
        bool conservative_query{true};
        // 조회 코너가 전부 무효일 때 가장 가까운 유효 격자점을 찾는 나선
        // 탐색 반경[격자 칸]. 2칸 = 0.6 m. 커버리지 구멍 하나가 스캔 신뢰를
        // 통째로 끄지 않게 합니다. 이 반경 밖이면 진짜 맵 밖으로 봅니다.
        int snap_radius_cells{2};
        // 법선 방향이 이 각도[도] 이내인 선분들을 하나의 방향으로 군집화해
        // 가중 평균 법선으로 누적합니다. 벽 요철이 만드는 선분 각도 지터
        // (±1~3도)는 "같은 방향의 반복 측정"이므로 스프레드가 소멸하고,
        // 그보다 크게 기운 벽(곡률 등 진짜 기하)은 별도 군집으로 남습니다.
        // 0이면 끕니다.
        double direction_cluster_deg{5.0};
    };

    struct Sample {
        bool valid{false};
        // M = (1/N_total) sum n n^T. 고유값 내림차순이 아니라, 아래
        // eigenvectors/sigma와 짝을 이루는 형태로 보관합니다.
        Eigen::Matrix2d sensitivity{Eigen::Matrix2d::Zero()};
        // 열 0 = 강축(민감도 큼 = 잘 구속), 열 1 = 약축(퇴화 방향).
        Eigen::Matrix2d eigenvectors{Eigen::Matrix2d::Identity()};
        // 민감도 고유값 [0,1], 내림차순. lambda(0)=강축.
        Eigen::Vector2d lambda{Eigen::Vector2d::Zero()};
        // sigma(v) = slip_tolerance / sqrt(lambda), [m]. 오름차순(강축 먼저).
        Eigen::Vector2d sigma{Eigen::Vector2d::Zero()};
        // 적중률 = trace(M) ∈ [0,1].
        double hit_fraction{0.0};
        std::size_t normal_count{0};
    };

    // 디버그: 한 스캔에서 검출된 선분입니다.
    struct LineSegment {
        double normal_angle_deg{0.0};   // 법선 방향(세계 프레임)
        double length_m{0.0};
        int beams{0};
        double nx{0.0};
        double ny{0.0};
        double mean_range_m{0.0};       // 선분 중심까지의 평균 거리
    };

    // 한 (x, y, heading)의 raycast에서 선분을 추출해 돌려줍니다(시각화/진단용).
    // computeEntry의 LineDetection 경로와 같은 코드를 사용합니다.
    std::vector<LineSegment> extractSegments(
        const float *ranges,
        std::size_t ray_count,
        double start_angle_rad,
        double ray_step_rad,
        double max_hit_range,
        double resolution) const;

    struct Diagnostics {
        std::size_t grid_points{0};
        std::size_t valid_points{0};
        std::size_t entries{0};
        double build_seconds{0.0};
        double memory_bytes{0.0};
    };

    ObservabilityMap() = default;

    void setParameters(const Parameters &parameters);
    const Parameters &parameters() const { return parameters_; }

    void build(
        const nav_msgs::msg::OccupancyGrid &map,
        const Relocalization &relocalization,
        double fov_deg,
        std::size_t fov_rays);

    bool ready() const { return ready_; }
    void clear();

    // 공간 bilinear + heading 선형보간. M은 평균이라 가법적이어서 보간이
    // 안전합니다. 보간 후 고유분해합니다.
    Sample query(double world_x, double world_y, double heading) const;

    const Diagnostics &diagnostics() const { return diagnostics_; }

private:
    // 격자점 하나, heading bin 하나. M의 상삼각 3성분.
    struct Entry {
        float mxx{0.0f};
        float mxy{0.0f};
        float myy{0.0f};
        uint16_t normal_count{0};
        uint16_t valid{0};
    };

    const Entry *entry(int ix, int iy, int heading_bin) const;
    Entry computeEntry(
        const float *ranges,
        std::size_t ray_count,
        double start_angle_rad,
        double ray_step_rad,
        double max_hit_range,
        double resolution) const;

    Parameters parameters_{};
    bool ready_{false};
    double origin_x_{0.0};
    double origin_y_{0.0};
    int grid_width_{0};
    int grid_height_{0};
    std::vector<Entry> entries_;
    Diagnostics diagnostics_{};
};
