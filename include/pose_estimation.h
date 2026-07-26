#pragma once

#include <cstdint>

#include <Eigen/Core>

#include "structs.h"

// 파티클 구름 추정과 휠(odom) 예측을 방향별로 다르게 섞어 최종 pose를 만듭니다.
//
// 직선 복도처럼 스캔이 한 방향으로만 정보를 못 주는(aperture) 상황에서, 그
// 방향은 휠을 더 믿고 나머지 방향은 스캔을 믿습니다. 임계값 분기 없이 관측성
// 신뢰도에 연속 비례하며, 튜닝 노브는 감쇠 계수 하나입니다.
//
// 관측성은 라이브 스캔이 아니라 맵 사전계산(Relocalization::queryStaticGeometry)
// 에서 오므로, 맵에 없는 동적 장애물이 만드는 가짜 기하 정보에 신뢰 방향이
// 오염되지 않습니다.
//
// 다른 모듈 타입에 의존하지 않도록 입력은 평범한 구조체로 받습니다.
class PoseEstimation {
public:
    // 파티클 구름에서 얻은 스캔 기반 추정입니다(ParticleFilter::estimate()).
    struct ScanEstimate {
        bool valid{false};
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
        Eigen::Matrix2d position_covariance{Eigen::Matrix2d::Zero()};
        double yaw_variance{0.0};
    };

    // 직전 융합 pose를 기준으로 한 body frame 이동량입니다.
    // particlePropagation이 파티클에 적용한 것과 같은 값을 넘기면 됩니다.
    struct OdomDelta {
        bool valid{false};
        double longitudinal{0.0};
        double lateral{0.0};
        double yaw{0.0};
    };

    // 맵 기반 static 관측성입니다(Relocalization::queryStaticGeometry()). confidence는 축별 신뢰도 [0,1]이며 내림차순
    // eigenvectors와 짝을 이룹니다. 고유값 비율(lambda/lambda_max)이 아니라
    // 절대량을 사상한 값이어야 합니다 — 비율은 강축이 항상 1이 되어
    // "양쪽 다 나쁨"(개활지, 벽이 전부 사거리 밖)을 표현할 수 없습니다.
    struct Observability {
        bool valid{false};
        Eigen::Matrix2d eigenvectors{Eigen::Matrix2d::Identity()};
        Eigen::Vector2d confidence{Eigen::Vector2d::Zero()};
    };

    // 융합 형태 (라): 축별 스캔 가중을 기하 관측성에서만 뽑습니다.
    //
    //     alpha(v) = scan_trust * confidence(v)
    //
    // 이전 형태는 alpha를 예측/스캔 공분산의 정보량 비로 계산했는데, 그러면
    // 융합 공분산이 매 사이클 P <- 1/(I_scan + 1/(P+g)) 로 갱신되어 프로세스
    // 노이즈 g가 작을 때 P -> 0 으로 붕괴합니다. 실데이터에서 관측성이
    // conf=0.92로 좋은데도 alpha가 0.02까지 떨어져 융합 출력이 사실상 순수
    // dead-reckoning이 됐습니다(예측 sigma 8 mm). 파티클 구름 분산은 매
    // 사이클 독립인 측정이 아닌데 독립 증거처럼 더해서 생긴 과신입니다.
    //
    // (라)는 그 되먹임 자체를 끊습니다. 공분산은 보고용으로만 유지하고
    // alpha에 관여시키지 않습니다. scan_trust는 [0,1] 감쇠 노브라서 아무리
    // 키워도 confidence=0인 방향의 보호를 무력화할 수 없습니다.
    struct Parameters {
        bool enabled{true};
        // 관측성이 완전한 방향에서 스캔 혁신을 얼마나 적용할지입니다 [0, 1].
        double scan_trust{1.0};
        // yaw는 아직 관측성 지표가 없어 상수 가중입니다. 3x3 Hessian +
        // Schur 주변화를 넣으면 여기도 confidence로 게이팅합니다.
        double yaw_scan_trust{1.0};
        // 이동량에 비례해 커지는 odom 예측 공분산 증가율입니다.
        // 보고되는 공분산에만 쓰이고 alpha에는 영향을 주지 않습니다.
        double odom_position_growth{0.10};   // [m] per [m]
        double odom_yaw_growth{0.05};        // [rad] per [rad]
        double odom_yaw_growth_per_meter{0.02};
        // 0 나눗셈 방지용 하한입니다.
        double min_variance{1.0e-9};
    };

    struct Result {
        bool valid{false};
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
        Eigen::Matrix2d position_covariance{Eigen::Matrix2d::Zero()};
        double yaw_variance{0.0};
        // 진단용: 관측성 고유방향별 스캔 신뢰도 alpha(0=휠, 1=스캔)와 그 기저입니다.
        Eigen::Vector2d scan_trust{Eigen::Vector2d::Ones()};
        Eigen::Matrix2d eigenvectors{Eigen::Matrix2d::Identity()};
        double yaw_scan_trust{1.0};
    };

    PoseEstimation() = default;
    explicit PoseEstimation(const Parameters &parameters);

    void setParameters(const Parameters &parameters);
    const Parameters &parameters() const { return parameters_; }

    // relocalization으로 다시 시작할 때 기준 pose를 새로 잡습니다.
    void reset(double x, double y, double yaw);
    void reset(
        double x, double y, double yaw,
        const Eigen::Matrix2d &position_covariance,
        double yaw_variance);
    void invalidate();
    bool initialized() const { return initialized_; }

    // odom 이동량으로 예측한 뒤 스캔 추정과 방향별로 융합합니다.
    Result update(
        const ScanEstimate &scan_estimate,
        const OdomDelta &odom_delta,
        const Observability &observability);

    const Result &last() const { return last_; }

    // 진단용으로 각 파티클의 약축/강축 오프셋을 score[3]/score[4]에 적습니다.
    // 융합 계산 자체는 이 값을 쓰지 않습니다(구름 공분산으로 충분).
    //   score[3] = 약축(퇴화 방향) 오프셋, score[4] = 강축 오프셋
    static void annotateParticles(
        particle *particles,
        int32_t particle_count,
        const Result &result);

private:
    Parameters parameters_{};
    bool initialized_{false};
    double x_{0.0};
    double y_{0.0};
    double yaw_{0.0};
    Eigen::Matrix2d position_covariance_{Eigen::Matrix2d::Identity()};
    double yaw_variance_{1.0};
    Result last_{};
};
