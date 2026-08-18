#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include <Eigen/Core>

#include "structs.h"

// 파티클 집합을 소유하고 초기화/가중정규화/리샘플링/구름통계를 담당합니다.
//
// 역할 경계:
//  - 예측(모션)     : particlePropagation::propagation()
//  - 측정(우도)     : scanScoring::calcScanLikelihood()  -> score[0], score[1]
//  - 가중/리샘플    : 이 클래스                          -> score[2]
//  - 이방성 융합    : estimation 모듈                    -> score[3], score[4]
//
// 이 클래스는 스캔이나 맵을 알지 못하며, 파티클 배열과 가중만 다룹니다.
class ParticleFilter {
public:
    struct Parameters {
        int32_t particle_count{1000};
        // score[0](빔당 평균 log-likelihood)을 가중으로 바꿀 때의 온도입니다.
        // 크게 하면 분포가 뾰족해지고, 작게 하면 평평해집니다.
        double likelihood_scale{1.0};
        // 유효표본수(Neff)가 (이 비율 * 파티클 수) 미만이면 리샘플링합니다.
        double resample_neff_ratio{0.5};
        // relocalization 결과 주변에 뿌릴 때의 초기 표준편차입니다.
        double init_position_std{0.10};
        double init_yaw_std{0.05};
        uint32_t seed{20250721u};
    };

    // 파티클 구름 자체의 통계입니다. 휠/스캔 이방성 융합은 estimation 모듈이
    // 이 값을 받아 수행하므로, 여기서는 순수 가중 통계만 냅니다.
    struct Estimate {
        bool valid{false};
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
        Eigen::Matrix2d position_covariance{Eigen::Matrix2d::Zero()};
        double yaw_variance{0.0};
        double effective_sample_size{0.0};
    };

    ParticleFilter() = default;
    explicit ParticleFilter(const Parameters &parameters);

    void setParameters(const Parameters &parameters);
    const Parameters &parameters() const { return parameters_; }

    // relocalization이 준 pose 주변에 가우시안으로 파티클을 뿌립니다.
    // 파티클을 새로 뿌린 뒤에는 반드시
    // particlePropagation::resetPropagationReference()도 호출해야 합니다.
    void initializeAround(double x, double y, double yaw);
    void initializeAround(
        double x, double y, double yaw,
        double position_std, double yaw_std);

    // 다중 가설 초기화: 파티클 예산을 가설별로 균등 분할해 뿌리고 mode
    // 라벨을 남깁니다. 정지 중에는 앨리어스끼리 점수가 같아 모든 모드가
    // 공존하고, 주행이 시작되면 틀린 모드의 우도가 무너져 리샘플링에서
    // 자연 소멸합니다.
    struct ModeSeed {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
        // 이 모드에 배분할 파티클 질량의 상대 비중. 증거가 애매한 후보를
        // 거부하는 대신 적은 질량으로 심어 주행이 판별하게 하는 데 씁니다
        // (보호관찰). 모두 같으면 예전처럼 균등 분할입니다.
        double weight{1.0};
    };
    void initializeMultiple(const std::vector<ModeSeed> &seeds);

    // 모드별 가중 질량 요약입니다. alive는 질량 1e-3 초과 모드 수입니다.
    struct ModeSummary {
        int alive_modes{0};
        uint16_t dominant{0};
        double dominant_mass{0.0};
    };
    ModeSummary modeSummary() const;
    // 해당 모드의 파티클만으로 낸 가중 통계입니다(모드 내부 재정규화).
    // 다봉 상태에서 전체 평균을 쓰면 모드 사이(벽 안)가 나오므로, pose
    // 출력은 항상 지배 모드의 이 값을 씁니다.
    Estimate estimateOfMode(uint16_t mode) const;
    uint16_t seededModeCount() const { return seeded_mode_count_; }

    bool initialized() const { return initialized_; }
    void reset();

    // propagation/scoring이 직접 순회할 수 있도록 연속 메모리를 노출합니다.
    particle *data() { return particles_.data(); }
    const particle *data() const { return particles_.data(); }
    int32_t count() const { return count_; }
    // propagation(particle*, int32_t&) 시그니처에 그대로 넘기기 위한 참조입니다.
    int32_t &countRef() { return count_; }

    // score[0]을 정규화 가중 score[2]로 바꾸고 유효표본수를 반환합니다.
    double normalizeWeights();
    double effectiveSampleSize() const;

    // 저분산(systematic) 리샘플링입니다. 가중은 균등으로 초기화됩니다.
    void resample();
    // Neff가 문턱 아래일 때만 리샘플링하고, 수행 여부를 반환합니다.
    // 정지 중 다양성 고갈을 막으려면 호출부가 이동량으로 한 번 더 걸러야 합니다.
    bool resampleIfNeeded();

    Estimate estimate() const;

private:
    // mode < 0 이면 전체, 아니면 해당 모드의 파티클만으로 통계를 냅니다.
    Estimate estimateFiltered(int mode) const;

    Parameters parameters_{};
    std::vector<particle> particles_;
    std::vector<particle> resample_buffer_;
    int32_t count_{0};
    bool initialized_{false};
    uint16_t seeded_mode_count_{1};
    std::mt19937 rng_{20250721u};
};
