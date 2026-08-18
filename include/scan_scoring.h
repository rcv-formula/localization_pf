#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

#include <Eigen/Dense>

#include "structs.h"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std;

class scanScoring {
public:
    scanScoring(nav_msgs::msg::OccupancyGrid &map, float scoring_factor);
    // 벽 근처 봉우리를 날카롭게 하는 혼합 커널 설정입니다.
    // K(d) = w * exp(-d^2/2*sharp^2) + (1-w) * exp(-d^2/2*sigma^2)
    void setSharpKernel(double sharp_sigma_m, double sharp_weight);
    void setScoringFactor(float scoring_factor);
    void setMap(nav_msgs::msg::OccupancyGrid &map);
    float calcScanLikelihood(const sensor_msgs::msg::LaserScan &scan, particle &p);

    // 고속 경로: 스캔당 1회 빔 테이블(사거리, cos/sin)을 만들어 두고,
    // 파티클마다 각도합 공식으로 회전만 적용합니다. 파티클x빔 trig/log가
    // 전부 사라지고, stride로 빔을 솎아 총량도 줄입니다.
    // prepare 후 scorePrepared는 읽기 전용이라 파티클 병렬 호출이 안전합니다.
    void prepareScan(const sensor_msgs::msg::LaserScan &scan, int beam_stride);
    float scorePrepared(particle &p) const;
    // 해당 pose에서 likelihood field 값이 prob_threshold 미만인(=벽에서 ~2
    // 시그마 이상 떨어진) 빔의 비율입니다. 평균 로그우도는 소수의 큰
    // 불일치에 둔감하므로, 불일치 '비율'을 노이즈 회복 신호로 씁니다.
    double outlierFraction(const particle &p, double prob_threshold) const;

    // ---- beam skipping (AMCL 방식) ----
    // 파티클 다수가 공통으로 설명 못 하는 빔 = 동적/미지도 물체로 보고
    // 채점에서 제외합니다. 단 제외 비율이 error_threshold를 넘으면 필터
    // 자체가 미아일 가능성이므로 스킵을 포기(전체 빔 업데이트)합니다.
    struct BeamSkipStats {
        // 스킵 후보 빔 비율(적용 여부와 무관한 원시 합의 실패율).
        double proposed_fraction{0.0};
        // 스킵 '후보' 빔 끝점들의 중심(라이다 프레임)과 개수. 마스크가
        // 나중에 폐기되어도(제안 비율 > error_threshold) 남습니다.
        //
        // 미아 판정에서 "움직이는 상대차"를 걸러내는 데 씁니다 — 같은 속도로
        // 앞서가는 차는 로봇 프레임에서 정지라 기존 지표로는 미아와 구분되지
        // 않지만, 월드(odom) 프레임에서는 에고 속도로 움직입니다.
        double centroid_x{0.0};
        double centroid_y{0.0};
        std::size_t skip_beams{0};
        bool applied{false};
    };
    void configureBeamSkip(bool enabled, double prob_threshold,
                           double consensus, double error_threshold,
                           int particle_stride);
    // prepareScan 이후, 병렬 채점 이전에 1회 호출(마스크는 읽기 전용이 됨).
    BeamSkipStats computeBeamSkip(const particle *particles, int32_t count);
    // 시각화용: 스킵된 빔만 남기고 나머지는 NaN인 LaserScan을 만듭니다.
    void buildSkippedScan(const sensor_msgs::msg::LaserScan &input,
                          sensor_msgs::msg::LaserScan &output) const;


private:
    void mapProcess();
    void buildGaussianLUT();
    
    // 최적화: 1차원 flatten 배열 인덱스 계산
    inline int32_t getIndex(int32_t px, int32_t py) {
        return py * mapWidth_ + px;
    }
    
    // 로그 정규화: log(0) 방지 epsilon 추가
    inline double logNormalizeScore(double logSum, int32_t numPoints) {
        const double epsilon = 1e-10;
        if (numPoints <= 0) return 0.0;
        return log(logSum + epsilon) / static_cast<double>(numPoints);
    }
    
    // 가우시안 함수
    inline double gaussianValue(double x, double y, double sigma) {
        const double twoSigmaSq = 2.0 * sigma * sigma;
        return exp(-(x * x + y * y) / twoSigmaSq);
    }

    // 멤버 변수 - 모두 Eigen으로 통일
    // 원본 맵을 Eigen 행렬로 저장 (int8_t 형태)
    Eigen::MatrixXi fullMapEigen_;  // 원본 occupancy 맵 (row-major: height x width)
    Eigen::VectorXi fullMapData_;    // 원본 데이터 저장 (ROS 메시지용)
    
    // 가우시안 블러 적용된 확률 맵 (double 형태)
    Eigen::MatrixXd processedMap_;  // 가우시안 블러 결과 (height x width)
    // log(prob + eps) 사전계산 — 빔당 log() 호출 제거용.
    // float row-major 연속 배열: 약한 기기의 캐시에 맞춰 대역폭 절반.
    std::vector<float> logProcessedMap_;
    // prepareScan이 채우는 유효 빔 테이블(레이저 프레임).
    std::vector<double> prep_range_;
    std::vector<double> prep_cos_;
    std::vector<double> prep_sin_;
    std::vector<int32_t> prep_beam_index_;
    // 스캔 기하(각도 시작/간격/개수/stride)는 매 스캔 동일하므로 빔 각도
    // trig를 1회만 계산해 캐시합니다.
    std::vector<double> beam_cos_table_;
    std::vector<double> beam_sin_table_;
    std::vector<int32_t> beam_index_table_;
    // beam skip 상태
    bool beam_skip_enabled_{false};
    double beam_skip_log_threshold_{-3.0};
    double beam_skip_consensus_{0.3};
    double beam_skip_error_threshold_{0.5};
    int beam_skip_particle_stride_{8};
    std::vector<uint8_t> beam_skip_mask_;
    double table_angle_min_{1.0e18};
    double table_angle_inc_{0.0};
    std::size_t table_count_{0};
    int table_stride_{0};
    
    int32_t mapWidth_;
    int32_t mapHeight_;
    
    // 최적화: 나눗셈 방지용 역수
    double invResolution_;
    double sharpSigma_{0.05};
    double sharpWeight_{0.6};
    
    // 맵 오프셋 (월드 좌표 -> 픽셀 좌표 변환용)
    double mapOriginX_;
    double mapOriginY_;
    
    // 스코어링 팩터 (가우시안 sigma로 사용)
    float scoringFactor_;
    double gaussianSigma_;
    
    // 가우시안 커널 (직접 Eigen::MatrixXd로 저장 - 중복 제거)
    Eigen::MatrixXd gaussianLUT_;  // 2차원 가우시안 커널 (kernelSize x kernelSize)
    int32_t lutRadius_;  // 커널 반지름 (ceil(3 * sigma))
    int32_t lutKernelSize_;  // 커널 크기 (2*radius+1)
};