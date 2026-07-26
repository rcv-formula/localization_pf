#include "scan_scoring.h"

// ============================================================
// 생성자: 맵과 스코어링 팩터로 초기화
// ============================================================
scanScoring::scanScoring(nav_msgs::msg::OccupancyGrid &map, float scoring_factor) {
    // 멤버 초기화
    scoringFactor_ = scoring_factor;
    gaussianSigma_ = static_cast<double>(scoring_factor);
    
    // setMap에서 모든 설정 + mapProcess() 호출
    setMap(map);
}

// ============================================================
// setScoringFactor: 스코어링 팩터 변경 시 호출
// ============================================================
void scanScoring::setSharpKernel(double sharp_sigma_m, double sharp_weight) {
    sharpSigma_ = std::max(0.005, sharp_sigma_m);
    sharpWeight_ = std::clamp(sharp_weight, 0.0, 1.0);
    if (mapWidth_ > 0 && mapHeight_ > 0) {
        buildGaussianLUT();
        mapProcess();
    }
}

void scanScoring::setScoringFactor(float scoring_factor) {
    scoringFactor_ = scoring_factor;
    gaussianSigma_ = static_cast<double>(scoring_factor);
    
    // sigma가 변경되면 LUT와 processedMap 재계산 필요
    if (mapWidth_ > 0 && mapHeight_ > 0) {
        buildGaussianLUT();
        mapProcess();
    }
}

// ============================================================
// setMap: 맵을 설정하고 processedMap 생성
// ROS 메시지 → Eigen 변환 + buildGaussianLUT + mapProcess 모두 호출
// ============================================================
void scanScoring::setMap(nav_msgs::msg::OccupancyGrid &map) {
    // 맵 정보 추출
    mapWidth_ = map.info.width;
    mapHeight_ = map.info.height;
    invResolution_ = 1.0 / map.info.resolution;  // 최적화: 나눗셈 → 곱셈
    mapOriginX_ = map.info.origin.position.x;
    mapOriginY_ = map.info.origin.position.y;
    
    // 원본 데이터를 Eigen VectorXi에 복사
    fullMapData_.resize(mapWidth_ * mapHeight_);
    for (int32_t i = 0; i < mapWidth_ * mapHeight_; ++i) {
        fullMapData_[i] = static_cast<int32_t>(map.data[i]);
    }
    
    // Eigen MatrixXi로 2차원 형태 구성 (height x width)
    fullMapEigen_.resize(mapHeight_, mapWidth_);
    for (int32_t py = 0; py < mapHeight_; ++py) {
        for (int32_t px = 0; px < mapWidth_; ++px) {
            fullMapEigen_(py, px) = fullMapData_[py * mapWidth_ + px];
        }
    }
    
    // processedMap 초기화 (height x width)
    processedMap_.resize(mapHeight_, mapWidth_);
    processedMap_.setZero();
    
    // 가우시안 LUT 생성
    buildGaussianLUT();
    
    // 맵 프로세스 수행 (가우시안 블러)
    mapProcess();
}

// ============================================================
// buildGaussianLUT: 가우시안 커널 생성
// gaussianLUT_은 직접 Eigen::MatrixXd로 저장 (중복 제거)
// ============================================================
void scanScoring::buildGaussianLUT() {
    // 혼합 커널: 벽 바로 근처는 좁은 성분(sharpSigma_)이 급격한 봉우리를,
    // 유역은 넓은 성분(gaussianSigma_)이 유지합니다. 적용 반경은 넓은
    // 성분의 3시그마(예: 0.2m -> 12셀 = 0.6m)로 종전과 동일합니다.
    const double resolution = 1.0 / invResolution_;
    const double sigma_cells = gaussianSigma_ * invResolution_;
    lutRadius_ = static_cast<int32_t>(ceil(3.0 * sigma_cells));
    if (lutRadius_ < 1) lutRadius_ = 1;
    lutKernelSize_ = 2 * lutRadius_ + 1;
    gaussianLUT_.resize(lutKernelSize_, lutKernelSize_);
    const double wide_var2 = 2.0 * gaussianSigma_ * gaussianSigma_;
    const double sharp_var2 = 2.0 * sharpSigma_ * sharpSigma_;
    for (int dy = -lutRadius_; dy <= lutRadius_; ++dy) {
        for (int dx = -lutRadius_; dx <= lutRadius_; ++dx) {
            const double d2 =
                (static_cast<double>(dx) * dx + static_cast<double>(dy) * dy) *
                resolution * resolution;
            gaussianLUT_(dy + lutRadius_, dx + lutRadius_) =
                sharpWeight_ * std::exp(-d2 / sharp_var2) +
                (1.0 - sharpWeight_) * std::exp(-d2 / wide_var2);
        }
    }
}

// ============================================================
// mapProcess: 장애물에 가우시안 블러 적용 (Eigen 기반)
// fullMapEigen_과 gaussianLUT_은 이미 Eigen 형태로 setMap/buildGaussianLUT에서 설정됨
// processedMap_에 직접 결과 저장
// processedMap_의 값 범위: [0.0, 1.0]
// ============================================================
void scanScoring::mapProcess() {
    // processedMap 초기화
    processedMap_.setZero();
    
    // 원본 맵 데이터 직접 포인터 (최적화: 맵 순회 시 사용)
    const int32_t* rawData = fullMapData_.data();
    
    for (int32_t py = 0; py < mapHeight_; ++py) {
        for (int32_t px = 0; px < mapWidth_; ++px) {
            int32_t cellValue = rawData[py * mapWidth_ + px];
            
            // 장애물 체크: value > 0 (1~99는 중간, 100은 완전 장애물)
            if (cellValue > 0) {
                // 가우시안 커널을 이 위치에 추가
                int32_t xMin = max(0, px - lutRadius_);
                int32_t xMax = min(mapWidth_ - 1, px + lutRadius_);
                int32_t yMin = max(0, py - lutRadius_);
                int32_t yMax = min(mapHeight_ - 1, py + lutRadius_);
                
                // 커널 부분 인덱스 (경계 고려)
                int32_t kStartY = lutRadius_ - (py - yMin);
                int32_t kStartX = lutRadius_ - (px - xMin);
                
                double weight = static_cast<double>(cellValue) / 100.0;
                
                // 가우시안 값 추가
                for (int32_t ky = yMin; ky <= yMax; ++ky) {
                    for (int32_t kx = xMin; kx <= xMax; ++kx) {
                        int32_t kIdxY = kStartY + (ky - yMin);
                        int32_t kIdxX = kStartX + (kx - xMin);
                        double gaussVal = gaussianLUT_(kIdxY, kIdxX);
                        
                        // 커널은 누적합이 아니라 max로 합성합니다.
                        // 표준 likelihood field는 "최근접 장애물까지 거리"의
                        // 가우시안이고, 그게 곧 커널들의 max입니다.
                        // 합으로 쌓으면 벽 하나가 여러 셀에 걸쳐 있으므로 값이
                        // 1.0으로 포화되어 벽 주변 수십 cm가 통째로 평평해집니다.
                        // 그러면 그 구간에서 pose를 흔들어도 점수가 변하지 않아
                        // PF의 판별력과 응답면 곡률이 함께 사라집니다.
                        // (Karto ScanMatcher CorrelationGrid::SmearPoint과 동일)
                        processedMap_(ky, kx) = max(processedMap_(ky, kx), gaussVal * weight);
                    }
                }
            }
        }
    }
    // 빔당 log() 제거: 로그 공간 맵을 한 번만 만들어 둡니다.
    // float row-major 연속 배열 — 무작위 셀 접근의 캐시 적중률을 높입니다.
    logProcessedMap_.resize(static_cast<std::size_t>(mapWidth_) * mapHeight_);
    for (int32_t py = 0; py < mapHeight_; ++py) {
        for (int32_t px = 0; px < mapWidth_; ++px) {
            logProcessedMap_[static_cast<std::size_t>(py) * mapWidth_ + px] =
                static_cast<float>(std::log(processedMap_(py, px) + 1.0e-10));
        }
    }

}

// ============================================================
// calcScanLikelihood: 스캔 포인트의 likelihood 계산
// - 파티클 pose 기준 스캔 포인트 월드 좌표 계산
// - processedMap에서 해당 위치 값 가져와 곱셈 (로그 공간에서 합산)
// - 반환: 정규화된 스코어 (float)
// ============================================================
float scanScoring::calcScanLikelihood(const sensor_msgs::msg::LaserScan &scan, particle &p) {
    const float* ranges = scan.ranges.data();
    
    // structs.h의 particle은 (x, y, theta, score[5]) 정의됨
    // x, y, theta를 파티클 위치로 사용
    double pX = static_cast<double>(p.x);
    double pY = static_cast<double>(p.y);
    double pTheta = static_cast<double>(p.theta);  // 기본값 0.0, 필요시 score[0] 등으로 변경 가능
    
    double logLikelihood = 0.0;
    int32_t validPoints = 0;
    
    const double epsilon = 1e-10;
    size_t numRanges = scan.ranges.size();
    double angleInc = static_cast<double>(scan.angle_increment);
    
    // 최적화: 반복문 내부에서 메모리 엑세스 최소화
    for (size_t i = 0; i < numRanges; ++i) {
        double range = static_cast<double>(ranges[i]);
        
        // NaN 또는 Inf 체크
        if (range < scan.range_min || range > scan.range_max || 
            std::isnan(range) || std::isinf(range)) {
            continue;
        }
        
        // 스캔 각도
        double scanAngle = static_cast<double>(scan.angle_min) + static_cast<double>(i) * angleInc;
        double absAngle = scanAngle + pTheta;
        
        // cos/sin 미리 계산
        double cosAngle = cos(absAngle);
        double sinAngle = sin(absAngle);
        
        // 월드 좌표 변환
        double worldX = pX + range * cosAngle;
        double worldY = pY + range * sinAngle;
        
        // 픽셀 좌표 변환 (최적화: invResolution_ 사용 - 나눗셈 → 곱셈)
        // floor여야 합니다. int 캐스트는 0쪽으로 잘라내므로 맵 원점보다 살짝
        // 작은 좌표(-res, 0)가 인덱스 0으로 앨리어싱되어, 맵 왼쪽/아래 경계
        // 바깥의 빔이 맵 첫 행/열 값을 읽습니다. 아래 범위검사도 못 걸러냅니다.
        int32_t px = static_cast<int32_t>(std::floor((worldX - mapOriginX_) * invResolution_));
        int32_t py = static_cast<int32_t>(std::floor((worldY - mapOriginY_) * invResolution_));
        
        // 맵 범위 체크 및 직접 엑세스 (Eigen 행렬)
        double prob = 0.0;
        if (px >= 0 && px < mapWidth_ && py >= 0 && py < mapHeight_) {
            prob = processedMap_(py, px);
        }
        
        // 로그 공간에서 합산 (곱셈 → 합산 변환으로 과플로우 방지)
        logLikelihood += log(prob + epsilon);
        validPoints++;
    }
    
    // 정규화: 포인트 수로 나눔
    if (validPoints <= 0) {
        return -std::numeric_limits<float>::max();  // 유효한 포인트 없음
    }
    
    double normalizedScore = logLikelihood / static_cast<double>(validPoints);
    p.score[0] = static_cast<float>(normalizedScore);
    p.score[1] = static_cast<float>(validPoints);
    
    return static_cast<float>(normalizedScore);
}

void scanScoring::prepareScan(const sensor_msgs::msg::LaserScan &scan, int beam_stride) {
    const int stride = std::max(1, beam_stride);
    const std::size_t count = scan.ranges.size();
    // 스캔 기하가 바뀔 때만(사실상 1회) 빔 각도 trig 테이블을 재계산합니다.
    if (table_count_ != count || table_stride_ != stride ||
        table_angle_min_ != static_cast<double>(scan.angle_min) ||
        table_angle_inc_ != static_cast<double>(scan.angle_increment)) {
        beam_cos_table_.clear();
        beam_sin_table_.clear();
        beam_index_table_.clear();
        for (std::size_t i = 0; i < count; i += static_cast<std::size_t>(stride)) {
            const double angle = static_cast<double>(scan.angle_min) +
                static_cast<double>(i) * static_cast<double>(scan.angle_increment);
            beam_cos_table_.push_back(std::cos(angle));
            beam_sin_table_.push_back(std::sin(angle));
            beam_index_table_.push_back(static_cast<int32_t>(i));
        }
        table_count_ = count;
        table_stride_ = stride;
        table_angle_min_ = static_cast<double>(scan.angle_min);
        table_angle_inc_ = static_cast<double>(scan.angle_increment);
    }
    prep_range_.clear();
    prep_cos_.clear();
    prep_sin_.clear();
    prep_beam_index_.clear();
    prep_range_.reserve(beam_index_table_.size());
    prep_beam_index_.reserve(beam_index_table_.size());
    prep_cos_.reserve(beam_index_table_.size());
    prep_sin_.reserve(beam_index_table_.size());
    for (std::size_t k = 0; k < beam_index_table_.size(); ++k) {
        const double range =
            static_cast<double>(scan.ranges[beam_index_table_[k]]);
        if (range < scan.range_min || range > scan.range_max ||
            std::isnan(range) || std::isinf(range)) {
            continue;
        }
        prep_range_.push_back(range);
        prep_cos_.push_back(beam_cos_table_[k]);
        prep_sin_.push_back(beam_sin_table_[k]);
        prep_beam_index_.push_back(beam_index_table_[k]);
    }
}

void scanScoring::buildSkippedScan(const sensor_msgs::msg::LaserScan &input,
                                   sensor_msgs::msg::LaserScan &output) const {
    output = input;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    std::fill(output.ranges.begin(), output.ranges.end(), nan);
    output.intensities.clear();
    if (beam_skip_mask_.size() != prep_beam_index_.size()) {
        return;
    }
    for (std::size_t i = 0; i < prep_beam_index_.size(); ++i) {
        if (beam_skip_mask_[i]) {
            const int32_t src = prep_beam_index_[i];
            if (src >= 0 && src < static_cast<int32_t>(input.ranges.size())) {
                output.ranges[src] = input.ranges[src];
            }
        }
    }
}

float scanScoring::scorePrepared(particle &p) const {
    const std::size_t count = prep_range_.size();
    if (count == 0) {
        p.score[0] = -std::numeric_limits<float>::max();
        p.score[1] = 0.0f;
        return p.score[0];
    }
    const double pX = static_cast<double>(p.x);
    const double pY = static_cast<double>(p.y);
    const double cos_theta = std::cos(static_cast<double>(p.theta));
    const double sin_theta = std::sin(static_cast<double>(p.theta));
    // 맵 밖 빔은 prob 0 = log(eps) 상수로 집계합니다(기존 경로와 동일).
    const double outside_log = std::log(1.0e-10);
    double log_likelihood = 0.0;
    std::size_t used = 0;
    const bool has_mask = beam_skip_mask_.size() == count;
    for (std::size_t i = 0; i < count; ++i) {
        if (has_mask && beam_skip_mask_[i]) {
            continue;
        }
        // cos(a+theta) = ca*ct - sa*st, sin(a+theta) = sa*ct + ca*st
        const double dir_x = prep_cos_[i] * cos_theta - prep_sin_[i] * sin_theta;
        const double dir_y = prep_sin_[i] * cos_theta + prep_cos_[i] * sin_theta;
        const double world_x = pX + prep_range_[i] * dir_x;
        const double world_y = pY + prep_range_[i] * dir_y;
        const int32_t px = static_cast<int32_t>(
            std::floor((world_x - mapOriginX_) * invResolution_));
        const int32_t py = static_cast<int32_t>(
            std::floor((world_y - mapOriginY_) * invResolution_));
        if (px >= 0 && px < mapWidth_ && py >= 0 && py < mapHeight_) {
            log_likelihood += static_cast<double>(logProcessedMap_[
                static_cast<std::size_t>(py) * mapWidth_ + px]);
        } else {
            log_likelihood += outside_log;
        }
        ++used;
    }
    if (used == 0) {
        p.score[0] = -std::numeric_limits<float>::max();
        p.score[1] = 0.0f;
        return p.score[0];
    }
    const double normalized = log_likelihood / static_cast<double>(used);
    p.score[0] = static_cast<float>(normalized);
    p.score[1] = static_cast<float>(used);
    return static_cast<float>(normalized);
}

void scanScoring::configureBeamSkip(bool enabled, double prob_threshold,
                                    double consensus, double error_threshold,
                                    int particle_stride) {
    beam_skip_enabled_ = enabled;
    beam_skip_log_threshold_ = std::log(std::max(1.0e-9, prob_threshold));
    beam_skip_consensus_ = consensus;
    beam_skip_error_threshold_ = error_threshold;
    beam_skip_particle_stride_ = std::max(1, particle_stride);
}

scanScoring::BeamSkipStats scanScoring::computeBeamSkip(
    const particle *particles, int32_t count) {
    BeamSkipStats stats;
    const std::size_t beam_count = prep_range_.size();
    beam_skip_mask_.assign(beam_count, 0);
    if (!beam_skip_enabled_ || beam_count == 0 || count <= 0) {
        return stats;
    }
    // 파티클 서브샘플로 빔별 "맵과 일치" 합의율을 계산합니다.
    std::vector<int32_t> good(beam_count, 0);
    int32_t sampled = 0;
    for (int32_t index = 0; index < count;
         index += beam_skip_particle_stride_) {
        const particle &p = particles[index];
        const double pX = static_cast<double>(p.x);
        const double pY = static_cast<double>(p.y);
        const double cos_theta = std::cos(static_cast<double>(p.theta));
        const double sin_theta = std::sin(static_cast<double>(p.theta));
        for (std::size_t i = 0; i < beam_count; ++i) {
            const double dir_x = prep_cos_[i] * cos_theta - prep_sin_[i] * sin_theta;
            const double dir_y = prep_sin_[i] * cos_theta + prep_cos_[i] * sin_theta;
            const int32_t px = static_cast<int32_t>(std::floor(
                (pX + prep_range_[i] * dir_x - mapOriginX_) * invResolution_));
            const int32_t py = static_cast<int32_t>(std::floor(
                (pY + prep_range_[i] * dir_y - mapOriginY_) * invResolution_));
            if (px >= 0 && px < mapWidth_ && py >= 0 && py < mapHeight_ &&
                static_cast<double>(logProcessedMap_[
                    static_cast<std::size_t>(py) * mapWidth_ + px]) >=
                    beam_skip_log_threshold_) {
                ++good[i];
            }
        }
        ++sampled;
    }
    if (sampled <= 0) {
        return stats;
    }
    std::size_t skip_count = 0;
    for (std::size_t i = 0; i < beam_count; ++i) {
        const double fraction =
            static_cast<double>(good[i]) / static_cast<double>(sampled);
        if (fraction < beam_skip_consensus_) {
            beam_skip_mask_[i] = 1;
            ++skip_count;
        }
    }
    stats.proposed_fraction =
        static_cast<double>(skip_count) / static_cast<double>(beam_count);
    // 대부분의 빔이 불일치하면 미아 가능성 — 스킵 포기(복구 증거 보존).
    if (stats.proposed_fraction > beam_skip_error_threshold_) {
        beam_skip_mask_.assign(beam_count, 0);
        stats.applied = false;
    } else {
        stats.applied = skip_count > 0;
    }
    return stats;
}

double scanScoring::outlierFraction(const particle &p, double prob_threshold) const {
    const std::size_t count = prep_range_.size();
    if (count == 0) {
        return 0.0;
    }
    const double log_threshold = std::log(std::max(1.0e-9, prob_threshold));
    const double pX = static_cast<double>(p.x);
    const double pY = static_cast<double>(p.y);
    const double cos_theta = std::cos(static_cast<double>(p.theta));
    const double sin_theta = std::sin(static_cast<double>(p.theta));
    int outliers = 0;
    int considered = 0;
    const bool has_mask = beam_skip_mask_.size() == count;
    for (std::size_t i = 0; i < count; ++i) {
        if (has_mask && beam_skip_mask_[i]) {
            continue;
        }
        const double dir_x = prep_cos_[i] * cos_theta - prep_sin_[i] * sin_theta;
        const double dir_y = prep_sin_[i] * cos_theta + prep_cos_[i] * sin_theta;
        const double world_x = pX + prep_range_[i] * dir_x;
        const double world_y = pY + prep_range_[i] * dir_y;
        const int32_t px = static_cast<int32_t>(
            std::floor((world_x - mapOriginX_) * invResolution_));
        const int32_t py = static_cast<int32_t>(
            std::floor((world_y - mapOriginY_) * invResolution_));
        if (px < 0 || px >= mapWidth_ || py < 0 || py >= mapHeight_ ||
            static_cast<double>(logProcessedMap_[
                static_cast<std::size_t>(py) * mapWidth_ + px]) < log_threshold) {
            ++outliers;
        }
        ++considered;
    }
    if (considered == 0) {
        return 0.0;
    }
    return static_cast<double>(outliers) / static_cast<double>(considered);
}