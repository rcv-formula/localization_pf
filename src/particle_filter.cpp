#include "particle_filter.h"

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

}  // namespace

ParticleFilter::ParticleFilter(const Parameters &parameters) {
    setParameters(parameters);
}

void ParticleFilter::setParameters(const Parameters &parameters) {
    parameters_ = parameters;
    parameters_.particle_count = std::max(1, parameters_.particle_count);
    parameters_.likelihood_scale = std::max(1.0e-6, parameters_.likelihood_scale);
    parameters_.resample_neff_ratio =
        std::clamp(parameters_.resample_neff_ratio, 0.0, 1.0);
    parameters_.init_position_std = std::max(0.0, parameters_.init_position_std);
    parameters_.init_yaw_std = std::max(0.0, parameters_.init_yaw_std);
    rng_.seed(parameters_.seed);
}

void ParticleFilter::reset() {
    particles_.clear();
    resample_buffer_.clear();
    count_ = 0;
    initialized_ = false;
}

void ParticleFilter::initializeAround(double x, double y, double yaw) {
    initializeAround(
        x, y, yaw,
        parameters_.init_position_std,
        parameters_.init_yaw_std);
}

void ParticleFilter::initializeAround(
    double x, double y, double yaw,
    double position_std, double yaw_std) {
    count_ = std::max(1, parameters_.particle_count);
    particles_.assign(static_cast<std::size_t>(count_), particle{});

    std::normal_distribution<double> position_noise(0.0, std::max(0.0, position_std));
    std::normal_distribution<double> yaw_noise(0.0, std::max(0.0, yaw_std));
    const float uniform_weight = 1.0f / static_cast<float>(count_);

    for (int32_t index = 0; index < count_; ++index) {
        particle &current = particles_[static_cast<std::size_t>(index)];
        current.x = static_cast<float>(x + position_noise(rng_));
        current.y = static_cast<float>(y + position_noise(rng_));
        current.theta = static_cast<float>(normalizeAngle(yaw + yaw_noise(rng_)));
        current.score[0] = 0.0f;
        current.score[1] = 0.0f;
        current.score[2] = uniform_weight;
        current.score[3] = 0.0f;
        current.score[4] = 0.0f;
        current.mode = 0;
    }
    seeded_mode_count_ = 1;
    initialized_ = true;
}

void ParticleFilter::initializeMultiple(const std::vector<ModeSeed> &seeds) {
    if (seeds.empty()) {
        return;
    }
    if (seeds.size() == 1) {
        initializeAround(seeds[0].x, seeds[0].y, seeds[0].yaw);
        return;
    }

    count_ = std::max(
        static_cast<int32_t>(seeds.size()),
        parameters_.particle_count);
    particles_.assign(static_cast<std::size_t>(count_), particle{});

    std::normal_distribution<double> position_noise(
        0.0, parameters_.init_position_std);
    std::normal_distribution<double> yaw_noise(0.0, parameters_.init_yaw_std);
    const float uniform_weight = 1.0f / static_cast<float>(count_);
    const std::size_t mode_count = seeds.size();

    // 가중 배분: 각 모드가 받을 파티클 수를 비중에 비례해 정하고, 어떤 모드도
    // 완전히 굶지 않도록 바닥(전체의 5%)을 보장합니다. 비중이 모두 같으면
    // 예전 균등 분할과 같습니다.
    std::vector<int32_t> mode_quota(mode_count, 0);
    {
        double weight_total = 0.0;
        for (const ModeSeed &seed : seeds) {
            weight_total += std::max(0.0, seed.weight);
        }
        const int32_t floor_count = std::max<int32_t>(
            1, static_cast<int32_t>(0.05 * static_cast<double>(count_)));
        int32_t assigned = 0;
        for (std::size_t mode = 0; mode < mode_count; ++mode) {
            const double share = weight_total > 0.0
                ? std::max(0.0, seeds[mode].weight) / weight_total
                : 1.0 / static_cast<double>(mode_count);
            mode_quota[mode] = std::max(
                floor_count,
                static_cast<int32_t>(share * static_cast<double>(count_)));
            assigned += mode_quota[mode];
        }
        // 반올림/바닥 보정으로 총합이 어긋나면 최대 비중 모드에서 조절합니다.
        const std::size_t dominant = static_cast<std::size_t>(
            std::max_element(
                seeds.begin(), seeds.end(),
                [](const ModeSeed &a, const ModeSeed &b) {
                    return a.weight < b.weight;
                }) - seeds.begin());
        mode_quota[dominant] += count_ - assigned;
        if (mode_quota[dominant] < 1) {
            mode_quota[dominant] = 1;
        }
    }
    std::vector<int32_t> mode_used(mode_count, 0);
    std::size_t cursor = 0;
    for (int32_t index = 0; index < count_; ++index) {
        while (cursor + 1 < mode_count &&
               mode_used[cursor] >= mode_quota[cursor]) {
            ++cursor;
        }
        const std::size_t mode = cursor;
        ++mode_used[mode];
        const ModeSeed &seed = seeds[mode];
        particle &current = particles_[static_cast<std::size_t>(index)];
        current.x = static_cast<float>(seed.x + position_noise(rng_));
        current.y = static_cast<float>(seed.y + position_noise(rng_));
        current.theta =
            static_cast<float>(normalizeAngle(seed.yaw + yaw_noise(rng_)));
        current.score[0] = 0.0f;
        current.score[1] = 0.0f;
        current.score[2] = uniform_weight;
        current.score[3] = 0.0f;
        current.score[4] = 0.0f;
        current.mode = static_cast<uint16_t>(mode);
    }
    seeded_mode_count_ = static_cast<uint16_t>(mode_count);
    initialized_ = true;
}

ParticleFilter::ModeSummary ParticleFilter::modeSummary() const {
    ModeSummary summary;
    if (count_ <= 0 || seeded_mode_count_ == 0) {
        return summary;
    }
    std::vector<double> masses(static_cast<std::size_t>(seeded_mode_count_), 0.0);
    for (int32_t index = 0; index < count_; ++index) {
        const particle &current = particles_[static_cast<std::size_t>(index)];
        const double weight = static_cast<double>(current.score[2]);
        if (current.mode < masses.size() &&
            std::isfinite(weight) && weight > 0.0) {
            masses[current.mode] += weight;
        }
    }
    double best = -1.0;
    for (std::size_t mode = 0; mode < masses.size(); ++mode) {
        if (masses[mode] > 1.0e-3) {
            ++summary.alive_modes;
        }
        if (masses[mode] > best) {
            best = masses[mode];
            summary.dominant = static_cast<uint16_t>(mode);
        }
    }
    summary.dominant_mass = std::max(0.0, best);
    return summary;
}

ParticleFilter::Estimate ParticleFilter::estimateOfMode(uint16_t mode) const {
    return estimateFiltered(static_cast<int>(mode));
}

double ParticleFilter::normalizeWeights() {
    if (count_ <= 0) {
        return 0.0;
    }

    // score[0]은 빔당 평균 log-likelihood(음수)입니다. 최대값을 빼고 지수화해
    // 오버/언더플로를 막습니다(log-sum-exp).
    double max_log = -std::numeric_limits<double>::infinity();
    for (int32_t index = 0; index < count_; ++index) {
        const double value =
            static_cast<double>(particles_[static_cast<std::size_t>(index)].score[0]);
        if (std::isfinite(value) && value > max_log) {
            max_log = value;
        }
    }

    const float uniform_weight = 1.0f / static_cast<float>(count_);
    // 유효한 관측이 하나도 없으면 측정으로 구분할 수 없으므로 균등하게 둡니다.
    if (!std::isfinite(max_log)) {
        for (int32_t index = 0; index < count_; ++index) {
            particles_[static_cast<std::size_t>(index)].score[2] = uniform_weight;
        }
        return static_cast<double>(count_);
    }

    double sum = 0.0;
    for (int32_t index = 0; index < count_; ++index) {
        particle &current = particles_[static_cast<std::size_t>(index)];
        const double value = static_cast<double>(current.score[0]);
        // exp(-40) ~ 4e-18은 어차피 0으로 소실되므로 exp 호출을 생략합니다
        // (결과 동일, 가중이 몰릴수록 exp 호출 수가 급감).
        const double exponent = std::isfinite(value) ?
            parameters_.likelihood_scale * (value - max_log) : -1.0e9;
        const double weight = exponent > -40.0 ? std::exp(exponent) : 0.0;
        current.score[2] = static_cast<float>(weight);
        sum += weight;
    }

    if (!(sum > 0.0) || !std::isfinite(sum)) {
        for (int32_t index = 0; index < count_; ++index) {
            particles_[static_cast<std::size_t>(index)].score[2] = uniform_weight;
        }
        return static_cast<double>(count_);
    }

    const double inverse_sum = 1.0 / sum;
    for (int32_t index = 0; index < count_; ++index) {
        particle &current = particles_[static_cast<std::size_t>(index)];
        current.score[2] = static_cast<float>(
            static_cast<double>(current.score[2]) * inverse_sum);
    }
    return effectiveSampleSize();
}

double ParticleFilter::effectiveSampleSize() const {
    if (count_ <= 0) {
        return 0.0;
    }
    double sum_squared = 0.0;
    for (int32_t index = 0; index < count_; ++index) {
        const double weight =
            static_cast<double>(particles_[static_cast<std::size_t>(index)].score[2]);
        if (std::isfinite(weight) && weight > 0.0) {
            sum_squared += weight * weight;
        }
    }
    if (!(sum_squared > 0.0)) {
        return 0.0;
    }
    return 1.0 / sum_squared;
}

void ParticleFilter::resample() {
    if (count_ <= 0) {
        return;
    }

    // 저분산(systematic) 리샘플링: 난수 하나로 등간격 표본을 뽑아 다항추출보다
    // 분산이 작고, 가중이 균등하면 원본을 그대로 보존합니다.
    resample_buffer_.resize(static_cast<std::size_t>(count_));
    const double step = 1.0 / static_cast<double>(count_);
    std::uniform_real_distribution<double> start_noise(0.0, step);
    const double start = start_noise(rng_);

    int32_t source = 0;
    double cumulative =
        static_cast<double>(particles_[0].score[2]);
    for (int32_t target_index = 0; target_index < count_; ++target_index) {
        const double target = start + static_cast<double>(target_index) * step;
        while (target > cumulative && source + 1 < count_) {
            ++source;
            cumulative +=
                static_cast<double>(particles_[static_cast<std::size_t>(source)].score[2]);
        }
        resample_buffer_[static_cast<std::size_t>(target_index)] =
            particles_[static_cast<std::size_t>(source)];
        // 리샘플 후에는 모든 파티클이 같은 비중을 가집니다.
        resample_buffer_[static_cast<std::size_t>(target_index)].score[2] =
            static_cast<float>(step);
    }
    particles_.swap(resample_buffer_);
}

bool ParticleFilter::resampleIfNeeded() {
    if (count_ <= 0) {
        return false;
    }
    const double threshold =
        parameters_.resample_neff_ratio * static_cast<double>(count_);
    if (effectiveSampleSize() >= threshold) {
        return false;
    }
    resample();
    return true;
}

ParticleFilter::Estimate ParticleFilter::estimate() const {
    return estimateFiltered(-1);
}

ParticleFilter::Estimate ParticleFilter::estimateFiltered(int mode) const {
    Estimate result;
    if (count_ <= 0) {
        return result;
    }

    double weight_sum = 0.0;
    double mean_x = 0.0;
    double mean_y = 0.0;
    double yaw_cos = 0.0;
    double yaw_sin = 0.0;
    for (int32_t index = 0; index < count_; ++index) {
        const particle &current = particles_[static_cast<std::size_t>(index)];
        const double weight = static_cast<double>(current.score[2]);
        if (!std::isfinite(weight) || weight <= 0.0) {
            continue;
        }
        if (mode >= 0 && current.mode != static_cast<uint16_t>(mode)) {
            continue;
        }
        weight_sum += weight;
        mean_x += weight * static_cast<double>(current.x);
        mean_y += weight * static_cast<double>(current.y);
        // yaw는 wrap이 있으므로 산술평균이 아니라 원형평균을 씁니다.
        yaw_cos += weight * std::cos(static_cast<double>(current.theta));
        yaw_sin += weight * std::sin(static_cast<double>(current.theta));
    }

    if (!(weight_sum > 0.0)) {
        return result;
    }

    const double inverse_sum = 1.0 / weight_sum;
    result.x = mean_x * inverse_sum;
    result.y = mean_y * inverse_sum;
    result.yaw = std::atan2(yaw_sin, yaw_cos);

    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    double yaw_variance = 0.0;
    for (int32_t index = 0; index < count_; ++index) {
        const particle &current = particles_[static_cast<std::size_t>(index)];
        const double weight = static_cast<double>(current.score[2]);
        if (!std::isfinite(weight) || weight <= 0.0) {
            continue;
        }
        if (mode >= 0 && current.mode != static_cast<uint16_t>(mode)) {
            continue;
        }
        const double offset_x = static_cast<double>(current.x) - result.x;
        const double offset_y = static_cast<double>(current.y) - result.y;
        covariance(0, 0) += weight * offset_x * offset_x;
        covariance(0, 1) += weight * offset_x * offset_y;
        covariance(1, 1) += weight * offset_y * offset_y;
        const double offset_yaw =
            normalizeAngle(static_cast<double>(current.theta) - result.yaw);
        yaw_variance += weight * offset_yaw * offset_yaw;
    }
    covariance *= inverse_sum;
    covariance(1, 0) = covariance(0, 1);

    result.position_covariance = covariance;
    result.yaw_variance = yaw_variance * inverse_sum;
    result.effective_sample_size = effectiveSampleSize();
    result.valid = true;
    return result;
}
