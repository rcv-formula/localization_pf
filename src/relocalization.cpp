#include "relocalization.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include "includes/RangeLib.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kHalfPi = 0.5 * kPi;

// 최종 pose의 yaw를 [-pi, pi] 범위로 유지합니다.
double normalizeAngle(double angle) {
    while (angle > kPi) {
        angle -= kTwoPi;
    }
    while (angle < -kPi) {
        angle += kTwoPi;
    }
    return angle;
}

// ray 인덱스 계산을 위해 각도를 [0, 2*pi) 범위로 바꿉니다.
double normalizePositiveAngle(double angle) {
    angle = std::fmod(angle, kTwoPi);
    if (angle < 0.0) {
        angle += kTwoPi;
    }
    return angle;
}

// undirected 축은 주기가 pi이므로 대표값을 (-pi/2, pi/2]로 통일합니다.
double canonicalizeAxis(double angle) {
    angle = std::fmod(angle, kPi);
    if (angle <= -kHalfPi) {
        angle += kPi;
    } else if (angle > kHalfPi) {
        angle -= kPi;
    }
    return angle;
}

// RangeLibc의 numpy_calc_range_angles는 내부에서 calc_range(y, x, ...)로 호출하고
// 경계검사도 첫 인자를 map.width와 비교합니다. 즉 OMap의 첫 축이 row여야 하며,
// upstream pywrapper(RangeLibc.pyx)와 같은 OMap(height, width) + grid[row][col]
// 규약으로 적재해야 합니다. 이 규약을 두 군데에서 각각 쓰면 한쪽만 틀려도
// 서로 상쇄되어 발견이 어려우므로, 맵 적재는 이 함수 한 곳으로 모읍니다.
ranges::OMap buildRangeMap(
    const nav_msgs::msg::OccupancyGrid &map,
    int occupied_threshold,
    bool unknown_is_occupied,
    double origin_yaw) {
    ranges::OMap range_map(
        static_cast<int>(map.info.height),
        static_cast<int>(map.info.width));
    for (int32_t cell_y = 0; cell_y < static_cast<int32_t>(map.info.height); ++cell_y) {
        for (int32_t cell_x = 0; cell_x < static_cast<int32_t>(map.info.width); ++cell_x) {
            const int8_t value = map.data[
                static_cast<std::size_t>(cell_y) * map.info.width +
                static_cast<std::size_t>(cell_x)];
            range_map.grid[cell_y][cell_x] =
                value < 0 ? unknown_is_occupied : value >= occupied_threshold;
        }
    }

    // RangeLibc의 world-to-grid 변환에 map origin과 방향을 전달합니다.
    const float map_angle = static_cast<float>(-origin_yaw);
    range_map.world_scale = static_cast<float>(map.info.resolution);
    range_map.world_angle = map_angle;
    range_map.world_origin_x = static_cast<float>(map.info.origin.position.x);
    range_map.world_origin_y = static_cast<float>(map.info.origin.position.y);
    range_map.world_sin_angle = std::sin(map_angle);
    range_map.world_cos_angle = std::cos(map_angle);
    return range_map;
}

}  // namespace

// 기본 파라미터를 사용하는 편의 생성자입니다.
Relocalization::Relocalization(const nav_msgs::msg::OccupancyGrid &map)
    : Relocalization(map, Parameters{}) {}

Relocalization::Relocalization(
    const nav_msgs::msg::OccupancyGrid &map,
    const Parameters &parameters)
    : parameters_(parameters) {
    validateParameters(parameters_);
    setMap(map);
}

// 원본 맵을 교체하면 이전 사전계산 결과를 버리고 다시 생성합니다.
void Relocalization::setMap(const nav_msgs::msg::OccupancyGrid &map) {
    map_ = map;
    preprocessMap();
}

// ray 수나 최대거리처럼 사전계산 모양을 바꾸는 값이 달라졌을 때만 맵을 다시
// 계산합니다. 탐색 단계에서만 쓰이는 값(허용오차, coarse-to-fine 등)은 즉시
// 반영되므로 파라미터 스윕에서 수 초짜리 재계산을 반복하지 않습니다.
void Relocalization::setParameters(const Parameters &parameters) {
    validateParameters(parameters);

    const bool rebuild =
        !map_ready_ ||
        parameters.occupied_threshold != parameters_.occupied_threshold ||
        parameters.unknown_is_occupied != parameters_.unknown_is_occupied ||
        parameters.ray_count != parameters_.ray_count ||
        parameters.max_range != parameters_.max_range ||
        parameters.fov_deg != parameters_.fov_deg ||
        parameters.axis_run_tolerance_deg != parameters_.axis_run_tolerance_deg;
    parameters_ = parameters;

    if (rebuild && !map_.data.empty()) {
        preprocessMap();
    }
}

geometry_msgs::msg::Pose Relocalization::relocalize(
    const sensor_msgs::msg::LaserScan &scan) {
    if (!map_ready_) {
        throw std::logic_error("Relocalization map features are not ready.");
    }

    // 스캔을 heading 격자에 접어 두고 주축까지 한 번에 구합니다.
    const ScanView view = convertScan(scan);
    if (view.size() < parameters_.minimum_scan_points) {
        throw std::invalid_argument("Relocalization scan has too few valid points.");
    }

    const SearchResult result = searchGlobalPose(view);
    diagnostics_.best_score = result.best.score;
    diagnostics_.scan_points = view.size();
    if (!result.valid) {
        throw std::runtime_error("Relocalization found no valid map candidate.");
    }

    return buildPose(result);
}

std::vector<Relocalization::Hypothesis> Relocalization::relocalizeMultiple(
    const sensor_msgs::msg::LaserScan &scan) {
    if (!map_ready_) {
        throw std::logic_error("Relocalization map features are not ready.");
    }

    const ScanView view = convertScan(scan);
    if (view.size() < parameters_.minimum_scan_points) {
        throw std::invalid_argument("Relocalization scan has too few valid points.");
    }

    std::vector<PoseCandidate> pool;
    pool.reserve(4096);
    const SearchResult result = searchGlobalPose(view, &pool);
    diagnostics_.best_score = result.best.score;
    diagnostics_.scan_points = view.size();
    if (!result.valid || pool.empty()) {
        throw std::runtime_error("Relocalization found no valid map candidate.");
    }

    // 점수 내림차순으로 훑으며 비최대 억제(NMS)로 가설을 뽑습니다.
    std::sort(
        pool.begin(), pool.end(),
        [](const PoseCandidate &a, const PoseCandidate &b) {
            return a.score > b.score;
        });
    return selectHypotheses(pool, result.best.score, &view);
}

double Relocalization::inlierFraction(
    double x, double y, double yaw, const ScanView &view) const {
    const int width = static_cast<int>(processed_map_.width);
    const int height = static_cast<int>(processed_map_.height);
    const int cell_x = static_cast<int>(std::floor(
        (x - processed_map_.origin_x) / processed_map_.resolution));
    const int cell_y = static_cast<int>(std::floor(
        (y - processed_map_.origin_y) / processed_map_.resolution));
    if (cell_x < 0 || cell_y < 0 || cell_x >= width || cell_y >= height) {
        return 0.0;
    }
    const int32_t candidate_index = processed_map_.cell_to_candidate[
        static_cast<std::size_t>(cell_y) * width + static_cast<std::size_t>(cell_x)];
    if (candidate_index < 0 || view.size() == 0) {
        return 0.0;
    }
    const int32_t ray_count = static_cast<int32_t>(parameters_.ray_count);
    const double ray_step = kTwoPi / static_cast<double>(ray_count);
    int32_t yaw_index = static_cast<int32_t>(std::llround(yaw / ray_step)) % ray_count;
    if (yaw_index < 0) {
        yaw_index += ray_count;
    }
    const float *profile = processed_map_.ranges.row(
        static_cast<Eigen::Index>(candidate_index)).data();
    const int32_t *base = view.base_index.data();
    const float *observed = view.range.data();
    int inliers = 0;
    for (std::size_t i = 0; i < view.size(); ++i) {
        int32_t index = base[i] + yaw_index;
        if (index >= ray_count) {
            index -= ray_count;
        }
        if (std::abs(static_cast<double>(observed[i]) -
                     static_cast<double>(profile[index])) <
            parameters_.hypothesis_verify_inlier_m) {
            ++inliers;
        }
    }
    return static_cast<double>(inliers) / static_cast<double>(view.size());
}

std::vector<Relocalization::Hypothesis> Relocalization::selectHypotheses(
    const std::vector<PoseCandidate> &pool,
    double best_score,
    const ScanView *verify_view) const {
    // 위치/각도 둘 다 가까울 때만 같은 가설로 흡수하므로, 같은 위치의
    // 180도 플립 후보는 별도 가설로 살아남습니다.
    // 점수는 음의 평균 오차라 best에 배율을 곱하면 하한이 됩니다.
    const double score_floor =
        best_score * std::max(1.0, parameters_.hypothesis_score_ratio);
    const double min_separation = parameters_.hypothesis_min_separation_m;
    const double min_angle =
        parameters_.hypothesis_min_separation_deg * kPi / 180.0;

    std::vector<Hypothesis> hypotheses;
    hypotheses.reserve(parameters_.max_hypotheses);
    for (const PoseCandidate &item : pool) {
        if (hypotheses.size() >= parameters_.max_hypotheses ||
            item.score < score_floor) {
            break;
        }
        bool absorbed = false;
        for (const Hypothesis &kept : hypotheses) {
            const double distance = std::hypot(item.x - kept.x, item.y - kept.y);
            const double angle =
                std::abs(normalizeAngle(item.yaw - kept.yaw));
            if (distance < min_separation && angle < min_angle) {
                absorbed = true;
                break;
            }
        }
        if (absorbed) {
            continue;
        }
        // 검증 게이트: 예측 프로파일 인라이어 비율 미달 후보는 탈락.
        // 통과 후보에는 비율을 실어 상위에서 재사용할 수 있게 합니다.
        double inlier = 0.0;
        if (verify_view != nullptr) {
            inlier = inlierFraction(item.x, item.y, item.yaw, *verify_view);
            if (parameters_.hypothesis_verify_fraction > 0.0 &&
                inlier < parameters_.hypothesis_verify_fraction) {
                continue;
            }
        }
        hypotheses.push_back(Hypothesis{item.x, item.y, item.yaw, item.score, inlier});
    }
    return hypotheses;
}

std::vector<Relocalization::RelativeMotion> Relocalization::accumulateOffsets(
    const std::vector<RelativeMotion> &motions) const {
    // T_k = T_{k+1} ∘ inv(motion_k). 마지막(최신) 오프셋은 항등입니다.
    std::vector<RelativeMotion> offsets(motions.size() + 1);
    offsets.back() = RelativeMotion{};
    for (std::size_t k = motions.size(); k-- > 0;) {
        const RelativeMotion &m = motions[k];
        const RelativeMotion &next = offsets[k + 1];
        const double inv_cos = std::cos(-m.dyaw);
        const double inv_sin = std::sin(-m.dyaw);
        const double ix = -(inv_cos * m.dx - inv_sin * m.dy);
        const double iy = -(inv_sin * m.dx + inv_cos * m.dy);
        const double next_cos = std::cos(next.dyaw);
        const double next_sin = std::sin(next.dyaw);
        offsets[k].dx = next.dx + next_cos * ix - next_sin * iy;
        offsets[k].dy = next.dy + next_sin * ix + next_cos * iy;
        offsets[k].dyaw = next.dyaw - m.dyaw;
    }
    return offsets;
}

double Relocalization::scorePoseWithProfile(
    double x, double y, double yaw, const ScanView &view) const {
    // 후보(=free) 셀 밖이면 큰 페널티. 궤적/이력 전체가 들어맞는 배치만
    // 살아남게 하는 장치입니다.
    constexpr double kMissingPenalty = -50.0;
    if (view.size() < parameters_.minimum_scan_points) {
        return kMissingPenalty;
    }
    const int width = static_cast<int>(processed_map_.width);
    const int height = static_cast<int>(processed_map_.height);
    const int cell_x = static_cast<int>(std::floor(
        (x - processed_map_.origin_x) / processed_map_.resolution));
    const int cell_y = static_cast<int>(std::floor(
        (y - processed_map_.origin_y) / processed_map_.resolution));
    if (cell_x < 0 || cell_y < 0 || cell_x >= width || cell_y >= height) {
        return kMissingPenalty;
    }
    const int32_t candidate_index = processed_map_.cell_to_candidate[
        static_cast<std::size_t>(cell_y) * width +
        static_cast<std::size_t>(cell_x)];
    if (candidate_index < 0) {
        return kMissingPenalty;
    }
    const int32_t ray_count = static_cast<int32_t>(parameters_.ray_count);
    const double ray_step = kTwoPi / static_cast<double>(ray_count);
    int32_t yaw_index = static_cast<int32_t>(std::llround(yaw / ray_step)) %
        ray_count;
    if (yaw_index < 0) {
        yaw_index += ray_count;
    }
    return scoreCandidate(
        processed_map_.ranges.row(
            static_cast<Eigen::Index>(candidate_index)).data(),
        view, yaw_index);
}

std::vector<Relocalization::Hypothesis> Relocalization::relocalizeMultiple(
    const std::vector<sensor_msgs::msg::LaserScan> &scans,
    const std::vector<RelativeMotion> &motions) {
    if (scans.empty()) {
        throw std::invalid_argument("relocalizeMultiple needs at least one scan.");
    }
    if (motions.size() + 1 != scans.size()) {
        throw std::invalid_argument(
            "relocalizeMultiple needs exactly scans-1 relative motions.");
    }
    if (scans.size() == 1) {
        return relocalizeMultiple(scans.back());
    }
    if (!map_ready_) {
        throw std::logic_error("Relocalization map features are not ready.");
    }

    // 1) 최신 스캔으로 후보 pool을 만듭니다(기존 단일 스캔 탐색 재사용).
    const ScanView latest = convertScan(scans.back());
    if (latest.size() < parameters_.minimum_scan_points) {
        throw std::invalid_argument("Relocalization scan has too few valid points.");
    }
    std::vector<PoseCandidate> pool;
    pool.reserve(4096);
    const SearchResult single = searchGlobalPose(latest, &pool);
    if (!single.valid || pool.empty()) {
        throw std::runtime_error("Relocalization found no valid map candidate.");
    }

    // 2) 점수순 + 느슨한 NMS로 공동 채점 대상 shortlist를 다양하게 뽑습니다.
    std::sort(
        pool.begin(), pool.end(),
        [](const PoseCandidate &a, const PoseCandidate &b) {
            return a.score > b.score;
        });
    const double shortlist_separation =
        0.5 * parameters_.hypothesis_min_separation_m;
    const double shortlist_angle =
        0.5 * parameters_.hypothesis_min_separation_deg * kPi / 180.0;
    std::vector<PoseCandidate> shortlist;
    shortlist.reserve(parameters_.multi_scan_top_candidates);
    for (const PoseCandidate &item : pool) {
        if (shortlist.size() >= parameters_.multi_scan_top_candidates) {
            break;
        }
        bool absorbed = false;
        for (const PoseCandidate &kept : shortlist) {
            if (std::hypot(item.x - kept.x, item.y - kept.y) < shortlist_separation &&
                std::abs(normalizeAngle(item.yaw - kept.yaw)) < shortlist_angle) {
                absorbed = true;
                break;
            }
        }
        if (!absorbed) {
            shortlist.push_back(item);
        }
    }

    // 3) 과거 스캔 뷰와, 최신 라이다 프레임에서 본 과거 라이다 오프셋을
    //    준비합니다. T_k = T_{k+1} ∘ inv(motion_k).
    const std::size_t scan_count = scans.size();
    std::vector<ScanView> views;
    views.reserve(scan_count - 1);
    for (std::size_t k = 0; k + 1 < scan_count; ++k) {
        views.push_back(convertScan(scans[k]));
    }
    const std::vector<RelativeMotion> offsets = accumulateOffsets(motions);

    // 4) 공동 채점: 후보 pose에서 역산한 과거 pose마다 그 위치의 사전계산
    //    프로파일로 해당 과거 스캔을 채점합니다. 과거 pose가 free 셀을
    //    벗어나면(벽/unknown) 큰 페널티 — 닮은꼴 루프라도 궤적 전체가
    //    들어맞지 않으면 여기서 탈락합니다.
    for (PoseCandidate &item : shortlist) {
        double total = item.score;
        int used = 1;
        const double cos_yaw = std::cos(item.yaw);
        const double sin_yaw = std::sin(item.yaw);
        for (std::size_t k = 0; k + 1 < scan_count; ++k) {
            if (views[k].size() < parameters_.minimum_scan_points) {
                continue;
            }
            total += scorePoseWithProfile(
                item.x + cos_yaw * offsets[k].dx - sin_yaw * offsets[k].dy,
                item.y + sin_yaw * offsets[k].dx + cos_yaw * offsets[k].dy,
                item.yaw + offsets[k].dyaw,
                views[k]);
            ++used;
        }
        item.score = total / static_cast<double>(used);
    }

    // 5) 공동 점수로 재정렬하고 표준 NMS로 가설을 반환합니다.
    std::sort(
        shortlist.begin(), shortlist.end(),
        [](const PoseCandidate &a, const PoseCandidate &b) {
            return a.score > b.score;
        });
    diagnostics_.best_score = shortlist.front().score;
    diagnostics_.scan_points = latest.size();
    return selectHypotheses(shortlist, shortlist.front().score, &latest);
}

std::vector<Relocalization::Hypothesis> Relocalization::relocalizeTrajectory(
    const std::vector<std::array<double, 2>> &points,
    double current_yaw,
    const std::vector<sensor_msgs::msg::LaserScan> &scans,
    const std::vector<RelativeMotion> &motions) {
    if (scans.empty() || motions.size() + 1 != scans.size()) {
        throw std::invalid_argument(
            "relocalizeTrajectory needs scans and scans-1 motions.");
    }
    if (!map_ready_) {
        throw std::logic_error("Relocalization map features are not ready.");
    }
    if (points.size() < 8) {
        throw std::invalid_argument("relocalizeTrajectory needs a longer trajectory.");
    }

    const int width = static_cast<int>(processed_map_.width);
    const int height = static_cast<int>(processed_map_.height);
    const double resolution = processed_map_.resolution;
    const double origin_x = processed_map_.origin_x;
    const double origin_y = processed_map_.origin_y;
    const auto &cell_to_candidate = processed_map_.cell_to_candidate;
    const auto free_at = [&](double wx, double wy) -> bool {
        const int cx = static_cast<int>(std::floor((wx - origin_x) / resolution));
        const int cy = static_cast<int>(std::floor((wy - origin_y) / resolution));
        if (cx < 0 || cy < 0 || cx >= width || cy >= height) {
            return false;
        }
        return cell_to_candidate[
            static_cast<std::size_t>(cy) * width + static_cast<std::size_t>(cx)] >= 0;
    };

    // 마지막 점(현재 위치)을 원점으로 옮겨 회전 중심을 현재 pose에 둡니다.
    // 이렇게 하면 (rot, tx, ty)의 (tx, ty)가 곧 현재 위치 가설이 됩니다.
    const double pivot_x = points.back()[0];
    const double pivot_y = points.back()[1];
    std::vector<std::array<double, 2>> local(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        local[i] = {points[i][0] - pivot_x, points[i][1] - pivot_y};
    }

    struct Fit {
        double free_ratio{-1.0};
        double rot{0.0};
        double x{0.0};
        double y{0.0};
    };
    const double rot_step = std::max(0.5, parameters_.trajectory_rot_step_deg);
    const double step = std::max(0.05, parameters_.trajectory_translation_step_m);
    const int rot_count = static_cast<int>(std::ceil(360.0 / rot_step));
    std::vector<Fit> merged;
    std::mutex merge_mutex;
    // 실차 기기 대비: 전역 정합도 스레드 4개 상한.
    const unsigned worker_count =
        std::max(1u, std::min(4u, std::thread::hardware_concurrency()));
    std::vector<std::thread> workers;
    workers.reserve(worker_count);
    for (unsigned worker = 0; worker < worker_count; ++worker) {
        workers.emplace_back([&, worker]() {
            std::vector<Fit> top;
            for (int rot_index = static_cast<int>(worker); rot_index < rot_count;
                 rot_index += static_cast<int>(worker_count)) {
                const double rot = rot_index * rot_step * kPi / 180.0;
                const double cos_rot = std::cos(rot);
                const double sin_rot = std::sin(rot);
                std::vector<std::array<double, 2>> rotated(local.size());
                for (std::size_t i = 0; i < local.size(); ++i) {
                    rotated[i] = {
                        cos_rot * local[i][0] - sin_rot * local[i][1],
                        sin_rot * local[i][0] + cos_rot * local[i][1]};
                }
                for (double tx = origin_x; tx < origin_x + width * resolution;
                     tx += step) {
                    for (double ty = origin_y; ty < origin_y + height * resolution;
                         ty += step) {
                        int hit = 0;
                        for (const auto &p : rotated) {
                            hit += free_at(p[0] + tx, p[1] + ty) ? 1 : 0;
                        }
                        const double ratio =
                            static_cast<double>(hit) /
                            static_cast<double>(rotated.size());
                        if (ratio > 0.6) {
                            top.push_back(Fit{
                                ratio, rot_index * rot_step, tx, ty});
                        }
                    }
                }
            }
            std::lock_guard<std::mutex> guard(merge_mutex);
            merged.insert(merged.end(), top.begin(), top.end());
        });
    }
    for (auto &worker : workers) {
        worker.join();
    }
    if (merged.empty()) {
        throw std::runtime_error("Trajectory fit found no on-track placement.");
    }

    std::sort(
        merged.begin(), merged.end(),
        [](const Fit &a, const Fit &b) { return a.free_ratio > b.free_ratio; });

    // NMS(현재 pose 기준 1.0 m / 30도)로 다양화한 뒤 국소 정밀화합니다.
    std::vector<Fit> survivors;
    const double best_ratio = merged.front().free_ratio;
    for (const Fit &item : merged) {
        if (survivors.size() >= parameters_.trajectory_keep ||
            item.free_ratio < best_ratio - parameters_.trajectory_score_band) {
            break;
        }
        bool absorbed = false;
        for (const Fit &kept : survivors) {
            const double rot_gap = std::abs(normalizeAngle(
                (item.rot - kept.rot) * kPi / 180.0));
            if (std::hypot(item.x - kept.x, item.y - kept.y) < 1.0 &&
                rot_gap < 30.0 * kPi / 180.0) {
                absorbed = true;
                break;
            }
        }
        if (!absorbed) {
            survivors.push_back(item);
        }
    }
    const auto ratio_at = [&](double rot_deg, double tx, double ty) -> double {
        const double rot = rot_deg * kPi / 180.0;
        const double cos_rot = std::cos(rot);
        const double sin_rot = std::sin(rot);
        int hit = 0;
        for (const auto &p : local) {
            hit += free_at(
                cos_rot * p[0] - sin_rot * p[1] + tx,
                sin_rot * p[0] + cos_rot * p[1] + ty) ? 1 : 0;
        }
        return static_cast<double>(hit) / static_cast<double>(local.size());
    };
    for (Fit &item : survivors) {
        Fit best = item;
        for (double rot = item.rot - rot_step; rot <= item.rot + rot_step + 1e-9;
             rot += 1.0) {
            for (double tx = item.x - step; tx <= item.x + step + 1e-9; tx += 0.05) {
                for (double ty = item.y - step; ty <= item.y + step + 1e-9;
                     ty += 0.05) {
                    const double ratio = ratio_at(rot, tx, ty);
                    if (ratio > best.free_ratio) {
                        best = Fit{ratio, rot, tx, ty};
                    }
                }
            }
        }
        item = best;
    }

    // 타이브레이크: free 비율은 배치의 물리 타당성을 책임지고, 루프 위상
    // (루프를 따라 어디인지)은 코너를 포함한 스캔 이력의 multi-scan 공동
    // 채점이 가립니다. 현재 스캔 한 장으로는 낡은 맵/주행 중 왜곡 때문에
    // 위상 판별력이 부족합니다(0526-1/0529 실측).
    std::vector<ScanView> views;
    views.reserve(scans.size());
    for (const auto &item : scans) {
        views.push_back(convertScan(item));
    }
    const std::vector<RelativeMotion> offsets = accumulateOffsets(motions);
    std::vector<Hypothesis> hypotheses;
    hypotheses.reserve(survivors.size());
    for (const Fit &item : survivors) {
        Hypothesis h;
        h.x = item.x;
        h.y = item.y;
        h.yaw = normalizeAngle(current_yaw + item.rot * kPi / 180.0);
        const double cos_yaw = std::cos(h.yaw);
        const double sin_yaw = std::sin(h.yaw);
        double total = 0.0;
        int used = 0;
        for (std::size_t k = 0; k < views.size(); ++k) {
            if (views[k].size() < parameters_.minimum_scan_points) {
                continue;
            }
            total += scorePoseWithProfile(
                h.x + cos_yaw * offsets[k].dx - sin_yaw * offsets[k].dy,
                h.y + sin_yaw * offsets[k].dx + cos_yaw * offsets[k].dy,
                h.yaw + offsets[k].dyaw,
                views[k]);
            ++used;
        }
        h.score = used > 0 ? total / used : -50.0;
        hypotheses.push_back(h);
    }
    std::sort(
        hypotheses.begin(), hypotheses.end(),
        [](const Hypothesis &a, const Hypothesis &b) {
            return a.score > b.score;
        });
    diagnostics_.best_score = hypotheses.empty() ? 0.0 : hypotheses.front().score;
    diagnostics_.scan_points = views.back().size();
    // 궤적 정합 가설에도 같은 검증 게이트: 통과 후보만 시드가 됩니다.
    for (Hypothesis &h : hypotheses) {
        h.inlier = inlierFraction(h.x, h.y, h.yaw, views.back());
    }
    if (parameters_.hypothesis_verify_fraction > 0.0) {
        std::vector<Hypothesis> gated;
        gated.reserve(hypotheses.size());
        for (const Hypothesis &h : hypotheses) {
            if (h.inlier >= parameters_.hypothesis_verify_fraction) {
                gated.push_back(h);
            }
        }
        hypotheses = std::move(gated);
    }
    return hypotheses;
}

geometry_msgs::msg::Pose Relocalization::relocalize(
    const std::vector<sensor_msgs::msg::LaserScan> &scans,
    const std::vector<geometry_msgs::msg::Transform> &current_from_next) {
    (void)scans;
    (void)current_from_next;
    throw std::logic_error("Multi-scan relocalization is not implemented yet.");
}

std::size_t Relocalization::fovWindow() const {
    return std::min(
        parameters_.ray_count,
        std::max<std::size_t>(3,
            static_cast<std::size_t>(std::llround(
                parameters_.fov_deg / 360.0 *
                static_cast<double>(parameters_.ray_count)))));
}

Relocalization::StaticGeometry Relocalization::queryStaticGeometry(
    double world_x,
    double world_y,
    double heading) const {
    StaticGeometry result;
    if (!map_ready_ || processed_map_.candidates.empty()) {
        return result;
    }

    // world -> map origin 로컬 -> cell 인덱스로 역변환합니다.
    const double cos_yaw = std::cos(processed_map_.origin_yaw);
    const double sin_yaw = std::sin(processed_map_.origin_yaw);
    const double delta_x = world_x - processed_map_.origin_x;
    const double delta_y = world_y - processed_map_.origin_y;
    const double local_x = cos_yaw * delta_x + sin_yaw * delta_y;
    const double local_y = -sin_yaw * delta_x + cos_yaw * delta_y;
    const int width = static_cast<int>(processed_map_.width);
    const int height = static_cast<int>(processed_map_.height);
    const int base_x = static_cast<int>(
        std::llround(local_x / processed_map_.resolution - 0.5));
    const int base_y = static_cast<int>(
        std::llround(local_y / processed_map_.resolution - 0.5));

    // 요청 셀이 후보가 아니면(장애물/unknown) 반경을 넓혀 가장 가까운 후보를 찾습니다.
    int32_t candidate_index = -1;
    const int max_search_radius = 16;
    for (int radius = 0; radius <= max_search_radius && candidate_index < 0; ++radius) {
        for (int dy = -radius; dy <= radius && candidate_index < 0; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                // 이미 살펴본 안쪽은 건너뛰고 테두리만 확인합니다.
                if (radius > 0 &&
                    std::abs(dx) != radius && std::abs(dy) != radius) {
                    continue;
                }
                const int nx = base_x + dx;
                const int ny = base_y + dy;
                if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                    continue;
                }
                const int32_t found = processed_map_.cell_to_candidate[
                    static_cast<std::size_t>(ny) * width + static_cast<std::size_t>(nx)];
                if (found >= 0) {
                    candidate_index = found;
                    break;
                }
            }
        }
    }
    if (candidate_index < 0) {
        return result;
    }

    const CandidateLocation &candidate =
        processed_map_.candidates[static_cast<std::size_t>(candidate_index)];
    result.source_x = candidate.world_x;
    result.source_y = candidate.world_y;

    const float *profile = processed_map_.ranges.row(
        static_cast<Eigen::Index>(candidate_index)).data();
    const std::size_t ray_count = parameters_.ray_count;
    const double ray_step = kTwoPi / static_cast<double>(ray_count);
    const double max_hit_range = parameters_.max_range -
        std::max(processed_map_.resolution * 0.5, 1.0e-3);

    // 360도 원본을 heading 중심 fov 창으로 자릅니다. 센서가 실제로 보는 빔만
    // 관측성에 기여해야 하므로 뒤쪽은 제외합니다.
    const int n = static_cast<int>(ray_count);
    const int win = static_cast<int>(fovWindow());
    const int center = static_cast<int>(
        std::llround(normalizePositiveAngle(heading) / ray_step)) % n;
    const int start = ((center - win / 2) % n + n) % n;

    // 창 안 빔의 끝점을 먼저 펼쳐 둡니다. 법선 추정 창이 빔마다 달라서
    // 러닝합을 쓸 수 없으므로 좌표를 재사용할 수 있게 모아 둡니다.
    static thread_local std::vector<double> wx, wy;
    static thread_local std::vector<uint8_t> whit;
    wx.assign(static_cast<std::size_t>(win), 0.0);
    wy.assign(static_cast<std::size_t>(win), 0.0);
    whit.assign(static_cast<std::size_t>(win), 0);
    for (int k = 0; k < win; ++k) {
        const int i = (start + k) % n;
        const double r = profile[i];
        if (!std::isfinite(r) || r <= 0.0 || r >= max_hit_range) {
            continue;
        }
        wx[k] = r * processed_map_.ray_cos[i];
        wy[k] = r * processed_map_.ray_sin[i];
        whit[k] = 1;
    }

    // 빔마다 국소 직선을 TLS로 적합하고 그 법선을 정보행렬에 누적합니다.
    // 창 반폭은 벽 위에서 normal_chord_m 만큼을 덮도록 거리에 따라 넓힙니다.
    // 인접 빔 차분(반폭 1 고정)을 쓰면 r * ray_step 이 격자 해상도 아래로
    // 내려가는 근거리에서 접선이 격자 방향으로 스냅되어, 정작 잡아야 할
    // 복도 퇴화가 실제보다 훨씬 약하게 읽힙니다.
    const double residual_limit_squared =
        parameters_.normal_residual_limit_m * parameters_.normal_residual_limit_m;
    Eigen::Matrix2d information = Eigen::Matrix2d::Zero();
    std::size_t normal_count = 0;
    for (int k = 0; k < win; ++k) {
        if (!whit[k]) {
            continue;
        }
        const int i = (start + k) % n;
        const double r0 = profile[i];
        const int half = std::clamp(
            static_cast<int>(std::ceil(
                parameters_.normal_chord_m / std::max(2.0 * r0 * ray_step, 1.0e-9))),
            1,
            parameters_.normal_max_half_window);

        // 같은 면 위에서만 창을 넓히기 위해 중심에서 양쪽으로 뻗어 나가다가
        // 깊이 불연속(가림 경계)을 만나면 그 방향에서 멈춥니다.
        const double gap_limit = std::max(
            2.0 * processed_map_.resolution, 4.0 * r0 * ray_step);
        double sx = 0.0, sy = 0.0, sxx = 0.0, syy = 0.0, sxy = 0.0;
        int used = 0;
        const auto accumulate = [&](int index) {
            sx += wx[index];
            sy += wy[index];
            sxx += wx[index] * wx[index];
            syy += wy[index] * wy[index];
            sxy += wx[index] * wy[index];
            ++used;
        };
        accumulate(k);
        for (int direction = -1; direction <= 1; direction += 2) {
            double previous = r0;
            for (int step = 1; step <= half; ++step) {
                const int j = k + direction * step;
                if (j < 0 || j >= win || !whit[j]) {
                    break;
                }
                const int jr = (start + j) % n;
                const double rj = profile[jr];
                if (std::abs(rj - previous) > gap_limit) {
                    break;
                }
                previous = rj;
                accumulate(j);
            }
        }
        if (used < 3) {
            continue;
        }

        const double inverse = 1.0 / static_cast<double>(used);
        const double cxx = sxx - sx * sx * inverse;
        const double cyy = syy - sy * sy * inverse;
        const double cxy = sxy - sx * sy * inverse;
        // 2x2 대칭행렬의 닫힌형 고유분해입니다. 작은 쪽 고유벡터가 법선입니다.
        const double trace = cxx + cyy;
        const double spread = std::hypot(cxx - cyy, 2.0 * cxy);
        const double minor = 0.5 * (trace - spread);
        if (trace <= 0.0) {
            continue;
        }
        // 직선에서 벗어난 RMS가 크면 모서리/잡동사니이므로 버립니다.
        if (minor * inverse > residual_limit_squared) {
            continue;
        }
        // 장축(접선) 방향에서 90도 돌려 법선을 얻습니다.
        const double major_angle = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
        const Eigen::Vector2d normal(-std::sin(major_angle), std::cos(major_angle));
        information.noalias() += normal * normal.transpose();
        ++normal_count;
    }

    // 개수로 나누지 않습니다. 단위법선의 외적은 trace가 1이므로 나누면
    // trace(I)가 항상 1이 되어 lambda0 + lambda1 = 1로 고정되고, 지표가
    // 사실상 lambda1 하나로 축퇴되어 "양쪽 다 나쁨"을 표현할 수 없게 됩니다.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(information);
    if (solver.info() != Eigen::Success) {
        return result;
    }

    // SelfAdjointEigenSolver는 오름차순이므로 내림차순으로 뒤집어 담습니다.
    result.eigenvalues(0) = std::max(0.0, solver.eigenvalues()(1));
    result.eigenvalues(1) = std::max(0.0, solver.eigenvalues()(0));
    result.eigenvectors.col(0) = solver.eigenvectors().col(1);
    result.eigenvectors.col(1) = solver.eigenvectors().col(0);
    // 절대량을 [0,1)로 사상합니다. 유효 법선이 하나도 없으면 자연히 0이 되므로
    // "빔이 부족하면 유효하지 않다"는 조기 반환이 필요 없습니다. 예전 구현은
    // 그 경로에서 valid=false를 내보냈고, 소비자는 그것을 등방 + 스캔 100%
    // 신뢰로 해석해서 관측성이 가장 나쁜 곳에서 오히려 스캔을 가장 믿었습니다.
    const double reference = std::max(1.0e-9, parameters_.observability_reference);
    result.confidence(0) = result.eigenvalues(0) / (result.eigenvalues(0) + reference);
    result.confidence(1) = result.eigenvalues(1) / (result.eigenvalues(1) + reference);
    result.information = information;
    result.normal_count = normal_count;
    result.valid = true;
    return result;
}

std::vector<float> Relocalization::synthesizeScans(
    const std::vector<std::array<double, 3>> &poses,
    std::size_t fov_rays,
    double max_range_override,
    int unknown_override) const {
    if (!map_ready_) {
        throw std::logic_error("Relocalization map is not ready for synthesis.");
    }
    if (fov_rays == 0) {
        return {};
    }
    const double max_range = max_range_override > 0.0
        ? max_range_override : parameters_.max_range;
    const bool unknown_is_occupied = unknown_override >= 0
        ? unknown_override != 0 : parameters_.unknown_is_occupied;

    // preprocessMap과 동일한 헬퍼를 써서 raycast 규약이 어긋나지 않게 합니다.
    ranges::OMap range_map = buildRangeMap(
        map_,
        parameters_.occupied_threshold,
        unknown_is_occupied,
        processed_map_.origin_yaw);

    const float max_range_cells = static_cast<float>(
        max_range / processed_map_.resolution);
    ranges::RayMarching raycaster(range_map, max_range_cells);

    const double ray_step = kTwoPi / static_cast<double>(parameters_.ray_count);
    const double fov_start = -0.5 * parameters_.fov_deg * kPi / 180.0;

    std::vector<float> out(poses.size() * fov_rays);
    std::vector<float> angles(fov_rays);
    std::vector<float> ranges(fov_rays);
    for (std::size_t p = 0; p < poses.size(); ++p) {
        const double yaw = poses[p][2];
        // 센서 bearing phi_k를 절대 map 각으로 바꿔 사전계산과 같은 규약으로 쏩니다.
        for (std::size_t k = 0; k < fov_rays; ++k) {
            const double phi = fov_start + static_cast<double>(k) * ray_step;
            angles[k] = static_cast<float>(normalizePositiveAngle(phi + yaw));
        }
        float pose[3] = {
            static_cast<float>(poses[p][0]),
            static_cast<float>(poses[p][1]),
            0.0f};
        raycaster.numpy_calc_range_angles(
            pose, angles.data(), ranges.data(), 1, static_cast<int>(fov_rays));

        float *dst = out.data() + p * fov_rays;
        for (std::size_t k = 0; k < fov_rays; ++k) {
            const float raw = ranges[k];
            dst[k] = std::isfinite(raw) ?
                std::clamp(raw, 0.0f, static_cast<float>(max_range)) :
                static_cast<float>(max_range);
        }
    }
    return out;
}

// 메모리 크기 계산과 RangeLibc int API 변환이 안전한 범위인지 확인합니다.
void Relocalization::validateParameters(const Parameters &parameters) const {
    if (parameters.occupied_threshold < 1 || parameters.occupied_threshold > 100) {
        throw std::invalid_argument("occupied_threshold must be in [1, 100].");
    }
    if (parameters.ray_count < 4) {
        throw std::invalid_argument("ray_count must be at least 4.");
    }
    // run 인덱스를 uint16으로 담으므로 ray_count 상한이 여기서 결정됩니다.
    if (parameters.ray_count > 65535) {
        throw std::invalid_argument("ray_count must not exceed 65535.");
    }
    if (parameters.raycast_batch_size == 0) {
        throw std::invalid_argument("raycast_batch_size must be greater than zero.");
    }
    if (parameters.raycast_batch_size >
        static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument("raycast_batch_size exceeds the RangeLibc API limit.");
    }
    if (parameters.raycast_batch_size >
        std::numeric_limits<std::size_t>::max() / parameters.ray_count) {
        throw std::invalid_argument("The ray-cast batch buffer would overflow.");
    }
    if (parameters.minimum_scan_points == 0) {
        throw std::invalid_argument("minimum_scan_points must be greater than zero.");
    }
    if (!std::isfinite(parameters.max_range) || parameters.max_range <= 0.0) {
        throw std::invalid_argument("max_range must be finite and greater than zero.");
    }
    if (!std::isfinite(parameters.huber_delta) || parameters.huber_delta <= 0.0) {
        throw std::invalid_argument("huber_delta must be finite and greater than zero.");
    }

    if (!std::isfinite(parameters.fov_deg) ||
        parameters.fov_deg <= 0.0 || parameters.fov_deg > 360.0) {
        throw std::invalid_argument("fov_deg must be in (0, 360].");
    }
    if (parameters.robust_axis_iterations < 0) {
        throw std::invalid_argument("robust_axis_iterations must be nonnegative.");
    }
    if (!std::isfinite(parameters.robust_axis_delta) ||
        parameters.robust_axis_delta <= 0.0) {
        throw std::invalid_argument("robust_axis_delta must be finite and greater than zero.");
    }
    if (!std::isfinite(parameters.axis_run_tolerance_deg) ||
        parameters.axis_run_tolerance_deg <= 0.0 ||
        parameters.axis_run_tolerance_deg > 90.0) {
        throw std::invalid_argument("axis_run_tolerance_deg must be in (0, 90].");
    }
    if (!std::isfinite(parameters.axis_match_tolerance_deg) ||
        parameters.axis_match_tolerance_deg <= 0.0 ||
        parameters.axis_match_tolerance_deg > 90.0) {
        throw std::invalid_argument("axis_match_tolerance_deg must be in (0, 90].");
    }
    if (parameters.yaw_refine_bins < 0) {
        throw std::invalid_argument("yaw_refine_bins must be nonnegative.");
    }
    if (parameters.max_heading_hypotheses == 0) {
        throw std::invalid_argument("max_heading_hypotheses must be greater than zero.");
    }
    if (!std::isfinite(parameters.normal_chord_m) || parameters.normal_chord_m <= 0.0) {
        throw std::invalid_argument("normal_chord_m must be finite and greater than zero.");
    }
    if (parameters.normal_max_half_window < 1) {
        throw std::invalid_argument("normal_max_half_window must be at least 1.");
    }
    if (!std::isfinite(parameters.normal_residual_limit_m) ||
        parameters.normal_residual_limit_m <= 0.0) {
        throw std::invalid_argument(
            "normal_residual_limit_m must be finite and greater than zero.");
    }
    if (!std::isfinite(parameters.observability_reference) ||
        parameters.observability_reference <= 0.0) {
        throw std::invalid_argument(
            "observability_reference must be finite and greater than zero.");
    }
    if (!std::isfinite(parameters.coarse_position_step_m) ||
        parameters.coarse_position_step_m <= 0.0) {
        throw std::invalid_argument("coarse_position_step_m must be finite and greater than zero.");
    }
    if (!std::isfinite(parameters.fine_position_radius_m) ||
        parameters.fine_position_radius_m < 0.0) {
        throw std::invalid_argument("fine_position_radius_m must be finite and nonnegative.");
    }
    if (parameters.top_candidates == 0) {
        throw std::invalid_argument("top_candidates must be greater than zero.");
    }
}

void Relocalization::preprocessMap() {
    // 중간에 실패하면 이전 결과를 사용할 수 없도록 먼저 초기화합니다.
    map_ready_ = false;
    processed_map_ = ProcessedMap{};

    // OccupancyGrid 메타데이터와 실제 셀 배열 크기를 먼저 검증합니다.
    const std::size_t expected_size =
        static_cast<std::size_t>(map_.info.width) *
        static_cast<std::size_t>(map_.info.height);
    if (map_.info.width == 0 ||
        map_.info.height == 0 ||
        !std::isfinite(map_.info.resolution) ||
        map_.info.resolution <= 0.0 ||
        map_.data.size() != expected_size) {
        return;
    }

    processed_map_.width = map_.info.width;
    processed_map_.height = map_.info.height;
    processed_map_.resolution = map_.info.resolution;
    processed_map_.origin_x = map_.info.origin.position.x;
    processed_map_.origin_y = map_.info.origin.position.y;
    processed_map_.origin_yaw = tf2::getYaw(map_.info.origin.orientation);

    // 360도를 균일하게 나눈 RangeLibc 입력 각도표를 한 번 생성합니다.
    // 방향 cos/sin은 후보와 무관하므로 여기 한 벌만 둡니다.
    processed_map_.ray_angles.resize(parameters_.ray_count);
    processed_map_.ray_cos.resize(parameters_.ray_count);
    processed_map_.ray_sin.resize(parameters_.ray_count);
    const double ray_step = kTwoPi / static_cast<double>(parameters_.ray_count);
    for (std::size_t ray_index = 0; ray_index < parameters_.ray_count; ++ray_index) {
        const double angle = static_cast<double>(ray_index) * ray_step;
        processed_map_.ray_angles[ray_index] = static_cast<float>(angle);
        processed_map_.ray_cos[ray_index] = static_cast<float>(std::cos(angle));
        processed_map_.ray_sin[ray_index] = static_cast<float>(std::sin(angle));
    }

    // occupied/unknown 셀을 제외하고 가능한 모든 free-cell 중심을 후보로 둡니다.
    // 동시에 cell -> candidate 역인덱스를 만들어 fine 단계 이웃 조회에 씁니다.
    processed_map_.candidates.reserve(expected_size);
    processed_map_.cell_to_candidate.assign(expected_size, -1);
    for (int32_t cell_y = 0; cell_y < static_cast<int32_t>(map_.info.height); ++cell_y) {
        for (int32_t cell_x = 0; cell_x < static_cast<int32_t>(map_.info.width); ++cell_x) {
            if (isCandidateCell(cell_x, cell_y)) {
                const std::size_t flat =
                    static_cast<std::size_t>(cell_y) * map_.info.width +
                    static_cast<std::size_t>(cell_x);
                processed_map_.cell_to_candidate[flat] =
                    static_cast<int32_t>(processed_map_.candidates.size());
                processed_map_.candidates.push_back(
                    makeCandidateLocation(cell_x, cell_y));
            }
        }
    }

    // 후보 수 x ray 수 크기의 연속된 RowMajor range 행렬을 할당합니다.
    processed_map_.ranges.resize(
        static_cast<Eigen::Index>(processed_map_.candidates.size()),
        static_cast<Eigen::Index>(parameters_.ray_count));

    processed_map_.axis_run_offsets.assign(
        processed_map_.candidates.size() + 1, 0u);
    // 후보당 run 수는 보통 수십 개 수준이라 대략 그만큼 미리 잡아 둡니다.
    processed_map_.axis_runs.reserve(processed_map_.candidates.size() * 8);

    // ROS OccupancyGrid를 RangeLibc가 사용하는 이진 점유맵으로 변환합니다.
    ranges::OMap range_map = buildRangeMap(
        map_,
        parameters_.occupied_threshold,
        parameters_.unknown_is_occupied,
        processed_map_.origin_yaw);

    // RangeLibc RayMarching의 내부 max_range 단위는 grid cell입니다.
    const float max_range_cells = static_cast<float>(
        parameters_.max_range / processed_map_.resolution);
    if (!std::isfinite(max_range_cells) || max_range_cells <= 0.0f) {
        throw std::length_error("RangeLibc max range in grid cells is not representable.");
    }
    ranges::RayMarching raycaster(range_map, max_range_cells);

    // 전체 후보를 작은 배치로 나눠 임시 ranges 배열이 과도하게 커지지 않게 합니다.
    std::vector<float> batch_poses;
    std::vector<float> batch_ranges;
    batch_poses.reserve(parameters_.raycast_batch_size * 3);
    batch_ranges.reserve(parameters_.raycast_batch_size * parameters_.ray_count);

    for (std::size_t batch_begin = 0;
        batch_begin < processed_map_.candidates.size();
        batch_begin += parameters_.raycast_batch_size) {
        const std::size_t batch_count = std::min(
            parameters_.raycast_batch_size,
            processed_map_.candidates.size() - batch_begin);

        batch_poses.clear();
        for (std::size_t offset = 0; offset < batch_count; ++offset) {
            const CandidateLocation &candidate =
                processed_map_.candidates[batch_begin + offset];
            batch_poses.push_back(static_cast<float>(candidate.world_x));
            batch_poses.push_back(static_cast<float>(candidate.world_y));
            batch_poses.push_back(0.0f);
        }

        batch_ranges.resize(batch_count * parameters_.ray_count);
        // 각 후보 위치에서 같은 360도 각도표를 사용해 ray를 한꺼번에 계산합니다.
        raycaster.numpy_calc_range_angles(
            batch_poses.data(),
            processed_map_.ray_angles.data(),
            batch_ranges.data(),
            static_cast<int>(batch_count),
            static_cast<int>(parameters_.ray_count));

        for (std::size_t offset = 0; offset < batch_count; ++offset) {
            const std::size_t candidate_index = batch_begin + offset;
            float *profile = processed_map_.ranges.row(
                static_cast<Eigen::Index>(candidate_index)).data();
            const float *raw_ranges =
                batch_ranges.data() + offset * parameters_.ray_count;

            for (std::size_t ray_index = 0;
                ray_index < parameters_.ray_count;
                ++ray_index) {
                const float raw_range = raw_ranges[ray_index];
                profile[ray_index] = std::isfinite(raw_range) ?
                    std::clamp(
                        raw_range,
                        0.0f,
                        static_cast<float>(parameters_.max_range)) :
                    static_cast<float>(parameters_.max_range);
            }

            // 이 후보의 상대주축 run을 만들어 CSR에 이어 붙입니다.
            buildAxisRuns(profile);
            processed_map_.axis_run_offsets[candidate_index + 1] =
                static_cast<uint32_t>(processed_map_.axis_runs.size());
        }
    }

    processed_map_.axis_runs.shrink_to_fit();
    map_ready_ = true;
}

bool Relocalization::isCandidateCell(int32_t cell_x, int32_t cell_y) const {
    if (cell_x < 0 ||
        cell_y < 0 ||
        cell_x >= static_cast<int32_t>(map_.info.width) ||
        cell_y >= static_cast<int32_t>(map_.info.height)) {
        return false;
    }

    const int8_t value = map_.data[
        static_cast<std::size_t>(cell_y) * map_.info.width +
        static_cast<std::size_t>(cell_x)];
    // unknown(-1)과 occupied 셀은 로봇 위치 후보에서 제외합니다.
    return value >= 0 && value < parameters_.occupied_threshold;
}

Relocalization::CandidateLocation Relocalization::makeCandidateLocation(
    int32_t cell_x,
    int32_t cell_y) const {
    const double local_x =
        (static_cast<double>(cell_x) + 0.5) * processed_map_.resolution;
    const double local_y =
        (static_cast<double>(cell_y) + 0.5) * processed_map_.resolution;
    const double cos_yaw = std::cos(processed_map_.origin_yaw);
    const double sin_yaw = std::sin(processed_map_.origin_yaw);

    // OccupancyGrid origin은 회전할 수 있으므로 단순 x/y 오프셋만 더하지 않습니다.
    CandidateLocation candidate;
    candidate.cell_x = cell_x;
    candidate.cell_y = cell_y;
    candidate.world_x = processed_map_.origin_x + cos_yaw * local_x - sin_yaw * local_y;
    candidate.world_y = processed_map_.origin_y + sin_yaw * local_x + cos_yaw * local_y;
    return candidate;
}

std::size_t Relocalization::buildAxisRuns(const float *profile) {
    const std::size_t ray_count = parameters_.ray_count;
    const int n = static_cast<int>(ray_count);
    const int win = static_cast<int>(fovWindow());
    const int hlf = win / 2;
    const double max_hit_range = parameters_.max_range -
        std::max(processed_map_.resolution * 0.5, 1.0e-3);

    // 슬라이딩 창에서 반복 사용할 끝점과 유효 표시입니다. 후보마다 크기가 같아
    // 재할당이 없도록 함수 로컬 스크래치를 재사용합니다.
    static thread_local std::vector<double> px, py;
    static thread_local std::vector<uint8_t> valid;
    static thread_local std::vector<double> dir_x, dir_y;  // 배각 단위벡터
    static thread_local std::vector<uint8_t> axis_valid;
    px.resize(ray_count);
    py.resize(ray_count);
    valid.resize(ray_count);
    dir_x.resize(ray_count);
    dir_y.resize(ray_count);
    axis_valid.resize(ray_count);

    for (std::size_t i = 0; i < ray_count; ++i) {
        const double range = profile[i];
        const bool ok = std::isfinite(range) && range > 0.0 && range < max_hit_range;
        valid[i] = ok ? 1 : 0;
        px[i] = ok ? range * processed_map_.ray_cos[i] : 0.0;
        py[i] = ok ? range * processed_map_.ray_sin[i] : 0.0;
    }

    // 중심 c=0의 창 [c-hlf, c-hlf+win)에 대한 러닝 합으로 시작합니다.
    double sxx = 0.0, syy = 0.0, sxy = 0.0, sx = 0.0, sy = 0.0;
    int count = 0;
    const auto add_index = [&](int idx) {
        if (valid[idx]) {
            sxx += px[idx] * px[idx];
            syy += py[idx] * py[idx];
            sxy += px[idx] * py[idx];
            sx += px[idx];
            sy += py[idx];
            ++count;
        }
    };
    const auto remove_index = [&](int idx) {
        if (valid[idx]) {
            sxx -= px[idx] * px[idx];
            syy -= py[idx] * py[idx];
            sxy -= px[idx] * py[idx];
            sx -= px[idx];
            sy -= py[idx];
            --count;
        }
    };

    const int start0 = ((0 - hlf) % n + n) % n;
    for (int k = 0; k < win; ++k) {
        add_index((start0 + k) % n);
    }

    // A(psi)를 각도로 바꾸지 않고 배각(2*theta) 단위벡터로 들고 갑니다.
    // 두 축의 각오차가 tol 이내인지는 배각 벡터의 내적 >= cos(2*tol)와 같으므로
    // atan2는 run 하나당 한 번만 부르면 됩니다.
    for (int c = 0; c < n; ++c) {
        axis_valid[c] = 0;
        if (count >= 3) {
            const double inv = 1.0 / static_cast<double>(count);
            const double cxx = sxx - sx * sx * inv;
            const double cyy = syy - sy * sy * inv;
            const double cxy = sxy - sx * sy * inv;
            // (cxx-cyy, 2cxy)의 크기는 산포행렬 고유값 차(lambda1-lambda2)이고
            // cxx+cyy는 그 합이므로, 비율이 곧 이방성입니다.
            const double vx = cxx - cyy;
            const double vy = 2.0 * cxy;
            const double magnitude = std::hypot(vx, vy);
            const double trace = cxx + cyy;
            if (magnitude > 1.0e-3 * std::max(trace, 1.0e-12)) {
                dir_x[c] = vx / magnitude;
                dir_y[c] = vy / magnitude;
                axis_valid[c] = 1;
            }
        }
        const int leave = ((c - hlf) % n + n) % n;
        const int enter = ((c - hlf + win) % n + n) % n;
        remove_index(leave);
        add_index(enter);
    }

    // run 시작점을 정합니다. 원형 배열이므로 "직전 칸과 이어지지 않는" 칸을
    // 하나 찾아 거기서부터 한 바퀴 돌아야 wrap 구간이 둘로 쪼개지지 않습니다.
    const double cos_run_tol = std::cos(
        2.0 * parameters_.axis_run_tolerance_deg * kPi / 180.0);
    int origin = -1;
    int valid_total = 0;
    for (int c = 0; c < n; ++c) {
        if (!axis_valid[c]) {
            continue;
        }
        ++valid_total;
        if (origin >= 0) {
            continue;
        }
        const int prev = (c - 1 + n) % n;
        if (!axis_valid[prev] ||
            dir_x[c] * dir_x[prev] + dir_y[c] * dir_y[prev] < cos_run_tol) {
            origin = c;
        }
    }
    if (valid_total == 0) {
        return 0;
    }
    if (origin < 0) {
        // 한 바퀴가 모두 이어져 있으면 어디서 시작해도 같습니다.
        origin = 0;
    }

    const std::size_t before = processed_map_.axis_runs.size();
    int k = 0;
    while (k < n) {
        const int c = (origin + k) % n;
        if (!axis_valid[c]) {
            ++k;
            continue;
        }
        // 앵커 대비 허용오차 안에 있는 동안만 같은 run으로 이어 붙입니다.
        const double anchor_x = dir_x[c];
        const double anchor_y = dir_y[c];
        double sum_x = 0.0;
        double sum_y = 0.0;
        int length = 0;
        while (k + length < n) {
            const int j = (origin + k + length) % n;
            if (!axis_valid[j] ||
                dir_x[j] * anchor_x + dir_y[j] * anchor_y < cos_run_tol) {
                break;
            }
            sum_x += dir_x[j];
            sum_y += dir_y[j];
            ++length;
        }

        AxisRun run;
        run.start = static_cast<uint16_t>(c);
        run.length = static_cast<uint16_t>(length);
        // 배각 공간의 평균 방향이 이 구간의 대표축입니다.
        run.axis = static_cast<float>(
            canonicalizeAxis(0.5 * std::atan2(sum_y, sum_x)));
        processed_map_.axis_runs.push_back(run);
        k += length;
    }
    return processed_map_.axis_runs.size() - before;
}

double Relocalization::principalAxisAngle(
    const std::vector<Eigen::Vector2d> &points) const {
    if (points.size() < 3) {
        return 0.0;
    }

    // 가중치가 주어진 점군의 주성분 축 각도와 가중 평균을 계산합니다.
    const auto weighted_axis = [](const std::vector<Eigen::Vector2d> &pts,
                                  const std::vector<double> &weights,
                                  Eigen::Vector2d &mean_out) -> double {
        double weight_sum = 0.0;
        Eigen::Vector2d mean = Eigen::Vector2d::Zero();
        for (std::size_t i = 0; i < pts.size(); ++i) {
            mean += weights[i] * pts[i];
            weight_sum += weights[i];
        }
        if (weight_sum <= 0.0) {
            mean_out = Eigen::Vector2d::Zero();
            return 0.0;
        }
        mean /= weight_sum;
        mean_out = mean;

        double cxx = 0.0, cxy = 0.0, cyy = 0.0;
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const Eigen::Vector2d centered = pts[i] - mean;
            cxx += weights[i] * centered.x() * centered.x();
            cxy += weights[i] * centered.x() * centered.y();
            cyy += weights[i] * centered.y() * centered.y();
        }
        // 2x2 공분산 주축의 닫힌형 각도(undirected)입니다.
        return canonicalizeAxis(0.5 * std::atan2(2.0 * cxy, cxx - cyy));
    };

    std::vector<double> weights(points.size(), 1.0);
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    // 비가중 PCA 주축(고정 경로). 오프라인 A(psi)는 러닝합 기반이라 항상
    // 비가중이고, 라이브 축도 같은 규약으로 맞춥니다.
    return weighted_axis(points, weights, mean);
}

Relocalization::ScanView Relocalization::convertScan(
    const sensor_msgs::msg::LaserScan &scan) const {
    ScanView view;

    // 사전계산 ray 간격보다 촘촘한 실제 스캔은 정수 stride로 다운샘플링합니다.
    const double ray_step = kTwoPi / static_cast<double>(parameters_.ray_count);
    const double input_angle_step = std::abs(static_cast<double>(scan.angle_increment));
    const std::size_t input_stride =
        std::isfinite(input_angle_step) && input_angle_step > 0.0 ?
        std::max<std::size_t>(
            1,
            static_cast<std::size_t>(std::llround(ray_step / input_angle_step))) :
        1;
    const std::size_t reserve_count =
        (scan.ranges.size() + input_stride - 1) / input_stride;
    view.base_index.reserve(reserve_count);
    view.range.reserve(reserve_count);

    // 센서 범위와 사전계산 최대거리 중 더 작은 값을 비교 범위로 사용합니다.
    const double scan_min = std::max(0.0, static_cast<double>(scan.range_min));
    const double scan_max =
        std::isfinite(scan.range_max) && scan.range_max > 0.0f ?
        static_cast<double>(scan.range_max) : parameters_.max_range;
    const double usable_max = std::min(scan_max, parameters_.max_range);

    std::vector<Eigen::Vector2d> points;
    points.reserve(reserve_count);

    for (std::size_t index = 0;
        index < scan.ranges.size();
        index += input_stride) {
        const double raw_range = scan.ranges[index];
        if (!std::isfinite(raw_range) ||
            raw_range < scan_min ||
            raw_range > scan_max ||
            raw_range <= 0.0) {
            continue;
        }

        const double angle =
            static_cast<double>(scan.angle_min) +
            static_cast<double>(index) * scan.angle_increment;
        const double range = std::min(raw_range, usable_max);

        // yaw를 ray 격자로 양자화하면 map ray 인덱스는 이 값에 yaw_index를
        // 더하기만 하면 됩니다. 정합 루프에서 삼각함수가 사라지는 지점입니다.
        const int32_t base = static_cast<int32_t>(
            std::llround(normalizePositiveAngle(angle) / ray_step)) %
            static_cast<int32_t>(parameters_.ray_count);
        view.base_index.push_back(base);
        view.range.push_back(static_cast<float>(range));
        points.emplace_back(range * std::cos(angle), range * std::sin(angle));
    }

    view.axis = principalAxisAngle(points);
    return view;
}

bool Relocalization::collectHeadingHypotheses(
    std::size_t candidate_index,
    double scan_axis,
    std::vector<int32_t> &yaw_indices) const {
    yaw_indices.clear();

    const uint32_t run_begin = processed_map_.axis_run_offsets[candidate_index];
    const uint32_t run_end = processed_map_.axis_run_offsets[candidate_index + 1];
    if (run_begin == run_end) {
        return false;
    }

    const int n = static_cast<int>(parameters_.ray_count);
    const double ray_step = kTwoPi / static_cast<double>(parameters_.ray_count);
    const int tolerance_bins = static_cast<int>(std::llround(
        parameters_.axis_match_tolerance_deg * kPi / 180.0 / ray_step));

    std::size_t solved = 0;
    for (uint32_t r = run_begin; r < run_end; ++r) {
        const AxisRun &run = processed_map_.axis_runs[r];
        // run 안에서는 A가 상수이므로 b(psi) = A_r - psi 이고,
        // b(psi) = scan_axis 를 만족하는 psi는 mod pi로 즉시 나옵니다.
        double base = std::fmod(
            static_cast<double>(run.axis) - scan_axis, kPi);
        if (base < 0.0) {
            base += kPi;
        }

        for (int branch = 0; branch < 2; ++branch) {
            // undirected 축은 pi 모호성이 있으므로 psi와 psi+pi를 모두 봅니다.
            const double psi = base + static_cast<double>(branch) * kPi;
            const int index =
                static_cast<int>(std::llround(psi / ray_step)) % n;
            // run 구간 [start, start+length)를 양쪽으로 tolerance_bins만큼
            // 넓힌 범위 안에 해가 들어오는지 확인합니다.
            const int relative = ((index - static_cast<int>(run.start)) % n + n) % n;
            const int span = static_cast<int>(run.length) + tolerance_bins;
            if (span < n && relative >= span && relative < n - tolerance_bins) {
                continue;
            }
            yaw_indices.push_back(index);
            ++solved;
        }
        if (solved >= parameters_.max_heading_hypotheses) {
            break;
        }
    }

    if (yaw_indices.empty()) {
        return false;
    }

    // 스캔 축 추정 오차를 정합으로 흡수하도록 해 주변 몇 칸을 함께 평가합니다.
    const int refine = parameters_.yaw_refine_bins;
    if (refine > 0) {
        const std::size_t solved_count = yaw_indices.size();
        for (std::size_t i = 0; i < solved_count; ++i) {
            const int center = yaw_indices[i];
            for (int d = -refine; d <= refine; ++d) {
                if (d == 0) {
                    continue;
                }
                yaw_indices.push_back(((center + d) % n + n) % n);
            }
        }
    }

    std::sort(yaw_indices.begin(), yaw_indices.end());
    yaw_indices.erase(
        std::unique(yaw_indices.begin(), yaw_indices.end()),
        yaw_indices.end());
    return true;
}

Relocalization::PoseCandidate Relocalization::evaluateCandidate(
    std::size_t candidate_index,
    const ScanView &scan,
    std::vector<int32_t> &yaw_scratch,
    std::vector<PoseCandidate> *collector) {
    const CandidateLocation &candidate = processed_map_.candidates[candidate_index];

    PoseCandidate best;
    best.x = candidate.world_x;
    best.y = candidate.world_y;
    best.yaw = 0.0;
    best.score = -std::numeric_limits<double>::infinity();

    // 상대주축이 스캔과 양립하지 않으면 range 정합 자체를 건너뜁니다.
    // 전체 지연의 대부분이 여기서 잘려 나갑니다.
    if (!collectHeadingHypotheses(candidate_index, scan.axis, yaw_scratch)) {
        return best;
    }

    const float *profile = processed_map_.ranges.row(
        static_cast<Eigen::Index>(candidate_index)).data();
    const double ray_step = kTwoPi / static_cast<double>(parameters_.ray_count);

    int32_t best_index = 0;
    for (const int32_t yaw_index : yaw_scratch) {
        const double score = scoreCandidate(profile, scan, yaw_index);
        if (score > best.score) {
            best.score = score;
            best_index = yaw_index;
        }
        if (collector != nullptr && std::isfinite(score)) {
            PoseCandidate item;
            item.x = best.x;
            item.y = best.y;
            item.yaw = normalizeAngle(static_cast<double>(yaw_index) * ray_step);
            item.score = score;
            collector->push_back(item);
        }
    }
    best.yaw = normalizeAngle(static_cast<double>(best_index) * ray_step);

    ++diagnostics_.candidates_scored;
    diagnostics_.hypotheses_scored += yaw_scratch.size();
    return best;
}

Relocalization::SearchResult Relocalization::searchGlobalPose(
    const ScanView &scan,
    std::vector<PoseCandidate> *collector) {
    SearchResult result;
    result.best.score = -std::numeric_limits<double>::infinity();

    diagnostics_ = Diagnostics{};
    diagnostics_.scan_axis = scan.axis;

    if (processed_map_.candidates.empty()) {
        return result;
    }

    std::vector<int32_t> yaw_scratch;
    yaw_scratch.reserve(64);

    const auto consider = [&](std::size_t idx) -> PoseCandidate {
        ++diagnostics_.candidates_visited;
        const PoseCandidate pc = evaluateCandidate(idx, scan, yaw_scratch, collector);
        if (pc.score > result.best.score) {
            result.valid = true;
            result.best = pc;
        }
        return pc;
    };

    // ----- coarse 단계: 성긴 격자 위 후보만 평가하고 상위 M개를 남깁니다. -----
    const int stride_cells = std::max(1, static_cast<int>(std::llround(
        parameters_.coarse_position_step_m / processed_map_.resolution)));
    std::vector<std::pair<double, std::size_t>> scored;
    scored.reserve(
        processed_map_.candidates.size() / static_cast<std::size_t>(stride_cells) + 1);
    for (std::size_t idx = 0; idx < processed_map_.candidates.size(); ++idx) {
        const CandidateLocation &c = processed_map_.candidates[idx];
        if (c.cell_x % stride_cells != 0 || c.cell_y % stride_cells != 0) {
            continue;
        }
        const PoseCandidate pc = consider(idx);
        if (std::isfinite(pc.score)) {
            scored.emplace_back(pc.score, idx);
        }
    }

    // ----- fine 단계: 상위 후보 주변 반경 안의 조밀 후보를 추가 평가합니다. -----
    const std::size_t keep = std::min(parameters_.top_candidates, scored.size());
    if (keep > 0) {
        std::partial_sort(
            scored.begin(),
            scored.begin() + static_cast<std::ptrdiff_t>(keep),
            scored.end(),
            [](const std::pair<double, std::size_t> &a,
               const std::pair<double, std::size_t> &b) {
                return a.first > b.first;
            });
    }

    const int radius_cells = static_cast<int>(std::llround(
        parameters_.fine_position_radius_m / processed_map_.resolution));
    if (radius_cells > 0) {
        const int width = static_cast<int>(processed_map_.width);
        const int height = static_cast<int>(processed_map_.height);
        std::unordered_set<int32_t> visited;
        for (std::size_t rank = 0; rank < keep; ++rank) {
            const CandidateLocation &center =
                processed_map_.candidates[scored[rank].second];
            for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                const int ny = center.cell_y + dy;
                if (ny < 0 || ny >= height) {
                    continue;
                }
                for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                    const int nx = center.cell_x + dx;
                    if (nx < 0 || nx >= width) {
                        continue;
                    }
                    // coarse 격자 위 셀은 이미 평가했으므로 건너뜁니다.
                    if (nx % stride_cells == 0 && ny % stride_cells == 0) {
                        continue;
                    }
                    const int32_t ci = processed_map_.cell_to_candidate[
                        static_cast<std::size_t>(ny) * width + static_cast<std::size_t>(nx)];
                    if (ci < 0) {
                        continue;
                    }
                    if (!visited.insert(ci).second) {
                        continue;
                    }
                    consider(static_cast<std::size_t>(ci));
                }
            }
        }
    }

    return result;
}

double Relocalization::scoreCandidate(
    const float *candidate_ranges,
    const ScanView &scan,
    int32_t yaw_index) const {
    const int32_t n = static_cast<int32_t>(parameters_.ray_count);
    const std::size_t point_count = scan.size();
    if (point_count == 0) {
        return -std::numeric_limits<double>::infinity();
    }

    const int32_t *base = scan.base_index.data();
    const float *observed = scan.range.data();
    double total_error = 0.0;

    // base_index와 yaw_index 모두 [0, n)이므로 합은 2n 미만이고 조건부 뺄셈
    // 한 번이면 modulo가 끝납니다. 삼각함수도 분기도 없는 gather + reduction이라
    // 그대로 GPU kernel로 옮길 수 있습니다.
    // MaskedMSE 채점(고정 경로).
    for (std::size_t i = 0; i < point_count; ++i) {
        int32_t index = base[i] + yaw_index;
        if (index >= n) {
            index -= n;
        }
        const double residual =
            static_cast<double>(observed[i]) -
            static_cast<double>(candidate_ranges[index]);
        total_error += residual * residual;
    }

    // 외부 탐색은 큰 점수를 선택하므로 평균 오차에 음수를 붙입니다.
    return -total_error / static_cast<double>(point_count);
}

// 내부 x/y/yaw 후보를 ROS Pose 메시지와 단위 quaternion으로 변환합니다.
geometry_msgs::msg::Pose Relocalization::buildPose(
    const SearchResult &result) const {
    geometry_msgs::msg::Pose pose;
    pose.position.x = result.best.x;
    pose.position.y = result.best.y;
    pose.orientation.z = std::sin(result.best.yaw * 0.5);
    pose.orientation.w = std::cos(result.best.yaw * 0.5);
    return pose;
}
