#include "observability_map.h"

#include <utility>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <thread>

#include <Eigen/Eigenvalues>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double normalizePositiveAngle(double angle) {
    angle = std::fmod(angle, kTwoPi);
    if (angle < 0.0) {
        angle += kTwoPi;
    }
    return angle;
}

}  // namespace

void ObservabilityMap::setParameters(const Parameters &parameters) {
    parameters_ = parameters;
    parameters_.grid_step_m = std::max(1.0e-3, parameters_.grid_step_m);
    parameters_.heading_bins = std::max(1, parameters_.heading_bins);
    parameters_.clearance_m = std::max(0.0, parameters_.clearance_m);
    parameters_.slip_tolerance_m = std::max(1.0e-4, parameters_.slip_tolerance_m);
    parameters_.max_sigma_m = std::max(parameters_.slip_tolerance_m,
                                       parameters_.max_sigma_m);
    parameters_.normal_chord_m = std::max(1.0e-6, parameters_.normal_chord_m);
    parameters_.normal_max_half_window =
        std::max(1, parameters_.normal_max_half_window);
    parameters_.normal_residual_limit_m =
        std::max(1.0e-6, parameters_.normal_residual_limit_m);
    parameters_.line_split_m = std::max(1.0e-4, parameters_.line_split_m);
    parameters_.line_min_beams = std::max(3, parameters_.line_min_beams);
    parameters_.line_min_length_m = std::max(0.0, parameters_.line_min_length_m);
    parameters_.reference_range_m = std::max(0.1, parameters_.reference_range_m);
    parameters_.perpendicular_noise_m =
        std::max(1.0e-4, parameters_.perpendicular_noise_m);
    parameters_.direction_cluster_deg =
        std::clamp(parameters_.direction_cluster_deg, 0.0, 45.0);
    parameters_.max_range_m = std::max(0.5, parameters_.max_range_m);
}

void ObservabilityMap::clear() {
    ready_ = false;
    entries_.clear();
    grid_width_ = 0;
    grid_height_ = 0;
    diagnostics_ = Diagnostics{};
}

const ObservabilityMap::Entry *ObservabilityMap::entry(
    int ix, int iy, int heading_bin) const {
    if (ix < 0 || iy < 0 || ix >= grid_width_ || iy >= grid_height_) {
        return nullptr;
    }
    const std::size_t index =
        (static_cast<std::size_t>(iy) * static_cast<std::size_t>(grid_width_) +
         static_cast<std::size_t>(ix)) *
            static_cast<std::size_t>(parameters_.heading_bins) +
        static_cast<std::size_t>(heading_bin);
    return &entries_[index];
}

std::vector<ObservabilityMap::LineSegment> ObservabilityMap::extractSegments(
    const float *ranges,
    std::size_t ray_count,
    double start_angle_rad,
    double ray_step_rad,
    double max_hit_range,
    double resolution) const {
    std::vector<LineSegment> segments;
    if (ray_count == 0) {
        return segments;
    }

    static thread_local std::vector<double> px, py;
    static thread_local std::vector<uint8_t> hit;
    px.assign(ray_count, 0.0);
    py.assign(ray_count, 0.0);
    hit.assign(ray_count, 0);
    for (std::size_t k = 0; k < ray_count; ++k) {
        const double range = ranges[k];
        if (!std::isfinite(range) || range <= 0.0 || range >= max_hit_range) {
            continue;
        }
        const double bearing = start_angle_rad + static_cast<double>(k) * ray_step_rad;
        px[k] = range * std::cos(bearing);
        py[k] = range * std::sin(bearing);
        hit[k] = 1;
    }

    // [begin, end] 구간을 IEPF로 재귀 분할해 채택된 선분을 push합니다.
    const auto flush_segment = [&](std::size_t begin, std::size_t end) {
        static thread_local std::vector<std::pair<std::size_t, std::size_t>> stack;
        stack.clear();
        stack.emplace_back(begin, end);
        while (!stack.empty()) {
            const auto [b, e] = stack.back();
            stack.pop_back();
            const std::size_t count = e - b + 1;
            if (count < static_cast<std::size_t>(parameters_.line_min_beams)) {
                continue;
            }
            const double ax = px[b], ay = py[b];
            const double bx2 = px[e], by2 = py[e];
            double ex = bx2 - ax, ey = by2 - ay;
            const double chord = std::hypot(ex, ey);
            double max_distance = 0.0;
            std::size_t split = b;
            if (chord > 1.0e-9) {
                ex /= chord; ey /= chord;
                for (std::size_t j = b + 1; j < e; ++j) {
                    const double d = std::abs(
                        (px[j] - ax) * ey - (py[j] - ay) * ex);
                    if (d > max_distance) { max_distance = d; split = j; }
                }
            }
            if (max_distance > parameters_.line_split_m && split > b && split < e) {
                stack.emplace_back(b, split);
                stack.emplace_back(split, e);
                continue;
            }
            if (chord < parameters_.line_min_length_m) {
                continue;
            }
            double sx = 0.0, sy = 0.0, sxx = 0.0, syy = 0.0, sxy = 0.0;
            for (std::size_t j = b; j <= e; ++j) {
                sx += px[j]; sy += py[j];
                sxx += px[j] * px[j]; syy += py[j] * py[j]; sxy += px[j] * py[j];
            }
            const double inverse = 1.0 / static_cast<double>(count);
            const double cxx = sxx - sx * sx * inverse;
            const double cyy = syy - sy * sy * inverse;
            const double cxy = sxy - sx * sy * inverse;
            if (cxx + cyy <= 0.0) {
                continue;
            }
            const double major_angle = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
            LineSegment segment;
            segment.nx = -std::sin(major_angle);
            segment.ny = std::cos(major_angle);
            segment.normal_angle_deg =
                std::atan2(segment.ny, segment.nx) * 180.0 / kPi;
            segment.length_m = chord;
            segment.beams = static_cast<int>(count);
            segment.mean_range_m =
                std::hypot(sx * inverse, sy * inverse);
            segments.push_back(segment);
        }
    };

    std::size_t run_begin = 0;
    bool in_run = false;
    double previous_range = 0.0;
    for (std::size_t k = 0; k <= ray_count; ++k) {
        const bool valid_beam = k < ray_count && hit[k];
        const double gap_limit = valid_beam
            ? std::max(2.0 * resolution, 4.0 * ranges[k] * ray_step_rad)
            : 0.0;
        const bool contiguous = valid_beam && in_run &&
            std::abs(ranges[k] - previous_range) <= gap_limit;
        if (valid_beam && !in_run) {
            run_begin = k;
            in_run = true;
        } else if (in_run && !contiguous) {
            if (k - 1 > run_begin) {
                flush_segment(run_begin, k - 1);
            }
            if (valid_beam) {
                run_begin = k;
            } else {
                in_run = false;
            }
        }
        if (valid_beam) {
            previous_range = ranges[k];
        }
    }
    return segments;
}

// M = (1/N_total) sum_hit n n^T 누적부입니다.
ObservabilityMap::Entry ObservabilityMap::computeEntry(
    const float *ranges,
    std::size_t ray_count,
    double start_angle_rad,
    double ray_step_rad,
    double max_hit_range,
    double resolution) const {
    Entry result;
    if (ray_count == 0) {
        return result;
    }

    // 끝점을 pose 기준 상대 좌표로 펼칩니다. 여기가 유일한 입력입니다.
    static thread_local std::vector<double> px, py;
    static thread_local std::vector<uint8_t> hit;
    px.assign(ray_count, 0.0);
    py.assign(ray_count, 0.0);
    hit.assign(ray_count, 0);
    for (std::size_t k = 0; k < ray_count; ++k) {
        const double range = ranges[k];
        if (!std::isfinite(range) || range <= 0.0 || range >= max_hit_range) {
            continue;
        }
        // 법선은 세계 프레임이어야 합니다(고유벡터를 세계 프레임 방향으로
        // 소비하므로). start_angle = heading + fov_start.
        const double bearing = start_angle_rad + static_cast<double>(k) * ray_step_rad;
        px[k] = range * std::cos(bearing);
        py[k] = range * std::sin(bearing);
        hit[k] = 1;
    }

    double mxx = 0.0;
    double mxy = 0.0;
    double myy = 0.0;
    std::size_t normal_count = 0;
    const double residual_limit_squared =
        parameters_.normal_residual_limit_m * parameters_.normal_residual_limit_m;

    if (parameters_.normal_method == NormalMethod::LineDetection) {
        const auto segments = extractSegments(
            ranges, ray_count, start_angle_rad, ray_step_rad,
            max_hit_range, resolution);

        // 선분별 유효 가중치(등가 길이 x 법선 신뢰도)를 먼저 계산합니다.
        //  - 등가 길이: 같은 벽은 거리와 무관하게 같은 가중(시점 불변).
        //  - 신뢰도 감쇠만, 등방 항 없음: "모름"은 구속이 아닙니다.
        struct Weighted {
            double weight;
            double angle;   // 법선 각도, mod pi 로 접은 배각 공간에서 군집화
            int beams;
        };
        static thread_local std::vector<Weighted> weighted;
        weighted.clear();
        for (const auto &segment : segments) {
            const double weight = segment.length_m /
                (parameters_.reference_range_m * ray_step_rad);
            const double sigma_alpha =
                parameters_.perpendicular_noise_m *
                std::sqrt(12.0 / std::max(1, segment.beams)) /
                std::max(1.0e-6, segment.length_m);
            const double confidence =
                std::exp(-2.0 * sigma_alpha * sigma_alpha);
            weighted.push_back({
                weight * confidence,
                std::atan2(segment.ny, segment.nx),
                segment.beams});
        }

        // 방향 군집화: 법선이 cluster_tol 이내인 선분은 같은 방향의 반복
        // 측정입니다. 벽 요철 지터(±1~3도)가 만드는 sin^2(지터) 누설이
        // 복도(벽 2개)를 벽 1개보다 덜 퇴화해 보이게 하므로, 군집의 가중
        // 평균 법선 하나로 합쳐 스프레드를 없앱니다. 배각(2*angle) 공간
        // 벡터합이 undirected 축의 가중 원형 평균입니다.
        const double cluster_tol =
            parameters_.direction_cluster_deg * kPi / 180.0;
        static thread_local std::vector<double> cluster_x, cluster_y, cluster_w;
        static thread_local std::vector<int> cluster_beams;
        cluster_x.clear(); cluster_y.clear(); cluster_w.clear(); cluster_beams.clear();
        const double cos_tol = std::cos(2.0 * cluster_tol);
        for (const auto &item : weighted) {
            const double dx = std::cos(2.0 * item.angle);
            const double dy = std::sin(2.0 * item.angle);
            int found = -1;
            if (cluster_tol > 0.0) {
                for (std::size_t c = 0; c < cluster_x.size(); ++c) {
                    const double norm = std::hypot(cluster_x[c], cluster_y[c]);
                    if (norm < 1.0e-12) continue;
                    if ((cluster_x[c] * dx + cluster_y[c] * dy) / norm >= cos_tol) {
                        found = static_cast<int>(c);
                        break;
                    }
                }
            }
            if (found < 0) {
                cluster_x.push_back(item.weight * dx);
                cluster_y.push_back(item.weight * dy);
                cluster_w.push_back(item.weight);
                cluster_beams.push_back(item.beams);
            } else {
                cluster_x[found] += item.weight * dx;
                cluster_y[found] += item.weight * dy;
                cluster_w[found] += item.weight;
                cluster_beams[found] += item.beams;
            }
        }
        for (std::size_t c = 0; c < cluster_w.size(); ++c) {
            const double angle = 0.5 * std::atan2(cluster_y[c], cluster_x[c]);
            const double nx2 = std::cos(angle);
            const double ny2 = std::sin(angle);
            mxx += cluster_w[c] * nx2 * nx2;
            mxy += cluster_w[c] * nx2 * ny2;
            myy += cluster_w[c] * ny2 * ny2;
            normal_count += static_cast<std::size_t>(cluster_beams[c]);
        }
        if (normal_count < 3) {
            return result;
        }
        const double inverse_total = 1.0 / static_cast<double>(ray_count);
        result.mxx = static_cast<float>(mxx * inverse_total);
        result.mxy = static_cast<float>(mxy * inverse_total);
        result.myy = static_cast<float>(myy * inverse_total);
        result.normal_count =
            static_cast<uint16_t>(std::min<std::size_t>(normal_count, 65535));
        result.valid = 1;
        return result;
    }

    for (std::size_t k = 0; k < ray_count; ++k) {
        if (!hit[k]) {
            continue;
        }
        const double range = ranges[k];
        double nx = 0.0;
        double ny = 0.0;

        if (parameters_.normal_method == NormalMethod::AdjacentDifference) {
            const std::size_t k2 = k + 1 < ray_count ? k + 1 : k - 1;
            if (k2 >= ray_count || !hit[k2]) {
                continue;
            }
            const double dx = px[k2] - px[k];
            const double dy = py[k2] - py[k];
            const double length = std::hypot(dx, dy);
            const double gap_limit =
                std::max(2.0 * resolution, 4.0 * range * ray_step_rad);
            if (length < 1.0e-9 || length > gap_limit) {
                continue;
            }
            nx = -dy / length;
            ny = dx / length;
        } else {
            // 호길이 고정 창 TLS. 인접 차분은 격자 양자화로 접선이 격자
            // 방향에 스냅되어(직선 복도 실측 수백 배 오염) 쓰지 않습니다.
            const int half = std::clamp(
                static_cast<int>(std::ceil(
                    parameters_.normal_chord_m /
                    std::max(2.0 * range * ray_step_rad, 1.0e-9))),
                1,
                parameters_.normal_max_half_window);
            const double gap_limit =
                std::max(2.0 * resolution, 4.0 * range * ray_step_rad);
            double sx = px[k], sy = py[k];
            double sxx = px[k] * px[k], syy = py[k] * py[k], sxy = px[k] * py[k];
            int used = 1;
            for (int direction = -1; direction <= 1; direction += 2) {
                double previous = range;
                for (int step = 1; step <= half; ++step) {
                    const long long j = static_cast<long long>(k) + direction * step;
                    if (j < 0 || j >= static_cast<long long>(ray_count) || !hit[j]) {
                        break;
                    }
                    const double rj = ranges[j];
                    if (std::abs(rj - previous) > gap_limit) {
                        break;
                    }
                    previous = rj;
                    sx += px[j];
                    sy += py[j];
                    sxx += px[j] * px[j];
                    syy += py[j] * py[j];
                    sxy += px[j] * py[j];
                    ++used;
                }
            }
            if (used < 3) {
                continue;
            }
            const double inverse = 1.0 / static_cast<double>(used);
            const double cxx = sxx - sx * sx * inverse;
            const double cyy = syy - sy * sy * inverse;
            const double cxy = sxy - sx * sy * inverse;
            const double trace = cxx + cyy;
            const double spread = std::hypot(cxx - cyy, 2.0 * cxy);
            const double minor = 0.5 * (trace - spread);
            if (trace <= 0.0 || minor * inverse > residual_limit_squared) {
                continue;
            }
            const double major_angle = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
            nx = -std::sin(major_angle);
            ny = std::cos(major_angle);
        }

        mxx += nx * nx;
        mxy += nx * ny;
        myy += ny * ny;
        ++normal_count;
    }

    if (normal_count < 3) {
        return result;
    }

    // 전체 빔 수로 나눕니다(적중률이 trace에 실리도록). 법선 게이트에서
    // 떨어진 hit 빔도 분모에 남는 것이 맞습니다 - 그 빔이 실제로는 정보를
    // 주지 못했기 때문입니다.
    const double inverse_total = 1.0 / static_cast<double>(ray_count);
    result.mxx = static_cast<float>(mxx * inverse_total);
    result.mxy = static_cast<float>(mxy * inverse_total);
    result.myy = static_cast<float>(myy * inverse_total);
    result.normal_count =
        static_cast<uint16_t>(std::min<std::size_t>(normal_count, 65535));
    result.valid = 1;
    return result;
}

void ObservabilityMap::build(
    const nav_msgs::msg::OccupancyGrid &map,
    const Relocalization &relocalization,
    double fov_deg,
    std::size_t fov_rays) {
    clear();
    if (map.info.width == 0 || map.info.height == 0 ||
        !std::isfinite(map.info.resolution) || map.info.resolution <= 0.0 ||
        fov_rays == 0) {
        return;
    }

    const auto start_time = std::chrono::steady_clock::now();
    const double resolution = map.info.resolution;
    const double map_width_m = static_cast<double>(map.info.width) * resolution;
    const double map_height_m = static_cast<double>(map.info.height) * resolution;
    origin_x_ = map.info.origin.position.x;
    origin_y_ = map.info.origin.position.y;
    grid_width_ =
        static_cast<int>(std::floor(map_width_m / parameters_.grid_step_m)) + 1;
    grid_height_ =
        static_cast<int>(std::floor(map_height_m / parameters_.grid_step_m)) + 1;
    if (grid_width_ <= 0 || grid_height_ <= 0) {
        return;
    }

    const std::size_t point_count =
        static_cast<std::size_t>(grid_width_) * static_cast<std::size_t>(grid_height_);
    entries_.assign(
        point_count * static_cast<std::size_t>(parameters_.heading_bins), Entry{});

    const int clearance_cells =
        static_cast<int>(std::ceil(parameters_.clearance_m / resolution));
    const auto cell_value = [&](int cell_x, int cell_y) -> int {
        if (cell_x < 0 || cell_y < 0 ||
            cell_x >= static_cast<int>(map.info.width) ||
            cell_y >= static_cast<int>(map.info.height)) {
            return 100;
        }
        return map.data[
            static_cast<std::size_t>(cell_y) * map.info.width +
            static_cast<std::size_t>(cell_x)];
    };
    const int occupied_threshold = relocalization.parameters().occupied_threshold;
    // 격자점 채택 조건: 중심은 주행 가능(free, include_unknown이면 unknown도),
    // clearance 반경 안에 "occupied"만 없으면 됩니다. 반경 전체 free를
    // 요구하면 SLAM 맵의 노이즈 픽셀(unknown 반점) 하나가 주행로 위
    // 격자점을 죽여 커버리지 구멍을 만듭니다.
    const auto is_center_ok = [&](int cell_x, int cell_y) -> bool {
        const int value = cell_value(cell_x, cell_y);
        if (value >= occupied_threshold) {
            return false;
        }
        return parameters_.include_unknown || value >= 0;
    };
    const auto is_free = [&](int cell_x, int cell_y) -> bool {
        // clearance 검사용: occupied만 아니면 통과.
        return cell_value(cell_x, cell_y) < occupied_threshold;
    };

    std::vector<std::array<double, 3>> poses;
    std::vector<std::size_t> entry_index;
    poses.reserve(point_count);
    entry_index.reserve(point_count);
    const double heading_step = kTwoPi / static_cast<double>(parameters_.heading_bins);
    std::size_t valid_points = 0;
    for (int iy = 0; iy < grid_height_; ++iy) {
        for (int ix = 0; ix < grid_width_; ++ix) {
            const int cell_x = static_cast<int>(
                std::floor(ix * parameters_.grid_step_m / resolution));
            const int cell_y = static_cast<int>(
                std::floor(iy * parameters_.grid_step_m / resolution));
            if (!is_center_ok(cell_x, cell_y)) {
                continue;
            }
            bool clear_here = true;
            for (int dy = -clearance_cells; dy <= clearance_cells && clear_here; ++dy) {
                for (int dx = -clearance_cells; dx <= clearance_cells; ++dx) {
                    if (!is_free(cell_x + dx, cell_y + dy)) {
                        clear_here = false;
                        break;
                    }
                }
            }
            if (!clear_here) {
                continue;
            }
            ++valid_points;
            const std::size_t base =
                (static_cast<std::size_t>(iy) * static_cast<std::size_t>(grid_width_) +
                 static_cast<std::size_t>(ix)) *
                static_cast<std::size_t>(parameters_.heading_bins);
            for (int bin = 0; bin < parameters_.heading_bins; ++bin) {
                poses.push_back({
                    origin_x_ + ix * parameters_.grid_step_m,
                    origin_y_ + iy * parameters_.grid_step_m,
                    static_cast<double>(bin) * heading_step});
                entry_index.push_back(base + static_cast<std::size_t>(bin));
            }
        }
    }
    if (poses.empty()) {
        return;
    }

    // 유일한 입력: 각 (지점, heading)에서의 raycast.
    // 자체 사거리(실효 반사거리) + unknown 통과로 raycast합니다.
    const std::vector<float> scans = relocalization.synthesizeScans(
        poses, fov_rays, parameters_.max_range_m, 0);
    if (scans.size() != poses.size() * fov_rays) {
        return;
    }

    const double ray_step = fov_deg * kPi / 180.0 / static_cast<double>(fov_rays);
    const double fov_start = -0.5 * fov_deg * kPi / 180.0;
    const double max_hit_range =
        parameters_.max_range_m - std::max(resolution * 0.5, 1.0e-3);

    const unsigned hardware = std::max(1u, std::min(4u, std::thread::hardware_concurrency()));
    const std::size_t worker_count = std::min<std::size_t>(hardware, poses.size());
    const auto work = [&](std::size_t begin, std::size_t end) {
        for (std::size_t index = begin; index < end; ++index) {
            entries_[entry_index[index]] = computeEntry(
                scans.data() + index * fov_rays,
                fov_rays, poses[index][2] + fov_start, ray_step,
                max_hit_range, resolution);
        }
    };
    if (worker_count <= 1) {
        work(0, poses.size());
    } else {
        std::vector<std::thread> workers;
        workers.reserve(worker_count);
        const std::size_t chunk = (poses.size() + worker_count - 1) / worker_count;
        for (std::size_t w = 0; w < worker_count; ++w) {
            const std::size_t begin = w * chunk;
            const std::size_t end = std::min(poses.size(), begin + chunk);
            if (begin >= end) break;
            workers.emplace_back(work, begin, end);
        }
        for (auto &worker : workers) worker.join();
    }

    const auto end_time = std::chrono::steady_clock::now();
    diagnostics_.grid_points = point_count;
    diagnostics_.valid_points = valid_points;
    diagnostics_.entries = poses.size();
    diagnostics_.build_seconds =
        std::chrono::duration<double>(end_time - start_time).count();
    diagnostics_.memory_bytes =
        static_cast<double>(entries_.size()) * static_cast<double>(sizeof(Entry));
    ready_ = true;
}

ObservabilityMap::Sample ObservabilityMap::query(
    double world_x, double world_y, double heading) const {
    Sample sample;
    if (!ready_) {
        return sample;
    }

    const double fx = (world_x - origin_x_) / parameters_.grid_step_m;
    const double fy = (world_y - origin_y_) / parameters_.grid_step_m;
    const int ix0 = static_cast<int>(std::floor(fx));
    const int iy0 = static_cast<int>(std::floor(fy));
    const double tx = fx - static_cast<double>(ix0);
    const double ty = fy - static_cast<double>(iy0);

    const double heading_step = kTwoPi / static_cast<double>(parameters_.heading_bins);
    const double fh = normalizePositiveAngle(heading) / heading_step;
    const int bin0 = static_cast<int>(std::floor(fh)) % parameters_.heading_bins;
    const int bin1 = (bin0 + 1) % parameters_.heading_bins;
    const double th = fh - std::floor(fh);

    double mxx = 0.0, mxy = 0.0, myy = 0.0;
    double weight_sum = 0.0;
    double normals = 0.0;
    const std::array<std::array<int, 2>, 4> corners = {{
        {{ix0, iy0}}, {{ix0 + 1, iy0}}, {{ix0, iy0 + 1}}, {{ix0 + 1, iy0 + 1}}}};
    const std::array<double, 4> corner_weights = {
        (1.0 - tx) * (1.0 - ty), tx * (1.0 - ty), (1.0 - tx) * ty, tx * ty};
    const std::array<int, 2> bins = {bin0, bin1};
    const std::array<double, 2> bin_weights = {1.0 - th, th};

    for (std::size_t ci = 0; ci < corners.size(); ++ci) {
        for (std::size_t bi = 0; bi < bins.size(); ++bi) {
            const Entry *item = entry(corners[ci][0], corners[ci][1], bins[bi]);
            if (item == nullptr || !item->valid) {
                continue;
            }
            const double weight = corner_weights[ci] * bin_weights[bi];
            if (weight <= 0.0) {
                continue;
            }
            weight_sum += weight;
            mxx += weight * static_cast<double>(item->mxx);
            mxy += weight * static_cast<double>(item->mxy);
            myy += weight * static_cast<double>(item->myy);
            normals += weight * static_cast<double>(item->normal_count);
        }
    }
    if (weight_sum <= 1.0e-9) {
        // 코너가 전부 무효면(커버리지 구멍/경계) 가까운 유효 격자점으로
        // 스냅합니다. 작은 구멍 하나가 스캔 신뢰 전체를 꺼버리지 않도록.
        const int center_x = static_cast<int>(std::llround(fx));
        const int center_y = static_cast<int>(std::llround(fy));
        const int snap_radius = std::max(0, parameters_.snap_radius_cells);
        for (int radius = 1; radius <= snap_radius && weight_sum <= 1.0e-9; ++radius) {
            for (int dy = -radius; dy <= radius && weight_sum <= 1.0e-9; ++dy) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    if (std::abs(dx) != radius && std::abs(dy) != radius) {
                        continue;
                    }
                    for (std::size_t bi = 0; bi < bins.size(); ++bi) {
                        const Entry *item =
                            entry(center_x + dx, center_y + dy, bins[bi]);
                        if (item == nullptr || !item->valid) {
                            continue;
                        }
                        const double weight = bin_weights[bi];
                        weight_sum += weight;
                        mxx += weight * static_cast<double>(item->mxx);
                        mxy += weight * static_cast<double>(item->mxy);
                        myy += weight * static_cast<double>(item->myy);
                        normals += weight * static_cast<double>(item->normal_count);
                    }
                    if (weight_sum > 1.0e-9) {
                        break;
                    }
                }
            }
        }
        if (weight_sum <= 1.0e-9) {
            return sample;
        }
    }
    const double inverse_weight = 1.0 / weight_sum;
    mxx *= inverse_weight;
    mxy *= inverse_weight;
    myy *= inverse_weight;
    normals *= inverse_weight;

    Eigen::Matrix2d sensitivity;
    sensitivity << mxx, mxy, mxy, myy;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(sensitivity);
    if (solver.info() != Eigen::Success) {
        return sample;
    }

    // 오름차순(작은 고유값 먼저)이므로 뒤집어 강축을 앞에 둡니다.
    double lambda_strong = std::max(0.0, solver.eigenvalues()(1));
    double lambda_weak = std::max(0.0, solver.eigenvalues()(0));
    sample.eigenvectors.col(0) = solver.eigenvectors().col(1);
    sample.eigenvectors.col(1) = solver.eigenvectors().col(0);

    // 보수적 풀링: 방향은 평균 M의 고유벡터로 정하되, 각 방향의 lambda는
    // 참여한 이웃 entry들 중 최소값을 취합니다. 한 이웃에서라도 그 방향
    // 구속이 사라지면(가림/분할 경계) 퇴화로 판정 - 한계 특징이 격자
    // 사이에서 나타났다 사라졌다 하며 만드는 계단 점프를 없앱니다.
    if (parameters_.conservative_query) {
        // 가장 가까운 격자점 중심의 대칭 3x3 이웃(±1 격자, 인접 heading bin)
        // 에 대해 방향별 lambda 최소값을 취합니다. 격자점 정중앙에서도
        // 대칭으로 동작하고, 퇴화 주머니에 접근하는 소비자가 한 격자 먼저
        // 보수적으로 전환됩니다.
        const int center_x = static_cast<int>(std::llround(fx));
        const int center_y = static_cast<int>(std::llround(fy));
        for (int axis = 0; axis < 2; ++axis) {
            const Eigen::Vector2d v = sample.eigenvectors.col(axis);
            double pooled = 1e30;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    for (std::size_t bi = 0; bi < bins.size(); ++bi) {
                        const Entry *item =
                            entry(center_x + dx, center_y + dy, bins[bi]);
                        if (item == nullptr || !item->valid) {
                            continue;
                        }
                        const double value =
                            v.x() * v.x() * item->mxx +
                            2.0 * v.x() * v.y() * item->mxy +
                            v.y() * v.y() * item->myy;
                        pooled = std::min(pooled, value);
                    }
                }
            }
            if (pooled < 1e29) {
                if (axis == 0) lambda_strong = std::max(0.0, pooled);
                else lambda_weak = std::max(0.0, pooled);
            }
        }
        if (lambda_weak > lambda_strong) std::swap(lambda_weak, lambda_strong);
    }
    sample.lambda(0) = lambda_strong;
    sample.lambda(1) = lambda_weak;

    const auto to_sigma = [&](double lambda) {
        if (lambda <= 0.0) {
            return parameters_.max_sigma_m;
        }
        return std::min(parameters_.max_sigma_m,
                        parameters_.slip_tolerance_m / std::sqrt(lambda));
    };
    sample.sigma(0) = to_sigma(lambda_strong);
    sample.sigma(1) = to_sigma(lambda_weak);

    sample.sensitivity = sensitivity;
    sample.hit_fraction = std::clamp(mxx + myy, 0.0, 1.0);
    sample.normal_count = static_cast<std::size_t>(normals + 0.5);
    sample.valid = true;
    return sample;
}
