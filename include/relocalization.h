#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// 상대주축 인덱스(relative-axis index) 기반 전역 위치추정.
//
// 핵심 아이디어
//   맵의 free cell마다 360도 ray-cast profile을 미리 만들어 둔다. 어떤 heading
//   psi로 서 있으면 센서는 그 profile의 [psi - fov/2, psi + fov/2] 구간만 본다.
//   그 창 안의 끝점으로 PCA를 돌리면 map frame 주축 A(psi)가 나오고, 센서
//   프레임에서 실제로 "관측될" 주축은 b(psi) = A(psi) - psi (mod pi)다.
//
//   b는 오프라인에서 전부 계산할 수 있으므로, 라이브 스캔의 주축 a_s가 주어지면
//   b(psi) = a_s 를 만족하는 psi만 heading 후보가 된다. 즉 "내 스캔에서 주축이
//   오른쪽 50도로 보였다"면 맵에서도 오른쪽 50도로 보이는 (위치, heading) 조합만
//   검사하면 된다. 나머지는 range 정합을 돌리기 전에 통째로 걸러진다.
//
//   A(psi)는 psi가 조금 변해도 거의 그대로인 구간이 길다(복도/벽면). 그래서
//   720개 psi를 그대로 저장하지 않고 A가 tolerance 안에서 일정한 구간을 하나의
//   run으로 압축한다. run 안에서는 A가 상수이므로 b(psi) = A_r - psi 가 psi에
//   대해 선형이고, psi = A_r - a_s (mod pi)로 스캔 없이 즉시 역산된다.
class Relocalization {
public:
    enum class ScoreMethod {
        MaskedMse,
        Huber
    };

    struct Parameters {
        // OccupancyGrid에서 이 값 이상인 셀은 장애물로 취급합니다.
        int occupied_threshold{65};
        bool unknown_is_occupied{true};
        // 360도를 나누는 ray 개수입니다. 720이면 ray 간격은 0.5도입니다.
        // heading 가설도 이 격자로 양자화되므로 yaw 분해능도 같이 정해집니다.
        std::size_t ray_count{720};
        // RangeLibc에 한 번에 전달할 후보 위치 개수입니다.
        std::size_t raycast_batch_size{512};
        std::size_t minimum_scan_points{10};
        double max_range{30.0};
        // 후보 비교에 사용할 거리 손실 함수입니다.
        ScoreMethod score_method{ScoreMethod::MaskedMse};
        // Huber에서 제곱 손실과 선형 손실이 바뀌는 거리 기준[m]입니다.
        double huber_delta{0.5};

        // ----- 주축 인덱스 -----
        // 라이브 스캔의 시야각[도]. 오프라인 슬라이딩 창 폭을 정합니다.
        double fov_deg{270.0};
        // 라이브 스캔 축 추정을 Huber-IRLS로 강건화할지 여부입니다.
        // 오프라인 A(psi)는 러닝합 기반이라 항상 비가중 PCA입니다.
        bool robust_axis{false};
        int robust_axis_iterations{2};
        // IRLS 가중에서 인라이어/아웃라이어가 갈리는 수직거리[m]입니다.
        double robust_axis_delta{0.5};
        // A(psi)가 이 각[도] 안에서 유지되면 같은 run으로 압축합니다.
        // 크게 잡으면 메모리와 run 수가 줄지만 heading 역산 오차가 커집니다.
        double axis_run_tolerance_deg{3.0};
        // 스캔 축과 b(psi)의 허용 각오차[도]. 이 값이 곧 pruning 강도입니다.
        double axis_match_tolerance_deg{1.0};
        // 역산된 heading 주변으로 추가 평가할 ray 격자 칸 수(+-)입니다.
        // 스캔 축 추정 오차를 range 정합으로 흡수하기 위한 국소 탐색입니다.
        int yaw_refine_bins{2};
        // 한 후보에서 실제로 range 정합을 돌릴 heading 가설의 최대 개수입니다.
        std::size_t max_heading_hypotheses{12};

        // ----- 관측성(queryStaticGeometry) -----
        // 법선을 추정할 때 창이 벽 위에서 덮을 목표 호길이[m]입니다.
        // 인접 빔만 쓰면 r * ray_step 이 격자 해상도보다 작아지는 근거리
        // (0.5도/0.05m 기준 5.7m 이내)에서 접선이 격자 방향으로 스냅되어
        // 퇴화가 실제보다 약하게 읽힙니다. 거리에 따라 창을 넓혀 막습니다.
        double normal_chord_m{0.30};
        // 창 반폭 상한[ray]입니다. 아주 가까운 벽에서 창이 과도해지는 것을 막습니다.
        int normal_max_half_window{20};
        // 창 점들이 직선에서 벗어난 RMS가 이 값[m]을 넘으면 모서리/잡동사니로
        // 보고 그 빔의 법선을 버립니다.
        double normal_residual_limit_m{0.06};
        // 관측성 신뢰도 포화 기준[유효 빔 수]입니다.
        // confidence(v) = lambda(v) / (lambda(v) + 이 값) 으로 [0,1)에 사상하며,
        // lambda가 이 값과 같을 때 0.5가 됩니다. 비율(lambda/lambda_max)과 달리
        // "두 축이 동시에 나쁨"을 표현할 수 있습니다.
        double observability_reference{40.0};

        // ----- 위치 coarse-to-fine 탐색 -----
        bool coarse_to_fine{true};
        // coarse 격자 간격[m]과 상위 후보 주변을 조밀 탐색할 반경[m]입니다.
        double coarse_position_step_m{0.30};
        double fine_position_radius_m{0.30};
        // coarse에서 유지하여 fine으로 넘길 상위 후보 개수입니다.
        std::size_t top_candidates{24};

        // ----- 다중 가설 반환 -----
        // 복도처럼 프로파일이 앨리어싱되는 맵에서는 단일 최적해 확정이
        // 위험하므로(0526-1 초기화 실패 사례) 상위 가설 목록을 반환해
        // 파티클 필터가 주행으로 판별하게 합니다.
        std::size_t max_hypotheses{5};
        // best 점수(음수, 0에 가까울수록 좋음) 대비 이 배율 안의 후보만
        // 가설로 인정합니다. 3.0이면 평균 오차가 best의 3배 이내입니다.
        double hypothesis_score_ratio{3.0};
        // 이 거리/각도보다 가까운 후보는 같은 가설로 보고 흡수합니다.
        // 각도 기준이 별도라 같은 위치의 180도 플립은 다른 가설로 남습니다.
        double hypothesis_min_separation_m{0.8};
        double hypothesis_min_separation_deg{25.0};
        // 가설의 절대 품질 하한 — 예측 프로파일 대비 RMS 오차[m].
        //
        // hypothesis_score_ratio는 best 대비 '상대' 기준이라, best 자체가
        // 쓰레기면 하한도 같이 무너진다(실측 icra: best score -20.6 = RMS
        // 4.5 m인데 ratio 3.0의 하한은 -61.8이라 무사통과 -> 맵과 전혀 안
        // 맞는 자리에 시드 -> 곧바로 재리로컬). 절대 기준을 함께 둔다.
        // 정상 시드는 RMS 0.17~0.63 m 범위였다. 0 이하면 비활성.
        double hypothesis_max_rms_m{1.0};

        // ----- 가설 검증 게이트 -----
        // 후보 pose의 예측 프로파일 대비 실측 인라이어(|오차|<inlier_m)
        // 비율이 이 값 미만이면 탈락 — 검증 통과 후보만 파티클 시드가
        // 됩니다(0이면 비활성).
        double hypothesis_verify_fraction{0.7};
        double hypothesis_verify_inlier_m{0.2};

        // ----- 부호 인식(원사이드) 검증 게이트 -----
        // 잔차 e = 실측 - 예측의 '부호'가 결정적 정보다. 미지도 물체/사람/
        // 잔해는 빔을 짧게만 만들 수 있으므로(e<0), 정답 pose에서 e>0(예측한
        // 벽 너머가 보임)은 맵 오류나 유리 말고는 설명되지 않는다. 반면 오답
        // pose는 양쪽 부호가 섞인다(복도 종방향 오프셋이면 한쪽 끝은 e>0).
        // |e|만 보는 RMS와 인라이어 비율은 이 정보를 통째로 버려서 서로
        // 반대 판정을 냈다(실측: RMS 0.36인데 inlier 0.54로 거부 -> 8초 뒤
        // RMS 0.86이 통과).
        //
        // false면 기존 인라이어 게이트로 판정하고 통계만 계산한다(섀도 모드).
        bool verify_signed_gate{false};
        // e < -이 값이면 '가림'으로 보고 분모에서 제외한다.
        double verify_occlusion_m{0.2};
        // e > +이 값이면 'see-through'(예측 벽 관통) = 오답의 증거.
        double verify_see_through_m{0.25};
        // 가시 빔 중 see-through 허용 상한. 5%는 유리문 하나(5m에서 0.9m 폭
        // = 약 10도)가 예산을 통째로 먹으므로 10%에서 시작한다.
        double verify_see_through_max{0.10};
        // 가시 빔(전체 - 가림) 중 인라이어 비율 하한.
        double verify_visible_fraction{0.8};
        // 가림이 분모를 다 먹어치운 퇴화 pose를 막는 커버리지 하한.
        double verify_min_visible_frac{0.40};
        // 30도 섹터(720빔 격자에서 60빈)당 가시 빔이 이 개수 이상이면 유효.
        std::size_t verify_min_sector_beams{15};
        // 유효 섹터가 이 개수 이상이어야 한다(4 = 실효 120도).
        std::size_t verify_min_visible_sectors{4};

        // ----- multi-scan 공동 채점 -----
        // 최신 스캔의 후보 pool에서 이 개수까지 골라 과거 스캔들과 공동
        // 채점합니다. 단일 스캔은 닮은꼴 복도/루프를 구분하지 못하지만
        // (0526-1 전역 앨리어스), 코너를 포함한 궤적 모양은 유일합니다.
        std::size_t multi_scan_top_candidates{64};
        // 이력 공동 채점의 집계 방식 — 최신 1장 + '유효 이력 상위 비율'의 평균.
        //
        // 전부 평균내면 오염된 한 장이 모든 후보를 함께 떨어뜨리고(실측 icra:
        // 전복 뒤 30초 가설 0개), 반대로 상위 1장만 쓰면 나머지가 일제히
        // 반대한다는 증거를 계산해놓고 소거한다(실측: 텀블 중 0.1초 만에
        // 17m 오시드가 모든 게이트를 통과). 단방향 절사가 두 실패를 함께 막는다:
        // 오염은 점수를 끌어내리기만 하므로 하위를 자르고, 요행 정합은 상위
        // 소수에만 생기므로 상위 60% 평균이면 희석된다. 앨리어스에게
        // "요행 1장"이 아니라 "요행 다수"를 요구하게 된다.
        double history_support_fraction{0.6};
        // 재투영 DR 거리가 이보다 먼 이력 스캔은 집계에서 제외(노후 방어).
        double history_valid_dr_max_m{4.0};
        // 뷰 자격: 가시분율이 이보다 낮으면 기권(채점에서 제외). 점수가 가시
        // 기준이라 근맹 뷰가 우연히 최고 점수를 가져가는 것을 막는다.
        // 채점은 경성 0.5, 검증은 연성 가중(w_vis 램프)으로 분리한다.
        double history_min_visible_frac{0.5};
        // ----- 이력 가중 다수결 (결정 2) -----
        // 뷰별 판정은 절대 RMS 컷을 쓸 수 없다 — 씬별 스케일 차이로 지지/반대
        // 대역이 겹친다(실측: icra 지지 뷰 1.18~1.51 vs busan2 반대 뷰
        // 1.50~2.36). 뷰 '내부'에서 자기 최선 대비 상대 비교를 한다.
        bool history_majority_gate{false};   // false면 판정은 안 하고 로깅만
        // 뷰 최선 대비 통과 밴드: visRMS <= max(ratio*best, best + slack).
        // 가산 슬랙이 없으면 best가 작을 때(0.2) 잡음 수준 차이로 탈락한다.
        double history_pass_ratio{1.5};
        double history_pass_slack_m{0.3};
        // 어떤 후보도 이만큼 이하로 설명하지 못하는 뷰는 기권(무의견).
        double history_view_useless_m{1.5};
        // 가시분율 -> 검증 가중 램프.
        double history_weight_low_f{0.3};
        double history_weight_high_f{0.7};
        // 통과율 요구와 증거량 하한.
        double history_majority_fraction{0.5};
        double history_min_support_weight{2.0};
        // false면 점수는 예전 방식(최신 + 과거 최고 1장)으로 내고 새 집계는
        // 로깅만 한다(섀도 모드).
        bool history_new_aggregate{false};

        // ----- 궤적-모양 전역 정합 -----
        // 누적 DR 궤적을 free-공간에 회전x평행이동 전역 탐색으로 정합
        // 합니다. 주행로 배치는 벽 디테일이 낡아도 유지되므로, 스캔
        // 채점이 앨리어스에 속는 맵에서도 진짜 배치를 찾습니다
        // (0526-1/0529 오프라인 검증: free 비율 97~98%로 유일 최적).
        double trajectory_rot_step_deg{3.0};
        double trajectory_translation_step_m{0.2};
        // free 비율이 best - band 이내인 후보만 살리고, 살아남은 후보는
        // 현재 스캔 정합 점수로 순위를 매깁니다(타이브레이크).
        double trajectory_score_band{0.05};
        std::size_t trajectory_keep{8};
    };

    // 부호 인식 검증의 원재료입니다. 비율이 아니라 정수 카운트를 남기는 이유는
    // 나중 소프트 시딩에서 "0.9가 40빔짜리인지 400빔짜리인지"가 신뢰도 보정에
    // 필요하기 때문입니다(라플라스/윌슨 보정에 N이 있어야 함).
    struct VerifyStats {
        uint16_t total{0};           // 채점에 쓴 유효 빔
        uint16_t occluded{0};        // e < -occlusion_m : 가림, 분모 제외
        uint16_t inlier{0};          // 대역 +- inlier_m 이내
        uint16_t see_through{0};     // e > +see_through_m : 예측 벽 관통
        uint16_t visible_sectors{0}; // 가시 빔이 충분한 30도 섹터 수
        float visible_rms{0.0f};     // 가시 빔만의 RMS[m] (대역 안은 오차 0)
        float visible_inlier_frac{0.0f};
        float see_through_frac{0.0f};
        float visible_frac{0.0f};
        bool pass{false};
        // bit0 커버리지, bit1 섹터, bit2 인라이어, bit3 see-through, bit4 RMS.
        // 단락 없이 전부 평가해 채웁니다 — 섀도 모드에서 "무엇이 죽였나"를
        // 한 줄로 읽기 위한 것입니다.
        uint8_t fail_mask{0};
    };

    // relocalizeMultiple이 돌려주는 전역 pose 가설입니다(점수 내림차순).
    struct Hypothesis {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
        double score{0.0};
        // 검증 인라이어 비율(|빔 오차| < verify_inlier_m 비율).
        double inlier{0.0};
        VerifyStats verify{};
        // 이 후보에 대한 이력 뷰별 (가시 RMS, 가시분율). 시드된 가설의 지지
        // 패턴을 봐야 하므로 후보마다 들고 다닌다 — 진단 배열에 1등 후보만
        // 담으면 집계 후 순위가 바뀌어 다른 후보의 값을 보게 된다.
        std::array<float, 16> view_rms{};
        std::array<float, 16> view_visible{};
        std::uint8_t view_count{0};
        float support_ratio{0.0f};
        float support_weight{0.0f};
    };

    // 한 번의 relocalize 호출에서 무엇이 얼마나 걸러졌는지 보고합니다.
    struct Diagnostics {
        std::size_t candidates_visited{0};
        std::size_t candidates_scored{0};
        std::size_t hypotheses_scored{0};
        double scan_axis{0.0};
        // 채택된 후보의 정합 점수(클수록 좋음)와 사용한 스캔 점 개수입니다.
        double best_score{0.0};
        std::size_t scan_points{0};
        // 가설 선별에서 무엇이 몇 개나 걸렸는지 — "후보가 없다"와 "후보는
        // 있는데 게이트에 걸렸다"를 구분하기 위한 계수입니다.
        std::size_t pool_size{0};
        std::size_t rejected_score_floor{0};
        std::size_t absorbed_duplicate{0};
        std::size_t rejected_verify{0};
        std::size_t rejected_verify_inlier{0};
        std::size_t rejected_verify_seethrough{0};
        std::size_t rejected_verify_coverage{0};
        std::size_t accepted{0};
        // 게이트 통과 여부와 무관한 최고 인라이어 비율.
        double best_inlier{0.0};
        // 점수 1등 후보의 신·구 집계 점수와 이력별 점수(포렌식/섀도 비교용).
        double top_score_old{0.0};
        double top_score_new{0.0};
        std::array<float, 16> top_view_scores{};
        // 같은 뷰의 가시분율 — "가림이 원인인가"를 판정하는 핵심 열.
        std::array<float, 16> top_view_visible{};
        std::uint8_t top_view_count{0};
        std::uint8_t top_view_valid{0};
        // 검증까지 도달한 후보 중 점수 1등의 통계(통과 여부 무관).
        // 거부된 후보를 봐야 어느 게이트가 정답을 죽였는지 알 수 있습니다.
        VerifyStats best_verify{};
        bool best_verify_valid{false};
    };

    explicit Relocalization(const nav_msgs::msg::OccupancyGrid &map);
    Relocalization(
        const nav_msgs::msg::OccupancyGrid &map,
        const Parameters &parameters);

    // 맵이나 파라미터가 바뀌면 모든 후보 지점의 ray-cast 특징을 다시 계산합니다.
    void setMap(const nav_msgs::msg::OccupancyGrid &map);
    void setParameters(const Parameters &parameters);
    const Parameters &parameters() const { return parameters_; }

    // 단일 스캔으로 전역 위치를 탐색합니다.
    // 반환 pose는 map 좌표계에서 본 해당 스캔 프레임의 pose입니다.
    geometry_msgs::msg::Pose relocalize(
        const sensor_msgs::msg::LaserScan &scan);

    // 단일 스캔으로 상위 가설 목록을 탐색합니다(점수 내림차순, 최소 1개).
    // 후보가 전혀 없으면 relocalize와 같은 예외를 던집니다.
    std::vector<Hypothesis> relocalizeMultiple(
        const sensor_msgs::msg::LaserScan &scan);

    // 스캔 사이의 라이다 프레임 상대 이동입니다(scans[k] -> scans[k+1]).
    struct RelativeMotion {
        double dx{0.0};
        double dy{0.0};
        double dyaw{0.0};
    };
    // 최신 스캔(scans.back())의 pose 가설을 과거 스캔들과 공동 채점합니다.
    // motions는 scans.size()-1개여야 하며, 반환 pose는 최신 스캔 기준입니다.
    // 후보 pose에서 상대 이동을 역산한 과거 pose가 free 셀을 벗어나면 큰
    // 페널티를 받으므로, 닮은꼴이라도 궤적 전체가 들어맞는 배치만 남습니다.
    std::vector<Hypothesis> relocalizeMultiple(
        const std::vector<sensor_msgs::msg::LaserScan> &scans,
        const std::vector<RelativeMotion> &motions);

    // 누적 DR 궤적(오래된 것부터, 마지막 점 = 현재 라이다 위치, DR 프레임)
    // 을 free-공간에 전역 정합해 현재 pose 가설을 반환합니다. current_yaw는
    // DR 프레임의 현재 yaw로, 반환 가설의 yaw = current_yaw + 찾은 회전.
    // 궤적이 랩 반복 루프면 free 비율만으로는 루프 위상(어느 지점인지)이
    // 갈리지 않으므로, scans(과거+현재, relocalizeMultiple과 같은 구조)의
    // multi-scan 공동 채점으로 살아남은 배치들의 순위를 정합니다.
    std::vector<Hypothesis> relocalizeTrajectory(
        const std::vector<std::array<double, 2>> &points,
        double current_yaw,
        const std::vector<sensor_msgs::msg::LaserScan> &scans,
        const std::vector<RelativeMotion> &motions);

    // 다중 스캔 경로는 별도 방식으로 구현할 예정이므로 현재는 스켈레톤만 유지합니다.
    geometry_msgs::msg::Pose relocalize(
        const std::vector<sensor_msgs::msg::LaserScan> &scans,
        const std::vector<geometry_msgs::msg::Transform> &current_from_next);

    // 직전 relocalize 호출의 탐색 통계입니다.
    const Diagnostics &lastDiagnostics() const { return diagnostics_; }

    // 사전계산 규모를 확인하기 위한 값들입니다(후보 수, run 총 개수).
    std::size_t candidateCount() const { return processed_map_.candidates.size(); }
    std::size_t axisRunCount() const { return processed_map_.axis_runs.size(); }

    // 파티클 필터가 쓰는 static 관측성 정보입니다. 라이브 스캔이 아니라 맵
    // 사전계산 결과에서 뽑기 때문에, 맵에 없는 동적 장애물이 만드는 가짜
    // 기하 정보에 신뢰 방향이 오염되지 않습니다.
    struct StaticGeometry {
        bool valid{false};
        // 실제 사용한 사전계산 후보의 위치입니다(요청 위치에서 가장 가까운 셀).
        double source_x{0.0};
        double source_y{0.0};
        // 270도 창 안의 벽 법선으로 만든 정보행렬 I = sum(n * n^T) 입니다.
        // 단위는 "유효 빔 수"이며 개수로 나누지 않습니다. 나누면 trace(I)가
        // 항상 1이 되어 lambda0 + lambda1 = 1 로 고정되고, 지표의 자유도가
        // 하나로 축퇴되어 "두 축이 동시에 나쁨"을 표현할 수 없게 됩니다.
        Eigen::Matrix2d information{Eigen::Matrix2d::Zero()};
        // information의 고유분해입니다. 열이 고유벡터이고 고유값은 내림차순입니다.
        Eigen::Matrix2d eigenvectors{Eigen::Matrix2d::Identity()};
        Eigen::Vector2d eigenvalues{Eigen::Vector2d::Zero()};
        // 축별 신뢰도 lambda / (lambda + observability_reference), 내림차순.
        // 소비자(융합/노이즈 확산)는 이 값만 쓰면 되고, 기준값이 한 곳에만
        // 있으므로 두 소비자가 서로 다른 정규화를 하는 일이 생기지 않습니다.
        Eigen::Vector2d confidence{Eigen::Vector2d::Zero()};
        std::size_t normal_count{0};
    };

    // (world_x, world_y)에서 가장 가까운 사전계산 후보의 360도 profile을 가져와
    // heading 기준 fov_deg 창으로 자른 뒤 관측성 정보행렬을 계산합니다.
    StaticGeometry queryStaticGeometry(
        double world_x,
        double world_y,
        double heading) const;

    // 벤치/테스트용 합성 스캔 생성기입니다. 내부 맵을 raycast하여 각
    // pose(x, y, yaw)에서 [-fov/2, fov/2] 구간 fov_rays개 range[m]를 만듭니다
    // (센서 프레임 bearing 순서, miss는 max_range). 반환은 poses 수 * fov_rays
    // 크기의 행우선 벡터입니다. RangeLibc를 이 번역단위에 가두어 벤치가
    // RangeLibc 헤더를 직접 포함하지 않도록 하는 목적도 있습니다.
    // max_range_override > 0 이면 그 사거리로, unknown_override >= 0 이면
    // 그 값(0/1)으로 unknown 취급을 바꿔 raycast합니다. 관측성 사전계산은
    // relocalization(원거리, unknown=벽)과 다른 설정(실효 반사거리,
    // unknown 통과)을 쓰므로 여기서 분기합니다. OMap은 호출마다 새로
    // 만들기 때문에 추가 비용이 없습니다.
    std::vector<float> synthesizeScans(
        const std::vector<std::array<double, 3>> &poses,
        std::size_t fov_rays,
        double max_range_override = 0.0,
        int unknown_override = -1) const;

private:
    // 각 행은 한 후보 지점이고 열은 ray 순서대로 range[m] 하나씩입니다.
    // ray 방향(cos/sin)은 후보와 무관하게 같으므로 ray_cos/ray_sin에 한 번만
    // 두고, 후보별 저장은 range만 남겨 메모리와 캐시 미스를 3분의 1로 줄입니다.
    using RangeMatrix =
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    // 사전계산 후보가 가리키는 map cell 중심입니다.
    struct CandidateLocation {
        int32_t cell_x{0};
        int32_t cell_y{0};
        double world_x{0.0};
        double world_y{0.0};
    };

    // A(psi)가 tolerance 안에서 일정한 heading 구간입니다. 원형 인덱스이므로
    // [start, start + length) 를 ray_count로 modulo 하여 해석합니다.
    struct AxisRun {
        uint16_t start{0};
        uint16_t length{0};
        // 구간 대표 map frame 주축(undirected, (-pi/2, pi/2]).
        float axis{0.0f};
    };

    // 라이브 스캔을 heading 격자에 맞춰 미리 접어둔 형태입니다. yaw를 ray
    // 격자로 양자화하면 map ray 인덱스가 (base_index + yaw_index) % ray_count로
    // 정수 연산만으로 나오므로, 정합 루프에서 삼각함수가 완전히 사라집니다.
    struct ScanView {
        std::vector<int32_t> base_index;
        std::vector<float> range;
        double axis{0.0};
        std::size_t size() const { return range.size(); }
    };

    struct PoseCandidate {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
        double score{0.0};
        // 이력 뷰별 통계(공동 채점 경로에서만 채워짐).
        std::array<float, 16> view_rms{};
        std::array<float, 16> view_visible{};
        std::uint8_t view_count{0};
        // 가중 다수결 결과.
        float support_ratio{0.0f};   // sum(w*pass) / sum(w)
        float support_weight{0.0f};  // sum(w) — 증거의 양
    };

    struct SearchResult {
        bool valid{false};
        PoseCandidate best;
    };

    // 맵이 들어올 때 한 번 생성하고 relocalize 호출마다 재사용하는 데이터입니다.
    struct ProcessedMap {
        uint32_t width{0};
        uint32_t height{0};
        double resolution{0.0};
        double origin_x{0.0};
        double origin_y{0.0};
        double origin_yaw{0.0};
        std::vector<float> ray_angles;
        std::vector<float> ray_cos;
        std::vector<float> ray_sin;
        std::vector<CandidateLocation> candidates;
        RangeMatrix ranges;
        // cell 인덱스(row*width+col) -> candidates 인덱스, 후보가 아니면 -1입니다.
        // fine 단계에서 상위 후보 주변 셀을 상수 시간에 찾기 위한 역인덱스입니다.
        std::vector<int32_t> cell_to_candidate;
        // CSR 배치입니다. 후보 i의 run은 [axis_run_offsets[i], axis_run_offsets[i+1]).
        std::vector<AxisRun> axis_runs;
        std::vector<uint32_t> axis_run_offsets;
    };

    // 파라미터 검증과 맵 전체 사전계산 절차입니다.
    void validateParameters(const Parameters &parameters) const;
    void preprocessMap();
    // 후보는 로봇이 위치할 수 있는 free cell만 사용합니다.
    bool isCandidateCell(int32_t cell_x, int32_t cell_y) const;
    // map origin의 회전까지 반영하여 cell 중심을 map 좌표로 변환합니다.
    CandidateLocation makeCandidateLocation(int32_t cell_x, int32_t cell_y) const;

    // FOV 창 폭(ray 개수)입니다. fov_deg와 ray_count에서 유도합니다.
    std::size_t fovWindow() const;

    // 한 후보의 360도 range profile을 FOV 폭 창으로 슬라이딩하며 모든 psi에서
    // map frame 주축 A(psi)를 구하고, 인접한 값들을 run으로 압축합니다.
    // 반환은 이 후보가 만든 run 개수이며 결과는 processed_map_.axis_runs 뒤에
    // 이어 붙습니다.
    std::size_t buildAxisRuns(const float *ranges);

    // 점군의 주성분 축(undirected, (-pi/2, pi/2])을 구합니다.
    // robust_axis가 켜져 있으면 Huber-IRLS로 아웃라이어를 감쇠합니다.
    double principalAxisAngle(const std::vector<Eigen::Vector2d> &points) const;

    // LaserScan을 heading 격자에 맞춘 ScanView로 변환하고 주축까지 채웁니다.
    ScanView convertScan(const sensor_msgs::msg::LaserScan &scan) const;

    // 상대주축 인덱스로 후보를 걸러가며 전역 탐색합니다.
    SearchResult searchGlobalPose(
        const ScanView &scan,
        std::vector<PoseCandidate> *collector = nullptr);

    // 점수 내림차순 pool에서 점수 하한과 NMS(거리/각도)로 가설을 뽑습니다.
    // verify_view가 있으면 인라이어 비율 게이트를 통과한 후보만 남깁니다.
    std::vector<Hypothesis> selectHypotheses(
        const std::vector<PoseCandidate> &pool,
        double best_score,
        const ScanView *verify_view = nullptr) const;
    // (x,y,yaw) 예측 프로파일 대비 실측 인라이어 비율입니다.
    // 빔 분류(가림/인라이어/see-through, +-1빈 대역)를 한 곳에서 수행합니다.
    //
    // 채점과 검증이 반드시 같은 루틴을 소비해야 합니다. 예전에는 검증만
    // 부호를 알고 이력 채점은 부호맹이라 의미론이 갈렸고, 그 결과 가림이
    // 반대표로 오역됐습니다(실측 busan2: 40cm 장애물에 가려진 이력 9장이
    // 정답 pose에서 원시 RMS 1.45~2.91을 내 정답을 탈락 직전까지 밀어냄).
    // 경로가 하나면 이런 분열이 재발할 수 없습니다.
    VerifyStats computeBeamStats(
        double x, double y, double yaw, const ScanView &view) const;
    // 부호 인식 검증 = 빔 분류 + 게이트 판정.
    VerifyStats verifyPose(
        double x, double y, double yaw, const ScanView &view) const;
    // 모든 시드 후보의 단일 관문 — 표준 채점(latest + 가중 이력) + 게이트.
    // 경로마다 게이트를 중복 구현하면 언젠가 한 곳을 빠뜨리고, 점수 단위가
    // 갈리면 절대 하한과 상대 배율이 잴 것을 잃는다.
    std::vector<Hypothesis> finalizeCandidates(
        std::vector<PoseCandidate> &shortlist,
        const ScanView &latest,
        const std::vector<ScanView> &views,
        const std::vector<RelativeMotion> &offsets);
    static double legacyInlierFraction(const VerifyStats &stats);

    // motions(연속 상대이동)를 "최신 프레임에서 본 각 스캔의 오프셋"으로
    // 접습니다. 마지막 원소는 항상 (0,0,0)입니다.
    std::vector<RelativeMotion> accumulateOffsets(
        const std::vector<RelativeMotion> &motions) const;
    // (x,y,yaw)의 후보 셀 프로파일로 뷰를 채점합니다. 셀 밖/free 아님은
    // 큰 페널티(-50)입니다.
    double scorePoseWithProfile(
        double x, double y, double yaw, const ScanView &view) const;

    // 한 후보의 run에서 heading 가설(ray 격자 인덱스)을 역산합니다.
    // 반환이 false면 이 후보는 스캔 축과 양립할 수 없어 정합을 건너뜁니다.
    bool collectHeadingHypotheses(
        std::size_t candidate_index,
        double scan_axis,
        std::vector<int32_t> &yaw_indices) const;

    // 한 후보에서 heading 가설들을 평가해 최고 점수 pose를 반환합니다.
    // collector가 있으면 평가한 모든 (pose, yaw) 조합을 점수와 함께 담습니다
    // (다중 가설 NMS의 재료. 같은 위치의 경쟁 yaw도 잃지 않습니다).
    PoseCandidate evaluateCandidate(
        std::size_t candidate_index,
        const ScanView &scan,
        std::vector<int32_t> &yaw_scratch,
        std::vector<PoseCandidate> *collector = nullptr);

    // yaw를 ray 격자 인덱스로 양자화한 상태에서의 정합 점수입니다.
    // 연속 메모리 gather + reduction만 남아 GPU kernel로 옮기기 쉽습니다.
    double scoreCandidate(
        const float *candidate_ranges,
        const ScanView &scan,
        int32_t yaw_index) const;

    geometry_msgs::msg::Pose buildPose(const SearchResult &result) const;

    // 원본 맵과 파라미터를 소유하며 processed_map_은 이 둘에서 파생됩니다.
    nav_msgs::msg::OccupancyGrid map_;
    Parameters parameters_;
    ProcessedMap processed_map_;
    bool map_ready_{false};
    // selectHypotheses가 const 경로에서도 계수를 남깁니다.
    mutable Diagnostics diagnostics_;
};
