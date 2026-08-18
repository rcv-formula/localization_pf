#pragma once

#include <deque>
#include <queue>
#include <random>
#include <string>

#include <Eigen/Dense>

#include "structs.h"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"

struct scan_entropy{
    float theta;
    float min, max;
};

class particlePropagation{
    public:
    // 뒤차축 좌표계에서 IMU 좌표계까지의 정적 변환값입니다.
    // 호출부에서 ROS TF 변환을 적용한 뒤 이 값을 넘겨주는 것을 가정합니다.
    struct ImuExtrinsic {
        double rear_to_imu_x{0.25};
        double rear_to_imu_y{0.0};
        double rear_to_imu_z{0.0};
    };

    // 뒤차축 좌표계에서 라이다 좌표계까지의 정적 변환값입니다.
    // EKF는 뒤차축 pose를 추정하지만 파티클/스캔 pose는 라이다 기준이므로,
    // 회전할 때 생기는 lever arm 이동을 propagation에서 보정해야 합니다.
    // 기본값은 현재 차량 구성(뒤차축 앞 25cm)이며, 노드가 시작 시 TF를 한 번
    // 조회해 setLaserExtrinsic()으로 덮어쓰는 것을 전제로 합니다.
    struct LaserExtrinsic {
        double rear_to_laser_x{0.25};
        double rear_to_laser_y{0.0};
    };

    // 휠/IMU EKF, 시작 시 중력 보정, 저지연 목표시각 보간에 사용하는
    // 조정 가능한 파라미터 묶음입니다.
    struct EkfParameters {
        // VESC 카운트->미터 변환에 곱하는 보정 배율입니다. 데이터셋 리플레이는
        // shim이 같은 상수로 인코딩하므로 1.0이 맞고(맵/스캔과 자기일관 검증됨),
        // 실차에서 드라이버 상수가 다른 플랫폼이면 여기서 보정합니다(예: 2.6).
        double wheel_scale{1.0};
        // 휠 주행거리 산출 경로 선택.
        //  false(기본) : state.speed(ERPM)를 vesc_to_odom과 동일한 식으로
        //                속도[m/s]로 바꾼 뒤 적분해 누적거리를 만듭니다.
        //                  v = (speed - speed_to_erpm_offset) / speed_to_erpm_gain
        //  true        : state.displacement(타코미터 카운트)를
        //                displacement/6/motor_speed_gain 으로 환산합니다(구 경로).
        // ERPM 경로가 플랫폼의 vesc_to_odom과 같은 단위계라 기본값입니다.
        bool wheel_use_displacement{false};
        // ERPM <-> 속도 변환. f1tenth_stack vesc.yaml과 같은 의미:
        //   erpm = speed_to_erpm_gain * speed[m/s] + speed_to_erpm_offset
        double speed_to_erpm_gain{1538.0};
        double speed_to_erpm_offset{0.0};
        // vesc_to_odom과 동일한 저속 데드밴드[m/s].
        double erpm_speed_deadband{0.05};
        double min_dt{1.0e-4};
        double velocity_deadzone{0.03};
        double max_wheel_speed{30.0};
        double history_keep_time{1.0};
        double max_imu_extrapolation{0.02};
        double max_wheel_extrapolation{0.03};

        int yaw_rate_filter_window{7};
        int yaw_rate_filter_order{2};

        double wheel_velocity_variance{0.03 * 0.03};
        double gyro_roll_rate_variance{0.03 * 0.03};
        double gyro_pitch_rate_variance{0.03 * 0.03};
        double lateral_accel_variance{0.35 * 0.35};
        double slip_reference_variance{0.45 * 0.45};

        double slip_gain{20.0};
        double slip_power{1.0};
        double max_wheel_variance_scale{100.0};
        double lateral_update_gain{1.0};

        double load_gain{0.05};
        double load_ax_gain{1.0};
        double load_ay_gain{1.0};
        double load_pitch_gain{3.0};
        double load_roll_gain{3.0};

        double pitch_natural_frequency{8.0};
        double pitch_damping_ratio{0.8};
        double pitch_accel_gain{-0.02};
        double roll_natural_frequency{8.0};
        double roll_damping_ratio{0.8};
        double roll_accel_gain{0.02};

        // --- 경량 dead-reckoning(궤적 버퍼 = 100 Hz 출력의 보간 대상) ---
        // IMU 자세(AHRS quaternion)로 기울기를 보정합니다. 끄면 평지 가정.
        bool dr_use_imu_tilt{true};
        // 기준자세 대비 이보다 기울면 평면 주행이 아니므로(들어올림/전복)
        // 보정을 중단하고 평지 식으로 되돌립니다.
        double dr_tilt_max_deg{45.0};

        double gravity_pitch_sign{1.0};
        double gravity_roll_sign{-1.0};
        double accel_gravity_sign{1.0};

        bool enable_startup_gravity_calibration{true};
        std::string gravity_calibration_path{};
        double gravity_startup_timeout{10.0};
        double gravity_stability_window{2.0};
        double gravity_stability_score_min{0.8};
        int gravity_min_imu_samples{30};
        double gravity_gyro_mean_sigma{0.02};
        double gravity_gyro_std_sigma{0.01};
        double gravity_accel_std_sigma{0.08};
        double gravity_accel_norm_sigma{0.35};
        double gravity_wheel_delta_sigma{0.005};
        double gravity_wheel_velocity_sigma{0.02};

        double process_position_std{0.02};
        double process_yaw_std{0.01};
        double process_velocity_std{0.20};
        // 0이면 yaw-rate bias를 상수로 얼립니다(권장). 이 EKF에는 절대 heading
        // 관측이 없어 bias가 관측 불가능한데, 온라인으로 두면 roll/pitch-rate와
        // 휠 속도 업데이트가 공분산 교차항을 통해 bias를 밀고 그게 실제 회전을
        // 흡수합니다. 데이터셋 실측에서 43초에 yaw 오차 -180도, 동결 시 -4.2도.
        double process_gyro_bias_std{0.0};
        double process_accel_bias_std{0.01};
        double process_tilt_std{0.01};
        double process_tilt_rate_std{0.05};
    };

    // 파티클 예측에 주입할 노이즈 파라미터입니다. 모든 항이 이동량에 비례하므로
    // 정지 중에는 노이즈가 0에 수렴하고, 임계값 분기가 필요 없습니다.
    struct MotionNoise {
        // 전진 이동량 대비 종/횡 위치 노이즈 비율입니다.
        // 차량은 비홀로노믹이라 횡 성분을 작게 둡니다.
        double trans_per_trans{0.15};
        double lateral_per_trans{0.05};
        // 회전량 대비 회전 노이즈, 전진량 대비 회전 노이즈 비율입니다.
        double rot_per_rot{0.15};
        double rot_per_trans{0.03};
        // 기하 퇴화 방향으로 추가 확산시키는 배율 상한입니다.
        // s(v) = 1 + degeneracy_gain * (1 - lambda(v)/lambda_max)
        // 잘 관측되는 방향은 s=1, 완전 퇴화 방향은 s=1+degeneracy_gain입니다.
        double degeneracy_gain{2.0};
        // 스캔 정합도(health, 1=양호 0=불량)가 떨어질수록 노이즈를
        // m = 1 + recovery_gain * (1 - health) 배로 등방 확대합니다.
        // 정합이 무너졌다 = 추정이 어긋났을 가능성이므로 탐색 반경을
        // 넓혀 재포착을 돕습니다(문턱 분기 없음, 이동량 비례는 유지).
        double recovery_gain{3.0};
        // 사이클당 노이즈 바닥[m, rad]. 이동량 비례 노이즈가 저속에서 너무
        // 작아지는 것을 막아 최소한의 국소 탐색을 유지합니다(사용자 요청).
        double min_trans_sigma{0.02};
        double min_yaw_sigma{0.005};
    };

    // 직전 propagation이 파티클에 적용한 body frame 이동량입니다(라이다 기준,
    // lever arm 보정 포함). estimation 모듈이 같은 값을 재계산하지 않도록 노출합니다.
    struct MotionDelta {
        bool valid{false};
        double longitudinal{0.0};
        double lateral{0.0};
        double yaw{0.0};
    };

    // poseAt()이 돌려주는 궤적 위의 pose입니다.
    struct TrajectoryPose {
        bool valid{false};
        double time{0.0};
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
    };

    particlePropagation(float wheel_base = 0.25, float motor_speed_gain = 1350.0);

    // ---- L2: 상태를 바꾸는 writer ----
    // 목표 시각까지 풀 EKF를 진행시키고(bias/tilt/속도/공분산 갱신) 경량
    // dead-reckoning 기준을 재동기합니다. scan 트리거로 PF 직전에 한 번
    // 호출하는 것을 전제로 하며, 이 클래스에서 상태를 바꾸는 유일한 진입점입니다.
    void advanceTo(builtin_interfaces::msg::Time target_time);

    // ---- L2: 상태를 바꾸지 않는 reader ----
    // IMU 샘플마다 채워진 궤적 링버퍼를 보간해 해당 시각의 pose를 돌려줍니다.
    // 과거 시각 조회가 가능하므로, 100Hz 출력 경로가 앞서 나가더라도 PF는
    // scan 시각의 pose를 정확히 얻을 수 있습니다.
    // 기준 자세 대비 상대 기울기[deg] 중 큰 쪽(roll/pitch). 기준이 아직
    // 잡히지 않았으면 0. 전복/들림 구간의 스캔을 이력에서 빼는 데 씁니다.
    double relativeTiltDeg() const;
    // 같은 기준 대비 상대 roll/pitch 를 부호까지 따로 돌려줍니다[deg].
    // relativeTiltDeg 는 두 축의 최대 크기만 주므로 "어느 축으로 얼마나"
    // 기울었는지가 사라집니다. 이력 스냅샷에 자세를 남기려면 축별 값이
    // 필요합니다(라이다 평면이 어느 방향으로 틀어졌는지가 뷰마다 다름).
    void relativeTilt(double &roll_deg, double &pitch_deg) const;
    TrajectoryPose poseAt(double target_seconds) const;
    TrajectoryPose poseAt(builtin_interfaces::msg::Time target_time) const;

    // 직전 호출 이후의 이동량을 각 파티클에 적용하고 노이즈를 주입합니다.
    // target_time 버전은 궤적 버퍼만 읽는 순수 reader이므로, 호출 전에
    // advanceTo()로 EKF를 진행시켜 두어야 합니다(무인자 버전은 자동 수행).
    void propagation(particle *p_ptr, int32_t &particle_count);
    void propagation(
        particle *p_ptr,
        int32_t &particle_count,
        builtin_interfaces::msg::Time target_time);

    void setMotionNoise(MotionNoise motion_noise);
    // 노드가 base_link->laser TF를 한 번 조회해 넣어줍니다.
    void setLaserExtrinsic(LaserExtrinsic laser_extrinsic);
    // 맵 기반 static 관측성(Relocalization::queryStaticGeometry 결과)을 넣어
    // 정보가 적은 방향으로 더 크게 확산시킵니다. 라이브 스캔을 쓰지 않으므로
    // 동적 장애물이 만드는 가짜 기하 정보에 오염되지 않습니다.
    // confidence는 축별 신뢰도 [0,1]이며 내림차순 eigenvectors와 짝입니다.
    // 고유값 비율을 넘기면 안 됩니다 - 강축 비율은 항상 1이라 확산 배율이
    // 언제나 1로 고정되고, 개활지처럼 양쪽이 모두 나쁜 상황을 놓칩니다.
    // 직전 사이클 스캔 정합 건강도[0,1]. 노이즈 회복 확장에 씁니다.
    void setNoiseHealth(double health);
    void setNoiseShaping(
        const Eigen::Matrix2d &eigenvectors,
        const Eigen::Vector2d &confidence);
    void clearNoiseShaping();
    // relocalization으로 파티클을 다시 뿌린 직후 호출합니다. 다음 propagation이
    // 오래된 기준 pose와의 큰 차이를 한꺼번에 적용하는 것을 막습니다.
    void resetPropagationReference();
    // EKF의 '동역학' 상태만 재초기화합니다 — 속도, gyro/accel 바이어스,
    // 서스펜션 pitch/roll(+rate)와 그 공분산. odom 프레임 pose(x,y,yaw)는
    // 남깁니다(이력 체인·궤적 버퍼가 절대 odom pose 연속성을 전제).
    //
    // 용도: 리로컬 시드 직후. Lost 중 차를 들고 움직이면 휠(0)과 IMU(가속)의
    // 모순이 바이어스·자세 상태로 흡수되고, 시드는 pose만 고치므로 오염된
    // 바이어스가 시드 후 지속 드리프트를 만든다(스캔이 계속 잡아끄는 증상).
    void resetDynamicState();
    const MotionDelta &lastMotionDelta() const { return last_motion_delta_; }
    // 궤적 버퍼 최신 샘플의 시각[s]. 출력 보간이 얼마나 오래된 데이터를
    // 쓰는지(보간 레이턴시) 측정용. 버퍼가 비면 음수.
    double newestTrajectoryTime() const {
        return trajectory_.empty() ? -1.0 : trajectory_.back().time;
    }
    // 순간 슬립 지표[0,1]: 모델 횡가속 대비 IMU 실측 잔차의 포화 정규화.
    double lastSlipScore() const { return last_slip_score_; }
    void imuGetter(sensor_msgs::msg::Imu current_imu);
    void imuGetter(sensor_msgs::msg::Imu current_imu, ImuExtrinsic imu_extrinsic);
    void wheelGetter(vesc_msgs::msg::VescStateStamped current_vesc);
    void scanGetter(sensor_msgs::msg::LaserScan current_scan);
    void setImuExtrinsic(ImuExtrinsic imu_extrinsic);
    void setEkfParameters(EkfParameters params);
    // 현재 보유한 센서 중 가장 최신 시각의 오도메트리를 반환합니다.
    nav_msgs::msg::Odometry ekfOdom();
    // 센서값을 보간/짧은 외삽하여 요청 시각의 오도메트리를 반환합니다.
    nav_msgs::msg::Odometry ekfOdom(builtin_interfaces::msg::Time target_time);

    private:
    // EKF 상태 벡터:
    // [rear_x, rear_y, yaw, rear_velocity, gyro_z_bias,
    //  accel_x_bias, accel_y_bias, pitch, pitch_rate, roll, roll_rate]
    static constexpr int32_t kStateSize = 11;
    using StateVector = Eigen::Matrix<double, kStateSize, 1>;
    using StateMatrix = Eigen::Matrix<double, kStateSize, kStateSize>;

    enum StateIndex : int32_t {
        kX = 0,
        kY,
        kYaw,
        kVelocity,
        kGyroZBias,
        kAccelXBias,
        kAccelYBias,
        kPitch,
        kPitchRate,
        kRoll,
        kRollRate
    };

    struct TimedImu {
        sensor_msgs::msg::Imu msg;
        ImuExtrinsic extrinsic;
    };

    struct YawRateSample {
        double time;
        double yaw_rate;
    };

    // 시작 시 중력 보정은 1회만 수행합니다. 안정 샘플을 모아 저장하거나,
    // 제한 시간 이후에는 이전 저장값을 불러옵니다.
    enum class GravityCalibrationState {
        WaitingForStability,
        CalibratedFromStartupImu,
        LoadedFromFile,
        Unavailable
    };

    struct GravityStabilitySample {
        double time;
        Eigen::Vector3d accel;
        Eigen::Vector3d gyro;
    };

    struct WheelStabilitySample {
        double time;
        double cumulative_distance;
    };

    // 궤적 링버퍼 한 칸입니다. 공분산은 담지 않습니다(위치 보간 전용).
    struct TrajectorySample {
        double time{0.0};
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
    };

    // 목표시각 보간에 사용하는 누적 휠 이동거리 샘플입니다.
    struct TimedWheel {
        builtin_interfaces::msg::Time stamp;
        double time{0.0};
        double cumulative_distance{0.0};
    };

    // 시작 시 중력 검출 결과입니다. 중력 벡터는 rear/body 좌표계에 저장하고,
    // 이후 동적 pitch/roll에 의한 중력 성분과 합쳐 사용합니다.
    struct GravityCalibrationEstimate {
        bool valid{false};
        double score{0.0};
        Eigen::Vector3d gravity_body{Eigen::Vector3d::Zero()};
        double static_pitch{0.0};
        double static_roll{0.0};
        // 정지 window에서 측정한 자이로 z 평균입니다. 이 EKF에는 절대 heading
        // 관측이 없어 yaw-rate bias가 원리적으로 관측 불가능하므로, 여기서 한
        // 번 정하고 이후 상수로 얼립니다.
        double gyro_z_bias{0.0};
        int sample_count{0};
        double window_duration{0.0};
        double accel_norm_mean{0.0};
        double gyro_norm_mean{0.0};
        double accel_std{0.0};
        double gyro_std{0.0};
        double wheel_delta{0.0};
        double wheel_velocity_rms{0.0};
    };

    scan_entropy calcScanEntropy(sensor_msgs::msg::LaserScan scan);
    nav_msgs::msg::Odometry bycicleModel(); // 운동 모델
    // EKF 핵심 생명주기 처리입니다.
    void initializeEkf(double stamp);
    void predictWithImu(const TimedImu &imu_sample);
    void processWheelMeasurementsUpTo(double stamp, double wheel_variance);
    void applyScalarUpdate(double residual, const StateVector &jacobian, double variance);
    // 차량 동역학 모델과 수치 야코비안입니다.
    StateVector predictState(
        const StateVector &state,
        const TimedImu &imu_sample,
        double dt,
        double yaw_accel) const;
    StateMatrix numericalProcessJacobian(
        const StateVector &state,
        const TimedImu &imu_sample,
        double dt,
        double yaw_accel) const;
    StateVector numericalLateralJacobian(
        const StateVector &state,
        const TimedImu &imu_sample,
        double yaw_accel) const;
    double predictLateralAccelMeasurement(
        const StateVector &state,
        const TimedImu &imu_sample,
        double yaw_accel) const;
    // 시작 시 중력 보정 보조 함수들입니다.
    void updateStartupGravityCalibration(const TimedImu &imu_sample);
    void pushGravityStabilitySample(const TimedImu &imu_sample);
    bool estimateStartupGravityCalibration(GravityCalibrationEstimate *estimate) const;
    bool saveGravityCalibration(const GravityCalibrationEstimate &estimate) const;
    bool loadGravityCalibration(GravityCalibrationEstimate *estimate) const;
    void applyGravityCalibration(
        const GravityCalibrationEstimate &estimate,
        GravityCalibrationState state);
    Eigen::Vector3d calcGravityBody(const StateVector &state) const;
    // IMU AHRS quaternion에서 차대 기울기(시동 자세 대비 roll/pitch)를 뽑습니다.
    // 성공하면 true. 자세를 안 주는 IMU이거나 과대 기울기면 false.
    bool chassisTiltFromImu(
        const TimedImu &imu_sample, double &roll, double &pitch);
    std::string gravityCalibrationPath() const;
    // 저지연 센서 보간/외삽 보조 함수들입니다.
    bool interpolateImu(double target_time, TimedImu *imu_sample) const;
    bool interpolateWheelDistance(double target_time, double *cumulative_distance) const;
    void applyWheelMeasurementAtTime(
        double target_time,
        const builtin_interfaces::msg::Time &stamp,
        double cumulative_distance,
        double wheel_variance);
    void pruneSensorHistory(double newest_time);
    // IMU 샘플마다 도는 경량 dead-reckoning입니다. 행렬 연산 없이 yaw만 적분하고
    // EKF가 유지하는 bias/속도를 가져다 쓰므로, 궤적을 1kHz 해상도로 채우면서도
    // 무거운 EKF는 scan 레이트로만 돌 수 있습니다.
    void integrateDeadReckoning(const TimedImu &imu_sample);
    // 풀 EKF 갱신 직후 dead-reckoning 기준을 EKF 상태로 되맞춥니다.
    void resyncDeadReckoning();
    void appendTrajectory(double time, double x, double y, double yaw);
    // 신호 처리와 출력 메시지 생성 보조 함수들입니다.
    double updateYawRateDerivative(double stamp, double yaw_rate);
    nav_msgs::msg::Odometry buildOdomMsg(const builtin_interfaces::msg::Time &stamp) const;


    float motor_speed_gain;
    float wheel_base;
    ImuExtrinsic imu_extrinsic_;
    LaserExtrinsic laser_extrinsic_;
    EkfParameters ekf_params_;
    MotionNoise motion_noise_;
    // 직전 propagation에서 사용한 EKF pose입니다. 다음 호출과의 차이가 파티클에
    // 적용할 이동량이 됩니다.
    MotionDelta last_motion_delta_{};
    bool has_propagation_reference_{false};
    double propagation_reference_x_{0.0};
    double propagation_reference_y_{0.0};
    double propagation_reference_yaw_{0.0};
    // 맵 기반 관측성 고유분해입니다. 없으면 등방 노이즈로 동작합니다.
    bool noise_shaping_valid_{false};
    double noise_health_{1.0};
    Eigen::Matrix2d noise_shaping_vectors_{Eigen::Matrix2d::Identity()};
    Eigen::Vector2d noise_shaping_confidence_{Eigen::Vector2d::Zero()};
    std::mt19937 noise_rng_{20250721u};
    bool ekf_initialized_{false};
    bool has_previous_wheel_{false};
    builtin_interfaces::msg::Time last_imu_time_;
    builtin_interfaces::msg::Time previous_wheel_time_;
    double previous_wheel_distance_{0.0};
    // ERPM 경로 적분 상태: 직전 샘플 시각과 누적 주행거리[m].
    double erpm_prev_stamp_{-1.0};
    double erpm_cumulative_distance_{0.0};
    double last_yaw_rate_{0.0};
    double last_yaw_accel_{0.0};
    double last_rear_accel_x_{0.0};
    double last_rear_accel_y_{0.0};
    double last_slip_score_{0.0};
    double last_wheel_variance_{0.0};
    bool gravity_startup_timer_started_{false};
    double gravity_startup_time_{0.0};
    GravityCalibrationState gravity_calibration_state_{
        GravityCalibrationState::WaitingForStability};
    GravityCalibrationEstimate gravity_calibration_;
    // 경량 DR의 기울기 기준자세. AHRS는 절대 자세를 주므로 IMU 장착 기울기를
    // 빼기 위해 시동 시점을 기준으로 잡습니다(중력 보정과 같은 전제).
    bool dr_tilt_reference_valid_{false};
    double dr_tilt_reference_roll_{0.0};
    double dr_tilt_reference_pitch_{0.0};
    double last_dr_tilt_roll_{0.0};
    double last_dr_tilt_pitch_{0.0};
    bool last_dr_tilt_valid_{false};
    StateVector ekf_state_;
    StateMatrix ekf_covariance_;
    std::deque<YawRateSample> yaw_rate_history_;
    std::deque<GravityStabilitySample> gravity_imu_history_;
    std::deque<WheelStabilitySample> gravity_wheel_history_;
    std::deque<TimedImu> imu_history_;
    std::deque<TimedWheel> wheel_history_;
    // poseAt()이 읽는 궤적 링버퍼입니다. writer는 dead-reckoning 적분기뿐입니다.
    std::deque<TrajectorySample> trajectory_;
    bool dead_reckoning_ready_{false};
    double dead_reckoning_time_{0.0};
    double dead_reckoning_x_{0.0};
    double dead_reckoning_y_{0.0};
    double dead_reckoning_yaw_{0.0};

    std::queue<builtin_interfaces::msg::Time> wheel_time_queue;
    std::queue<double> wheel_queue;
    std::queue<TimedImu> imu_queue;
    std::queue<sensor_msgs::msg::LaserScan> scan_queue;


};
