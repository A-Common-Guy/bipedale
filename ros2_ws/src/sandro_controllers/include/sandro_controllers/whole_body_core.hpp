#pragma once

// Whole-body controller core for Sandro. It is independent of ROS messages
// and plugin lifecycle so that task, allocation and safety behaviour can be
// unit-tested against a planar plant.
//
// Structure (one realtime update):
//   estimate   attitude from the IMU, Cartesian leg states from the five-bar
//              kinematics (velocities low-passed), ground odometry and the
//              wheel-differential yaw rate from the wheel encoders
//   references speed/position, height, lean-into-turn roll, a lagged yaw
//              rate, and the slow posture trim that puts the axle under the
//              centre of mass
//   balance    one LQR over [x, x_dot, pitch, pitch_rate, leg_offset] whose
//              two inputs are total wheel torque and the rate at which the
//              legs shift the wheels fore-aft under the body; its gains are
//              scheduled on the measured leg height because the pendulum
//              length changes with it. The leg offset it feeds back is the
//              measured axle position by default (the physical lever, and
//              in simulation the deflection of the legs under wheel torque
//              is damping information); measured_offset_blend can move it
//              toward the commanded offset if the robot hunts through its
//              leg compliance.
//   tasks      Cartesian impedance per leg (height with gravity support,
//              fore-aft placement), damping on low-passed velocities, an
//              optional feed-forward of the wheel torque's reaction on the
//              leg, mapped to motor torques through J^T with the gravity
//              support protected from saturation; a model-based PI on the
//              yaw rate with scrub and inertia feed-forward producing a
//              differential wheel torque
//   safety     command/IMU timeouts, engage-when-upright, effort ramp, fall
//              latch, torque and workspace limits, rate limits
//
// Both backends must present the logical joint convention documented in
// sandro_description (zero-down five-bar angles, forward-positive wheels).

#include <array>
#include <cstdint>
#include <string>

#include "sandro_controllers/whole_body_model.hpp"

namespace sandro_controllers {

constexpr int kWholeBodyStates = 5;
constexpr int kWholeBodyInputs = 2;
constexpr int kMaxGainPoints = 32;
constexpr int kMaxDelayMarginSamples = 16;
constexpr double kLegJointLimitRad = 2.0943951024;

struct WholeBodyParameters {
  WholeBodyModel model{};

  // Balance LQR design, roughly Bryson's rule (1 / acceptable deviation^2 in
  // SI units). Position and velocity are weighted so that speed tracks
  // against the measured 0.4 Nm of drive friction with the legs free (the
  // robot's tracking failure came from legs pinned at their offset limit by
  // a wrong neutral, not from the gains); weighting them harder (60 / 8)
  // made the slow loop hunt against Coulomb friction in simulation. The
  // per-wheel velocity feedback stays well below wheel_inertia /
  // control_period (about 0.1 Nm per rad/s here): that feedback also closes
  // a loop on the wheel's own inertia through the tyre or gearbox
  // compliance, which chatters at the sampling rate when the gain is higher.
  // The leg offset rate is cheap so the legs carry a real share of the
  // balance work. The wheel torque weight sets the loop-delay margin of the
  // balance loop; the set is rejected below required_delay_margin_s.
  std::array<double, kWholeBodyStates> state_weights{30.0, 4.0, 150.0, 1.0,
                                                     1500.0};
  std::array<double, kWholeBodyInputs> effort_weights{2.0, 20.0};
  double gain_table_min_height_m{0.14};
  double gain_table_max_height_m{0.19};
  int gain_table_points{7};
  // Largest pure input delay the discrete closed loop must tolerate at every
  // table height (computed by augmenting the plant with delayed inputs).
  // The physical sensor-plus-command path is 10-15 ms.
  double required_delay_margin_s{0.012};
  // Which leg offset the LQR sees: 0 is the offset it commanded (the legs
  // treated as an inner loop, so deflections under wheel torque or stiction
  // never become balance action), 1 is the measured axle position, values
  // between blend the two. On the robot the measured axle follows its
  // command with about 70 ms of lag (overdamped impedance on the light
  // wheel plus stiction), the same time constant as the LQR's leg-offset
  // mode, and with 1 the legs hunted in phase at 1.9 Hz with +-25 mm and
  // +-1.5 Nm of wheel torque; 0 stopped it outright (fourth hardware run).
  // In the latency-emulating simulation, whose legs lag only 30 ms, 1 is
  // calmer (pitch RMS 0.015 against 0.036), but the robot decides.
  double measured_offset_blend{0.0};
  // Fore-aft wheel placement the legs may use for balance, about neutral.
  double leg_offset_limit_m{0.025};
  double leg_offset_rate_limit_m_s{0.25};
  // Anti-windup for the position state. While driving it is the integral
  // action of the speed loop: the reference runs ahead of a robot held back
  // by friction by at most this distance.
  double position_error_limit_m{0.3};
  // Coulomb friction of one drive train and tyre as seen at the wheel, added
  // in the direction of the torque the controller is already commanding
  // (tanh(command / smoothing)), so the command is not eaten by friction
  // before the wheel moves. Keyed on the command, not on wheel speed: a
  // speed-keyed term is negative damping around zero and destabilises the
  // wheel mode. Keep it below the measured friction.
  double wheel_friction_ff_nm{0.0};
  double wheel_friction_ff_smoothing_nm{0.1};
  // Slow posture trim. At rest the body balances when
  //   com_height * pitch + leg_offset + unmodelled_com_offset = 0,
  // so the measured lever com_height * pitch + leg_offset estimates the
  // unmodelled offset directly. It is low-pass filtered at trim_rate_per_s
  // only while the robot is at rest: drive command and measured speed inside
  // the velocity deadband and the balance torque inside the torque deadband.
  // The torque test matters on hardware: a body leaning against wheel
  // stiction (0.4 Nm on Sandro) is at rest and level-looking, but that lean
  // is friction, not mass, and learning it moved the neutral 30 mm off. The
  // trim goes into the leg offset reference so the body stands level instead
  // of leaning to compensate an unmodelled centre of mass or a slope. Beyond
  // trim_limit_m or the leg offset limit the remainder becomes a pitch
  // reference.
  double trim_rate_per_s{1.0};
  double trim_limit_m{0.03};
  double trim_velocity_deadband_m_s{0.05};
  double trim_torque_deadband_nm{0.15};
  double wheel_torque_sign{1.0};
  double wheel_torque_rate_nm_s{100.0};
  // First-order low-pass on the encoder-derived forward velocity and on the
  // wheel-differential yaw rate. The wheel velocity feedback acts on the
  // wheel's own inertia through whatever compliance sits between tyre and
  // ground (or gearbox backlash), a mode far above the pendulum's; filtering
  // keeps that loop gain low.
  double velocity_filter_hz{15.0};
  // Balancing engages once the body is upright and still after settle_s.
  double engage_pitch_rad{0.15};
  double engage_pitch_rate_rad_s{0.5};

  // Height task (planar leg height, pivot to axle). The legs are the balance
  // loop's inner actuator and should be as stiff as the loop delay allows:
  // at 300 N/m one newton-metre of wheel torque moved the axle 20 to 50 mm
  // through the ground force and the stator reaction, which is what the
  // robot's legs did at 2 Hz. With about 13 ms of sensor-plus-command delay
  // the stiffness term itself is negative damping (k sin(wT)/w), so the
  // damping must grow with the stiffness (kd / kp of 0.04 s, the ratio of
  // the PID cascade's joint gains that balance the robot) while kd / m_leg
  // times the delay stays small. The latency-emulating simulation falls
  // above about 400 N/m with either ratio, and it also falls with the
  // cascade's joint gains that the robot balances on, so the leg stiffness
  // is tuned on the robot (WHOLE_BODY_CONTROLLER.md); these are the values
  // the emulation accepts. The derivative gain acts on a low-passed
  // Cartesian velocity. The integral absorbs the part of the body weight the
  // feed-forward gets wrong (the lower links rest on the wheels directly).
  double height_target_m{0.17};
  double height_min_m{0.14};
  double height_max_m{0.19};
  double height_rate_m_s{0.05};
  double height_kp_n_m{400.0};
  double height_kd_n_s_m{20.0};
  double height_ki_n_m_s{150.0};
  double height_integral_limit_n{15.0};
  // Fore-aft task (planar x of the axle). neutral_m is the planar x with the
  // wheels centred under the pivots; neutral_offset_m shifts that neutral in
  // the body frame (positive forward, the same on both legs) so a robot
  // whose centre of mass is not where the CAD says keeps the full leg offset
  // range for balancing. The trim readout (trim_m, positive means the CoM is
  // behind the neutral) is minus this.
  double fore_aft_neutral_m{0.045};
  double fore_aft_neutral_offset_m{0.0};
  double fore_aft_kp_n_m{400.0};
  double fore_aft_kd_n_s_m{20.0};
  // Low-pass on the Cartesian leg velocities used by the damping terms.
  double leg_velocity_filter_hz{30.0};
  // Optional feed-forward of the wheel torque's reaction on the leg, off by
  // default. Rolling without slip, the wheel torque tau works on the leg
  // through the ground force tau / r at the axle and through the stator
  // moment -tau on the lower link that carries the wheel motor; the
  // compensation is J^T [-tau / r, 0] + tau d(link angle)/d(joints),
  // following the wheel torque through a low-pass. It is off because the
  // ground term assumes the torque reaches the ground: below the drive's
  // Coulomb friction (0.2 Nm per wheel) it does not, yet the term still
  // shifts the wheels back under the body, which tips the body the same way
  // the torque did, a positive-feedback path that fell in the
  // latency-emulating simulation with 400 N/m legs. The stator term alone
  // is harmless there but changes little. Try them on the robot only with
  // stiff legs and after the wheel friction is identified.
  double leg_reaction_ground_gain{0.0};
  double leg_reaction_stator_gain{0.0};
  double leg_reaction_filter_hz{4.0};
  // Roll task: lean into turns by a differential leg height, and level the
  // body on uneven ground.
  double roll_lean_gain{1.0};
  double roll_kp_m_rad{0.1};
  double roll_ki_m_rad_s{0.2};
  double roll_differential_limit_m{0.04};
  // Roll the IMU reports when the body is level (mounting misalignment);
  // subtracted before the roll task, otherwise the integral levels the IMU
  // and tilts the body.
  double roll_imu_offset_rad{0.0};
  // Yaw task. The plant is I_z psi_dd = (track / 2r) dtau - scrub, so the
  // proportional gain follows from the wanted bandwidth: kp = I_z w / (track
  // / 2r), about 0.3 Nm per rad/s for 7 rad/s here. The old 1.0 to 1.5 closed
  // a 30-plus rad/s loop through the IMU, the tyre scrub and the soft legs and
  // rang at 3.6 Hz even while the robot stood still. The proportional term
  // acts on the wheel-differential rate (synchronous with the loop, no leg
  // twist in it); the integral acts on the IMU rate so the body really turns
  // at the commanded rate despite tyre scrub and drive friction. Coulomb-like
  // scrub is fed forward with the sign of the commanded rate, and the yaw
  // inertia with the commanded acceleration.
  double yaw_kp_nm_s_rad{0.3};
  double yaw_ki_nm_rad{1.0};
  double yaw_wheel_rate_blend{1.0};
  double yaw_scrub_ff_nm{0.3};
  double yaw_scrub_ff_smoothing_rad_s{0.2};
  double yaw_torque_limit_nm{1.5};
  double yaw_torque_rate_nm_s{20.0};
  double yaw_torque_sign{1.0};
  // Joint damping applied to a leg whose kinematics cannot be evaluated.
  double leg_fallback_damping_nm_s_rad{0.2};

  // Policy limits.
  double leg_torque_limit_nm{3.0};
  double wheel_torque_limit_nm{6.0};

  // Drive command limits. The yaw rate command follows the operator through
  // a first-order lag so the lean-into-turn and inertia feed-forwards are
  // smooth.
  double max_forward_velocity_m_s{0.5};
  double max_yaw_rate_rad_s{1.5};
  double forward_accel_limit_m_s2{1.0};
  double yaw_command_time_constant_s{0.15};
  double max_height_rate_m_s{0.05};

  // Safety.
  double command_timeout_s{0.3};
  double imu_timeout_s{0.15};
  double settle_s{0.0};
  double ramp_s{0.5};
  double fall_angle_rad{1.0};
};

struct WholeBodyState {
  // right inner, right outer, right wheel, left inner, left outer, left wheel
  std::array<double, 6> position{};
  std::array<double, 6> velocity{};
  double quaternion_w{1.0};
  double quaternion_x{};
  double quaternion_y{};
  double quaternion_z{};
  // Body-frame rates: roll (x), pitch (y), yaw (z).
  std::array<double, 3> angular_velocity_rad_s{};
  double imu_age_s{};
};

struct WholeBodyCommand {
  double forward_velocity_m_s{};
  double yaw_rate_rad_s{};
  // Body height rate; positive extends the legs.
  double height_rate_m_s{};
  double age_s{};
};

enum WholeBodyStatus : std::uint32_t {
  WBC_OK = 0,
  WBC_COMMAND_STALE = 1U << 0,
  WBC_IMU_STALE = 1U << 1,
  WBC_FALL_LATCH = 1U << 2,
  WBC_SETTLING = 1U << 3,
  WBC_LEG_KINEMATICS = 1U << 4,
  WBC_LEG_TORQUE_LIMIT = 1U << 5,
  WBC_WHEEL_TORQUE_LIMIT = 1U << 6,
  WBC_LEG_OFFSET_LIMIT = 1U << 7,
  WBC_POSITION_ERROR_LIMIT = 1U << 8,
  WBC_HEIGHT_LIMIT = 1U << 9,
  WBC_ROLL_LIMIT = 1U << 10,
  WBC_YAW_TORQUE_LIMIT = 1U << 11,
  WBC_INVALID_SETTINGS = 1U << 12
};

std::string whole_body_status_text(std::uint32_t status);

using WholeBodyGains =
    std::array<std::array<double, kWholeBodyStates>, kWholeBodyInputs>;

struct GainPoint {
  double leg_height_m{};
  WholeBodyGains gains{};
};

// Gains scheduled on leg height, linearly interpolated and clamped.
struct GainSchedule {
  std::array<GainPoint, kMaxGainPoints> points{};
  int count{};
  WholeBodyGains at(double leg_height_m) const;
};

// Parameters plus everything derived from them off the realtime path.
struct WholeBodySettings {
  WholeBodyParameters parameters{};
  GainSchedule gains{};
  double control_period_s{1.0 / 300.0};
  // Smallest input-delay margin over the gain table, in seconds.
  double delay_margin_s{};
  bool valid{};
};

bool valid_whole_body_parameters(const WholeBodyParameters &parameters,
                                 std::string *reason = nullptr);

// Validates, solves the Riccati equation at every table height and checks
// the delay margin of each solution.
bool build_whole_body_settings(const WholeBodyParameters &parameters,
                               double control_period_s,
                               WholeBodySettings &settings,
                               std::string *reason = nullptr);

struct WholeBodyOutput {
  std::array<double, 6> effort_nm{};
  // Estimate.
  double pitch_rad{};
  double roll_rad{};
  double pitch_rate_rad_s{};
  double yaw_rate_rad_s{};
  double wheel_yaw_rate_rad_s{};
  double position_m{};
  double forward_velocity_m_s{};
  double leg_height_m{};
  double leg_offset_m{};
  std::array<Point2, 2> leg_position_m{};
  std::array<bool, 2> leg_valid{};
  // References.
  double position_reference_m{};
  double velocity_reference_m_s{};
  double pitch_reference_rad{};
  double leg_offset_reference_m{};
  double leg_offset_command_m{};
  // The leg offset the LQR fed back this cycle (see measured_offset_blend).
  double leg_offset_state_m{};
  double height_reference_m{};
  double height_differential_m{};
  double height_support_n{};
  double roll_reference_rad{};
  double yaw_rate_reference_rad_s{};
  double trim_m{};
  std::array<Point2, 2> leg_reference_m{};
  // Efforts before allocation.
  double balance_torque_nm{};
  double leg_offset_rate_m_s{};
  double yaw_torque_nm{};
  double yaw_feedforward_nm{};
  // Per leg [inner, outer]: joint torque fed forward for the wheel torque's
  // reaction on the leg.
  std::array<std::array<double, 2>, 2> leg_reaction_torque_nm{};
  WholeBodyGains gains{};
  double ramp{};
  std::uint32_t status{WBC_OK};
  bool engaged{};
  bool fallen{};
};

class WholeBodyCore {
public:
  void reset(const WholeBodyState &state, const WholeBodySettings &settings);
  WholeBodyOutput update(const WholeBodyState &state,
                         const WholeBodyCommand &command,
                         const WholeBodySettings &settings, double period_s);

private:
  double elapsed_s_{};
  double ramp_elapsed_s_{};
  bool engaged_{};
  bool fallen_{};
  double velocity_command_m_s_{};
  double yaw_rate_command_rad_s_{};
  bool driving_{};
  double position_reference_m_{};
  double height_offset_command_m_{};
  double height_reference_m_{0.17};
  double height_integral_n_{};
  double height_differential_m_{};
  double roll_integral_m_{};
  double leg_offset_command_m_{};
  double trim_m_{};
  double balance_torque_nm_{};
  double yaw_torque_nm_{};
  double yaw_integral_nm_{};
  double filtered_velocity_m_s_{};
  double filtered_wheel_yaw_rate_rad_s_{};
  bool have_filtered_velocity_{};
  std::array<Point2, 2> filtered_leg_velocity_m_s_{};
  std::array<bool, 2> have_filtered_leg_velocity_{};
  std::array<double, 2> filtered_wheel_torque_nm_{};
};

} // namespace sandro_controllers
