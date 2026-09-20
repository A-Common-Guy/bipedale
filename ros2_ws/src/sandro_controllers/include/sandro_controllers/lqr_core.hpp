#pragma once

#include <array>

#include "sandro_controllers/control_core.hpp"

namespace sandro_controllers {

// Full-state feedback balancer, as an alternative to the cascade in
// DrivingBalancerCore. The cascade splits balancing and driving into an inner
// pitch loop and an outer velocity loop, which needs the inner loop to be much
// faster than the outer one. This plant does not allow that: the pitch loop
// must run above roughly 2x the unstable pole and the velocity loop must stay
// below roughly half of it, leaving about 4x of separation where a cascade
// wants 5-10x. The loops therefore interact, and every outer gain change moves
// the inner loop's stability margin.
//
// Feeding all four states to one coordinated gain vector removes the
// assumption entirely. Wheel velocity becomes a first-class gain acting on
// (velocity - command) instead of the ad-hoc wheel damping that had to serve
// as both the stabilising feedback and the drag that fought driving.
//
// Gains come from tools/lqr_design.py, which reduces the MJCF to a planar
// wheeled inverted pendulum and solves the Riccati equation for them.
struct LqrParameters {
  // Total wheel torque per unit of state error; the core halves it per wheel.
  // Positive on position is not a typo: moving backward requires first driving
  // the wheels forward to tip the body that way.
  double position_nm_m{0.8165};
  double velocity_nm_s_m{1.7642};
  double pitch_nm_rad{8.0343};
  double pitch_rate_nm_s_rad{1.0413};
  // Optional trim for what the model does not capture, mostly a centre of mass
  // that is not exactly where the MJCF says and rolling resistance.
  double velocity_integral_nm_s_m{0.8};
  double velocity_integral_limit_nm{1.5};
  // Engage only once the chassis is within this much of upright. A balancer
  // that drives the moment it is activated will spin the wheels out from under
  // a robot that is still lying down or still being lifted, which pushes it
  // further over rather than catching it.
  double engage_angle_rad{0.15};
};

bool valid_lqr_parameters(const LqrParameters &gains);

class LqrBalancerCore {
public:
  void reset(const RobotState &state, const ControlParameters &parameters);
  ControlOutput update(const RobotState &state, const MotionCommand &command,
                       const ControlParameters &parameters,
                       const LqrParameters &gains, double period_s);

private:
  double leg_height_m_{0.17};
  std::array<double, 4> leg_setpoint_{};
  double elapsed_s_{};
  double balance_torque_nm_{};
  double velocity_command_m_s_{};
  double position_reference_m_{};
  double velocity_integral_nm_{};
  double yaw_torque_nm_{};
  bool engaged_{};
  bool fallen_{};
};

} // namespace sandro_controllers
