#pragma once

// The torque law a GIM6010 drive computes in MIT mode from one command
// frame, applied here to emulate the drive inside the simulation. Header-only
// and free of ROS so it can be unit-tested.
//
//   torque = kp (q_ref - q) + kd (v_ref - v) + effort, clipped to the limit
//
// Fields a controller leaves unset arrive as NaN through the command
// interfaces: gains and the velocity reference then count as zero and the
// position reference as the measured position, so a controller that only
// claims the effort interface drives a pure torque source.

#include <algorithm>
#include <cmath>
#include <limits>

namespace sandro_sim {

struct MitCommand {
  double position{};
  double velocity{};
  double kp{};
  double kd{};
  double effort{};
};

inline double mit_torque(const MitCommand &command, const double position,
                         const double velocity, const double torque_limit) {
  const auto gain = [](const double value) {
    return std::isfinite(value) && value > 0.0 ? value : 0.0;
  };
  const double kp = gain(command.kp);
  const double kd = gain(command.kd);
  const double effort = std::isfinite(command.effort) ? command.effort : 0.0;
  const double velocity_reference =
      std::isfinite(command.velocity) ? command.velocity : 0.0;
  const double position_reference =
      kp > 0.0 && std::isfinite(command.position) ? command.position : position;
  const double torque = kp * (position_reference - position) +
                        kd * (velocity_reference - velocity) + effort;
  const double limit = std::isfinite(torque_limit) && torque_limit > 0.0
                           ? torque_limit
                           : std::numeric_limits<double>::infinity();
  return std::clamp(torque, -limit, limit);
}

} // namespace sandro_sim
