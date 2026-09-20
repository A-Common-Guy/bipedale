#pragma once

#include <array>
#include <optional>

namespace sandro_controllers {

struct Point2 {
  double x{};
  double y{};
};

struct LegGeometry {
  double base{0.09};
  double upper{0.08};
  double lower{0.13};
};

using JointPair = std::array<double, 2>;
using Jacobian2 = std::array<std::array<double, 2>, 2>;

std::optional<JointPair> inverse_kinematics(double x, double y,
                                            const LegGeometry &geometry);
std::optional<Point2> forward_kinematics(double theta_inner, double theta_outer,
                                         const LegGeometry &geometry);
std::optional<Jacobian2> jacobian(double theta_inner, double theta_outer,
                                  const LegGeometry &geometry,
                                  double epsilon = 1e-6);

// Absolute planar angles of the two lower links (from the inner knee and from
// the outer knee to the axle), measured from +x toward +y. The wheel motor's
// stator is bolted to one of them, so its reaction torque acts on that link.
struct LowerLinkAngles {
  double inner{};
  double outer{};
};
std::optional<LowerLinkAngles> lower_link_angles(double theta_inner,
                                                 double theta_outer,
                                                 const LegGeometry &geometry);

// d(angle of one lower link) / d(theta_inner, theta_outer), by central
// differences. A moment M on that link is the generalised joint torque
// M * jacobian.
std::optional<JointPair> lower_link_angle_jacobian(double theta_inner,
                                                   double theta_outer,
                                                   const LegGeometry &geometry,
                                                   bool outer_link,
                                                   double epsilon = 1e-6);

} // namespace sandro_controllers
