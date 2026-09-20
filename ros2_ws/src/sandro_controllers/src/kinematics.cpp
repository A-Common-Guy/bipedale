#include "sandro_controllers/kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace sandro_controllers {
namespace {
constexpr double kPi = 3.14159265358979323846;

double wrap(double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

bool valid_geometry(const LegGeometry &g) {
  return std::isfinite(g.base) && std::isfinite(g.upper) &&
         std::isfinite(g.lower) && g.base > 0.0 && g.upper > 0.0 &&
         g.lower > 0.0;
}
} // namespace

std::optional<JointPair> inverse_kinematics(const double x, const double y,
                                            const LegGeometry &g) {
  if (!valid_geometry(g) || !std::isfinite(x) || !std::isfinite(y)) {
    return std::nullopt;
  }
  const double r1 = std::hypot(x, y);
  const double r2 = std::hypot(x - g.base, y);
  const double lo = std::max(std::abs(g.upper - g.lower), 1e-9);
  const double hi = g.upper + g.lower;
  if (r1 < lo || r1 > hi || r2 < lo || r2 > hi) {
    return std::nullopt;
  }
  const auto clamp_unit = [](const double value) {
    return std::clamp(value, -1.0, 1.0);
  };
  const double a1 =
      std::acos(clamp_unit((g.upper * g.upper + r1 * r1 - g.lower * g.lower) /
                           (2.0 * g.upper * r1)));
  const double a2 =
      std::acos(clamp_unit((g.upper * g.upper + r2 * r2 - g.lower * g.lower) /
                           (2.0 * g.upper * r2)));
  const double d1 = std::atan2(y, x) + a1;
  const double d2 = std::atan2(y, x - g.base) - a2;
  return JointPair{wrap(d1 - kPi / 2.0), wrap(kPi / 2.0 - d2)};
}

std::optional<Point2> forward_kinematics(const double theta_inner,
                                         const double theta_outer,
                                         const LegGeometry &g) {
  if (!valid_geometry(g) || !std::isfinite(theta_inner) ||
      !std::isfinite(theta_outer)) {
    return std::nullopt;
  }
  const double p = g.upper * (std::cos(theta_outer) - std::cos(theta_inner));
  const double q =
      g.base + g.upper * (std::sin(theta_inner) + std::sin(theta_outer));
  const double radius = std::hypot(p, q);
  if (radius > 2.0 * g.lower) {
    return std::nullopt;
  }
  const double direction = std::atan2(p, q);
  const double branch = std::acos(std::min(1.0, radius / (2.0 * g.lower)));
  const double b1 = branch + direction;
  return Point2{-g.upper * std::sin(theta_inner) + g.lower * std::cos(b1),
                g.upper * std::cos(theta_inner) + g.lower * std::sin(b1)};
}

std::optional<Jacobian2> jacobian(const double theta_inner,
                                  const double theta_outer,
                                  const LegGeometry &g, const double epsilon) {
  if (!std::isfinite(epsilon) || epsilon <= 0.0) {
    return std::nullopt;
  }
  const auto p1 = forward_kinematics(theta_inner + epsilon, theta_outer, g);
  const auto m1 = forward_kinematics(theta_inner - epsilon, theta_outer, g);
  const auto p2 = forward_kinematics(theta_inner, theta_outer + epsilon, g);
  const auto m2 = forward_kinematics(theta_inner, theta_outer - epsilon, g);
  if (!p1 || !m1 || !p2 || !m2) {
    return std::nullopt;
  }
  return Jacobian2{
      {{{(p1->x - m1->x) / (2.0 * epsilon), (p2->x - m2->x) / (2.0 * epsilon)}},
       {{(p1->y - m1->y) / (2.0 * epsilon),
         (p2->y - m2->y) / (2.0 * epsilon)}}}};
}

std::optional<LowerLinkAngles> lower_link_angles(const double theta_inner,
                                                 const double theta_outer,
                                                 const LegGeometry &g) {
  const auto axle = forward_kinematics(theta_inner, theta_outer, g);
  if (!axle) {
    return std::nullopt;
  }
  // Knees: the upper links hang from the inner pivot (0, 0) and the outer
  // pivot (base, 0), zero straight down (+y), opening away from the centre.
  const double knee_inner_x = -g.upper * std::sin(theta_inner);
  const double knee_inner_y = g.upper * std::cos(theta_inner);
  const double knee_outer_x = g.base + g.upper * std::sin(theta_outer);
  const double knee_outer_y = g.upper * std::cos(theta_outer);
  LowerLinkAngles result;
  result.inner = std::atan2(axle->y - knee_inner_y, axle->x - knee_inner_x);
  result.outer = std::atan2(axle->y - knee_outer_y, axle->x - knee_outer_x);
  if (!std::isfinite(result.inner) || !std::isfinite(result.outer)) {
    return std::nullopt;
  }
  return result;
}

std::optional<JointPair> lower_link_angle_jacobian(const double theta_inner,
                                                   const double theta_outer,
                                                   const LegGeometry &g,
                                                   const bool outer_link,
                                                   const double epsilon) {
  if (!std::isfinite(epsilon) || epsilon <= 0.0) {
    return std::nullopt;
  }
  const auto p1 = lower_link_angles(theta_inner + epsilon, theta_outer, g);
  const auto m1 = lower_link_angles(theta_inner - epsilon, theta_outer, g);
  const auto p2 = lower_link_angles(theta_inner, theta_outer + epsilon, g);
  const auto m2 = lower_link_angles(theta_inner, theta_outer - epsilon, g);
  if (!p1 || !m1 || !p2 || !m2) {
    return std::nullopt;
  }
  const auto pick = [outer_link](const LowerLinkAngles &a) {
    return outer_link ? a.outer : a.inner;
  };
  return JointPair{wrap(pick(*p1) - pick(*m1)) / (2.0 * epsilon),
                   wrap(pick(*p2) - pick(*m2)) / (2.0 * epsilon)};
}

} // namespace sandro_controllers
