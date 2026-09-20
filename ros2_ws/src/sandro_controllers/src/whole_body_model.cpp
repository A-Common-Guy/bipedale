#include "sandro_controllers/whole_body_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sandro_controllers {
namespace {

bool positive_geometry(const LegGeometry &g) {
  return std::isfinite(g.base) && std::isfinite(g.upper) &&
         std::isfinite(g.lower) && g.base > 0.0 && g.upper > 0.0 &&
         g.lower > 0.0;
}

bool unit_sign(const double value) {
  return std::abs(std::abs(value) - 1.0) < 1e-12;
}

} // namespace

bool valid_model(const WholeBodyModel &m) {
  const double values[] = {m.body_mass_kg,
                           m.wheel_mass_kg,
                           m.wheel_inertia_kgm2,
                           m.wheel_radius_m,
                           m.track_width_m,
                           m.com_height_per_leg_height,
                           m.com_height_offset_m,
                           m.pitch_inertia_kgm2,
                           m.pitch_inertia_reference_height_m,
                           m.pitch_inertia_per_m,
                           m.gravity_m_s2,
                           m.right_forward_sign,
                           m.left_forward_sign,
                           m.right_wheel_on_outer_link,
                           m.left_wheel_on_outer_link,
                           m.yaw_inertia_kgm2,
                           m.leg_lag_s};
  if (!std::all_of(std::begin(values), std::end(values),
                   [](const double v) { return std::isfinite(v); })) {
    return false;
  }
  return positive_geometry(m.right_geometry) &&
         positive_geometry(m.left_geometry) &&
         unit_sign(m.right_forward_sign) && unit_sign(m.left_forward_sign) &&
         (m.right_wheel_on_outer_link == 0.0 || m.right_wheel_on_outer_link == 1.0) &&
         (m.left_wheel_on_outer_link == 0.0 || m.left_wheel_on_outer_link == 1.0) &&
         m.yaw_inertia_kgm2 > 0.0 && m.leg_lag_s >= 0.002 &&
         m.body_mass_kg > 0.0 && m.wheel_mass_kg >= 0.0 &&
         m.wheel_inertia_kgm2 >= 0.0 && m.wheel_radius_m > 0.0 &&
         m.track_width_m > 0.0 && m.pitch_inertia_kgm2 > 0.0 &&
         m.pitch_inertia_reference_height_m > 0.0 && m.gravity_m_s2 > 0.0;
}

double com_height(const WholeBodyModel &m, const double leg_height_m) {
  return m.com_height_per_leg_height * leg_height_m + m.com_height_offset_m;
}

double pitch_inertia(const WholeBodyModel &m, const double leg_height_m) {
  return m.pitch_inertia_kgm2 +
         m.pitch_inertia_per_m *
             (leg_height_m - m.pitch_inertia_reference_height_m);
}

LinearModel linearize(const WholeBodyModel &m, const double leg_height_m) {
  // Lagrangian of a body (mass M, CoM at height L above the axle, forward
  // offset d, pitch inertia I about its CoM) on wheels (total mass m_w,
  // total axle inertia I_w, radius r) rolling without slip, with the total
  // motor torque tau acting between wheels and body:
  //   a x_dd + b pitch_dd = tau / r
  //   b x_dd + c pitch_dd = M g (L pitch + d) - tau
  // with a = M + m_w + I_w / r^2, b = M L, c = I + M L^2. The leg offset d
  // enters exactly like pitch scaled by 1/L, which is why moving the wheels
  // under the body is as effective as leaning it. The legs are not
  // instantaneous, though: the achieved d follows the commanded d_c with
  //   d_dot = (d_c - d) / leg_lag_s,   d_c_dot = u_2,
  // so the regulator knows what the legs can deliver and does not ask them
  // for modes faster than their own.
  const double M = m.body_mass_kg;
  const double L = com_height(m, leg_height_m);
  const double I = pitch_inertia(m, leg_height_m);
  const double r = m.wheel_radius_m;
  const double g = m.gravity_m_s2;
  const double a = M + 2.0 * m.wheel_mass_kg +
                   2.0 * m.wheel_inertia_kgm2 / (r * r);
  const double b = M * L;
  const double c = I + M * L * L;
  const double determinant = a * c - b * b;

  LinearModel result;
  auto &A = result.A;
  auto &B = result.B;
  A(0, 1) = 1.0;
  A(2, 3) = 1.0;
  A(1, 2) = -b * M * g * L / determinant;
  A(1, 5) = -b * M * g / determinant;
  A(3, 2) = a * M * g * L / determinant;
  A(3, 5) = a * M * g / determinant;
  A(5, 4) = 1.0 / m.leg_lag_s;
  A(5, 5) = -1.0 / m.leg_lag_s;
  B(1, 0) = (c / r + b) / determinant;
  B(3, 0) = -(b / r + a) / determinant;
  B(4, 1) = 1.0;
  return result;
}

Attitude attitude_from_quaternion(double w, double x, double y, double z) {
  Attitude result;
  const double norm = std::sqrt(w * w + x * x + y * y + z * z);
  if (!std::isfinite(norm) || norm < 1e-9) {
    result.pitch_rad = std::numeric_limits<double>::quiet_NaN();
    result.roll_rad = std::numeric_limits<double>::quiet_NaN();
    return result;
  }
  w /= norm;
  x /= norm;
  y /= norm;
  z /= norm;
  // World up expressed in the body frame is the third row of R(q):
  // (2(xz - wy), 2(yz + wx), 1 - 2(x^2 + y^2)). Both angles read that
  // vector only, so heading drops out.
  const double up_x = 2.0 * (x * z - w * y);
  const double up_y = 2.0 * (y * z + w * x);
  const double up_z = 1.0 - 2.0 * (x * x + y * y);
  result.pitch_rad = std::asin(std::clamp(-up_x, -1.0, 1.0));
  result.roll_rad = std::atan2(up_y, up_z);
  result.valid = std::isfinite(result.pitch_rad) && std::isfinite(result.roll_rad);
  return result;
}

LegState leg_state(const double theta_inner, const double theta_outer,
                   const double omega_inner, const double omega_outer,
                   const LegGeometry &geometry, const bool wheel_on_outer_link) {
  LegState result;
  const auto point = forward_kinematics(theta_inner, theta_outer, geometry);
  const auto j = jacobian(theta_inner, theta_outer, geometry);
  const auto link = lower_link_angle_jacobian(theta_inner, theta_outer,
                                              geometry, wheel_on_outer_link);
  if (!point || !j || !link) {
    return result;
  }
  result.position = *point;
  result.jacobian = *j;
  result.wheel_link_jacobian = *link;
  result.velocity.x = (*j)[0][0] * omega_inner + (*j)[0][1] * omega_outer;
  result.velocity.y = (*j)[1][0] * omega_inner + (*j)[1][1] * omega_outer;
  result.valid = std::isfinite(result.velocity.x) &&
                 std::isfinite(result.velocity.y) &&
                 std::isfinite(result.wheel_link_jacobian[0]) &&
                 std::isfinite(result.wheel_link_jacobian[1]);
  return result;
}

bool inside_leg_workspace(const double x, const double y,
                          const LegGeometry &geometry,
                          const double joint_limit_rad,
                          const double stretch_margin) {
  const auto angles = inverse_kinematics(x, y, geometry);
  if (!angles) {
    return false;
  }
  const double reach = stretch_margin * (geometry.upper + geometry.lower);
  const double r1 = std::hypot(x, y);
  const double r2 = std::hypot(x - geometry.base, y);
  const double fold = std::abs(geometry.upper - geometry.lower) / stretch_margin;
  return r1 <= reach && r2 <= reach && r1 >= fold && r2 >= fold &&
         std::abs((*angles)[0]) <= joint_limit_rad &&
         std::abs((*angles)[1]) <= joint_limit_rad &&
         forward_kinematics((*angles)[0], (*angles)[1], geometry).has_value();
}

} // namespace sandro_controllers
