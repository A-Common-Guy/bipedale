#pragma once

// Names of the whole-body controller's parameters, shared by the ROS
// controller (declaration, reading, live updates) and the closed-loop
// simulation harness (sandro_sim/tools/closed_loop), so a parameter file means
// the same thing to both.

#include <string>
#include <vector>

#include "sandro_controllers/whole_body_core.hpp"

namespace sandro_controllers {

struct WholeBodyDoubleParameter {
  const char *name;
  double &(*field)(WholeBodyParameters &);
  const char *description;
};

constexpr const char *kWholeBodyGainPointsName = "balance.gain_table.points";

// Every double parameter, in declaration order.
const std::vector<WholeBodyDoubleParameter> &whole_body_double_parameters();

// Sets the named parameter (a double, or the integer gain-table size);
// false if no parameter has that name.
bool set_whole_body_parameter(WholeBodyParameters &parameters,
                              const std::string &name, double value);

} // namespace sandro_controllers
