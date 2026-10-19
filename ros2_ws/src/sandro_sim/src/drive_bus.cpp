#include "sandro_sim/drive_bus.hpp"

namespace sandro_sim {

DriveBus &drive_bus() {
  static DriveBus bus;
  return bus;
}

} // namespace sandro_sim
