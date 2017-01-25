#include "franka/robot_state.h"

namespace franka {

std::ostream& operator<<(std::ostream& os, const franka::RobotState& rs) {
  os << boost::format(
            "{timestamp: %1%, q: [%2%], dq: [%3%], tau_J: [%4%], dtau_J: "
            "[%5%]}") %
            rs.timestamp % joinArray(rs.q) % joinArray(rs.dq) %
            joinArray(rs.tau_J) % joinArray(rs.dtau_J);
  return os;
}

} // namespace franka