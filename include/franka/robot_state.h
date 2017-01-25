#pragma once

#include <boost/format.hpp>
#include <cstdint>
#include <iostream>
#include <sstream>

namespace franka {

struct RobotState {
  std::uint32_t timestamp;
  double q[7];
  double dq[7];
  double tau_J[7];
  double dtau_J[7];

  friend std::ostream& operator<<(std::ostream& os, const RobotState& rs);
};

template <typename T, size_t N>
std::string joinArray(const T (&array)[N]) {
  std::ostringstream oss;

  oss << array[0];
  for (unsigned int i = 1; i < N; i++) {
    oss << ", " << array[i];
  }
  return oss.str();
}

std::ostream& operator<<(std::ostream& os, const franka::RobotState& rs);

}  // namespace franka