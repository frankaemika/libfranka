#include "franka/robot_state.h"

#include <algorithm>
#include <iterator>

namespace franka {

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  std::copy(array.cbegin(), array.cend(),
            std::ostream_iterator<T>(ostream, ","));
  return ostream;
}

std::ostream& operator<<(std::ostream& ostream,
                         const franka::RobotState& robot_state) {
  ostream << "q_start: " << robot_state.q_start;
  // TODO: output all members
  return ostream;
}

}  // namespace franka
