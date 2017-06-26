#include "franka/gripper_state.h"

#include <cstring>

namespace franka {

std::ostream& operator<<(std::ostream& ostream, const franka::GripperState& gripper_state) {
  ostream << "{opening_width: " << gripper_state.opening_width
          << ", max_opening_width: " << gripper_state.max_opening_width
          << ", object_grasped: " << gripper_state.object_grasped
          << ", temperature: " << gripper_state.temperature << "}";
  return ostream;
}

}  // namespace franka
