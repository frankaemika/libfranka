#pragma once

#include <ostream>

/**
 * @file gripper_state.h
 * Contains the GripperState struct.
 */

namespace franka {

/**
 * Describes FRANKA's gripper state.
 */
struct GripperState {
  /**
   * Current opening width.
   */
  double width;

  /**
   * Maximum opening width.
   */
  double max_width;

  /**
   * Object grasped flag.
   * True: Gripper successfully grasped an object
   * False: Part lost or nothing grasped.
   */
  bool is_grasped;

  /**
   * Current gripper temperature.
   */
  uint16_t temperature;
};

std::ostream& operator<<(std::ostream& ostream, const franka::GripperState& gripper_state);

}  // namespace franka
