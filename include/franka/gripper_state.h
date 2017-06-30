#pragma once

#include <cstdint>
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
   * Current gripper opening width.
   */
  double opening_width{};

  /**
   * Maximum gripper opening width.
   * This parameter is estimated by homing the gripper.
   * After changing the gripper fingers, a homing needs to be done.
   *
   * @see Gripper::homing.
   */
  double max_opening_width{};

  /**
   * Indicated whether an object is currently grasped.
   */
  bool object_grasped{};

  /**
   * Current gripper temperature.
   */
  uint16_t temperature{};

  /**
   * Strictly increasing sequence number for each received gripper state.
   */
  uint32_t sequence_number{};
};

/**
 * Streams the GripperState according to the following format: {field_name_1:
 * value, field_name_2: value, ...}
 */
std::ostream& operator<<(std::ostream& ostream, const franka::GripperState& gripper_state);

}  // namespace franka
