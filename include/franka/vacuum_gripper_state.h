// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <cstdint>
#include <ostream>
#include <string>

#include <franka/duration.h>

/**
 * @file vacuum_gripper_state.h
 * Contains the franka::VacuumGripperState type.
 */

namespace franka {

/**
 * Describes the vacuum gripper state. For more information check the cobot-pump manual.
 */
struct VacuumGripperState {
  /**
   * Vacuum value wthin in setpoint area.
   */
  bool in_control_range{};

  /**
   * The part has been detached after a suction cycle
   */
  bool part_detached{};

  /**
   * Vacuum is over H2 and not yet under H2-h2. For more information check the cobot-pump manual.
   */
  bool part_present{};

  /**
   * Current vacuum gripper device status. Can be green, yellow, orange or red.
   */
  std::string device_status{};

  /**
   * Current vacuum gripper actual power. Unit: \f$[%]\f$.
   */
  uint16_t actual_power{};

  /**
   * Current vacuum. Unit: \f$[mbar]\f$.
   */
  uint16_t vacuum{};

  /**
   * Strictly monotonically increasing timestamp since robot start.
   */
  Duration time{};
};

/**
 * Streams the vacuum gripper state as JSON object:
 * {"field_name_1": value, "field_name_2": value, ...}
 *
 * @param[in] ostream Ostream instance
 * @param[in] vacuum_gripper_state VacuumGripperState struct instance to stream
 *
 * @return Ostream instance
 */
std::ostream& operator<<(std::ostream& ostream,
                         const franka::VacuumGripperState& vacuum_gripper_state);

}  // namespace franka
