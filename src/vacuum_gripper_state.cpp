// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "franka/vacuum_gripper_state.h"

#include <cstring>

namespace franka {

std::ostream& operator<<(std::ostream& ostream,
                         const franka::VacuumGripperState& vacuum_gripper_state) {
  std::string device_status;
  switch (vacuum_gripper_state.device_status) {
    case VacuumGripperDeviceStatus::kGreen:
      device_status = "Green";
      break;
    case VacuumGripperDeviceStatus::kYellow:
      device_status = "Yellow";
      break;
    case VacuumGripperDeviceStatus::kOrange:
      device_status = "Orange";
      break;
    case VacuumGripperDeviceStatus::kRed:
      device_status = "Red";
      break;
  }

  ostream << "{\"in_control_range\": " << vacuum_gripper_state.in_control_range
          << ", \"part_detached\": " << vacuum_gripper_state.part_detached
          << ", \"part_present\": " << vacuum_gripper_state.part_present << ", \"device_status\": "
          << "\"" << device_status << "\""
          << ", \"actual_power\": " << vacuum_gripper_state.actual_power
          << ", \"vacuum\": " << vacuum_gripper_state.vacuum
          << ", \"time\": " << vacuum_gripper_state.time.toSec() << "}";
  return ostream;
}

}  // namespace franka
