// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <cstdint>

#include <franka/control_types.h>
#include <franka/robot_state.h>
#include <research_interface/robot/rbk_types.h>
#include <research_interface/robot/service_types.h>

namespace franka {

class RobotControl {
 public:
  virtual ~RobotControl() = default;

  virtual uint32_t startMotion(
      research_interface::robot::Move::ControllerMode controller_mode,
      research_interface::robot::Move::MotionGeneratorMode motion_generator_mode,
      const research_interface::robot::Move::Deviation& maximum_path_deviation,
      const research_interface::robot::Move::Deviation& maximum_goal_pose_deviation) = 0;
  virtual void finishMotion(
      uint32_t motion_id,
      const research_interface::robot::MotionGeneratorCommand* motion_command,
      const research_interface::robot::ControllerCommand* control_command) = 0;
  virtual void cancelMotion(uint32_t motion_id) = 0;

  virtual RobotState update(
      const research_interface::robot::MotionGeneratorCommand* motion_command,
      const research_interface::robot::ControllerCommand* control_command) = 0;

  virtual void throwOnMotionError(const RobotState& robot_state, uint32_t motion_id) = 0;

  virtual RealtimeConfig realtimeConfig() const noexcept = 0;
};

}  // namespace franka
