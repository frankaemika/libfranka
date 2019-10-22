// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>

#include <franka/gripper_state.h>
#include <franka/log.h>
#include <franka/robot_state.h>
#include <franka/vacuum_gripper_state.h>
#include <research_interface/gripper/types.h>
#include <research_interface/robot/rbk_types.h>
#include <research_interface/robot/service_types.h>

bool stringContains(const std::string& actual, const std::string& expected);

void randomRobotState(franka::RobotState& robot_state);
void randomRobotState(research_interface::robot::RobotState& robot_state);
void testRobotStateIsZero(const franka::RobotState& actual);
void testRobotStatesAreEqual(const research_interface::robot::RobotState& expected,
                             const franka::RobotState& actual);
void testRobotStatesAreEqual(const franka::RobotState& expected, const franka::RobotState& actual);

void randomRobotCommand(research_interface::robot::RobotCommand& command);
void testMotionGeneratorCommandsAreEqual(
    const research_interface::robot::MotionGeneratorCommand& expected,
    const research_interface::robot::MotionGeneratorCommand& actual);
void testControllerCommandsAreEqual(const research_interface::robot::ControllerCommand& expected,
                                    const research_interface::robot::ControllerCommand& actual);
void testRobotCommandsAreEqual(const research_interface::robot::RobotCommand& expected,
                               const research_interface::robot::RobotCommand& actual);
void testRobotCommandsAreEqual(const research_interface::robot::RobotCommand& expected,
                               const franka::RobotCommand actual);
std::array<double, 16> identityMatrix();
franka::RobotState generateValidRobotState();

namespace research_interface {
namespace robot {

bool operator==(const Move::Deviation& left, const Move::Deviation& right);

}  // namespace robot
}  // namespace research_interface

namespace franka {

bool operator==(const Errors& lhs, const Errors& rhs);

}  // namespace franka
