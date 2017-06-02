#pragma once

#include "franka/robot_state.h"
#include "research_interface/rbk_types.h"
#include "research_interface/service_types.h"

void randomRobotState(franka::RobotState& robot_state);
void randomRobotState(research_interface::RobotState& robot_state);
void testRobotStateIsZero(const franka::RobotState& actual);
void testRobotStatesAreEqual(const research_interface::RobotState& expected,
                             const franka::RobotState& actual);
void testRobotStatesAreEqual(const franka::RobotState& expected, const franka::RobotState& actual);

void randomRobotCommand(research_interface::RobotCommand& command);
void testMotionGeneratorCommandsAreEqual(const research_interface::MotionGeneratorCommand& expected,
                                         const research_interface::MotionGeneratorCommand& actual);
void testControllerCommandsAreEqual(const research_interface::ControllerCommand& expected,
                                    const research_interface::ControllerCommand& actual);

namespace research_interface {

bool operator==(const Move::Deviation& left, const Move::Deviation& right);

}  // namespace research_interface
