#pragma once

#include <array>

/**
 * @file command_types.h
 * Contains types for the commands that can be sent from franka::Robot.
 */

namespace franka {

/**
 * Controller mode for FRANKA.
 */
enum ControllerMode {
  kMotorPD,
  kJointPosition,
  kJointImpedance,
  kCartesianImpedance
};

/**
 * Helper type for holding Cartesian limits.
 */
struct CartesianLimits {
};

}  // namespace franka
