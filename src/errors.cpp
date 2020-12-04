// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/errors.h>

#include <algorithm>
#include <iterator>
#include <sstream>

#include <research_interface/robot/error.h>

using Error = research_interface::robot::Error;

namespace franka {

Errors::Errors(const std::array<bool, 37>& errors)  // NOLINT(modernize-pass-by-value)
    : errors_(errors) {}

Errors::operator bool() const noexcept {
  return std::any_of(errors_.cbegin(), errors_.cend(), [](bool x) { return x; });
}

Errors::operator std::string() const {
  std::string error_string = "[";

  for (size_t i = 0; i < errors_.size(); i++) {
    if (errors_[i]) {
      error_string += "\"";
      error_string += getErrorName(static_cast<Error>(i));
      error_string += "\", ";
    }
  }

  if (error_string.size() > 1) {
    error_string.erase(error_string.end() - 2, error_string.end());
  }

  error_string += "]";

  return error_string;
}

const bool& Errors::joint_position_limits_violation() const {
  return errors_[static_cast<size_t>(Error::kJointPositionLimitsViolation)];
}

const bool& Errors::cartesian_position_limits_violation() const {
  return errors_[static_cast<size_t>(Error::kCartesianPositionLimitsViolation)];
}

const bool& Errors::self_collision_avoidance_violation() const {
  return errors_[static_cast<size_t>(Error::kSelfcollisionAvoidanceViolation)];
}

const bool& Errors::joint_velocity_violation() const {
  return errors_[static_cast<size_t>(Error::kJointVelocityViolation)];
}

const bool& Errors::cartesian_velocity_violation() const {
  return errors_[static_cast<size_t>(Error::kCartesianVelocityViolation)];
}

const bool& Errors::force_control_safety_violation() const {
  return errors_[static_cast<size_t>(Error::kForceControlSafetyViolation)];
}

const bool& Errors::joint_reflex() const {
  return errors_[static_cast<size_t>(Error::kJointReflex)];
}

const bool& Errors::cartesian_reflex() const {
  return errors_[static_cast<size_t>(Error::kCartesianReflex)];
}

const bool& Errors::max_goal_pose_deviation_violation() const {
  return errors_[static_cast<size_t>(Error::kMaxGoalPoseDeviationViolation)];
}

const bool& Errors::max_path_pose_deviation_violation() const {
  return errors_[static_cast<size_t>(Error::kMaxPathPoseDeviationViolation)];
}

const bool& Errors::cartesian_velocity_profile_safety_violation() const {
  return errors_[static_cast<size_t>(Error::kCartesianVelocityProfileSafetyViolation)];
}

const bool& Errors::joint_position_motion_generator_start_pose_invalid() const {
  return errors_[static_cast<size_t>(Error::kJointPositionMotionGeneratorStartPoseInvalid)];
}

const bool& Errors::joint_motion_generator_position_limits_violation() const {
  return errors_[static_cast<size_t>(Error::kJointMotionGeneratorPositionLimitsViolation)];
}

const bool& Errors::joint_motion_generator_velocity_limits_violation() const {
  return errors_[static_cast<size_t>(Error::kJointMotionGeneratorVelocityLimitsViolation)];
}

const bool& Errors::joint_motion_generator_velocity_discontinuity() const {
  return errors_[static_cast<size_t>(Error::kJointMotionGeneratorVelocityDiscontinuity)];
}

const bool& Errors::joint_motion_generator_acceleration_discontinuity() const {
  return errors_[static_cast<size_t>(Error::kJointMotionGeneratorAccelerationDiscontinuity)];
}

const bool& Errors::cartesian_position_motion_generator_start_pose_invalid() const {
  return errors_[static_cast<size_t>(Error::kCartesianPositionMotionGeneratorStartPoseInvalid)];
}

const bool& Errors::cartesian_motion_generator_elbow_limit_violation() const {
  return errors_[static_cast<size_t>(Error::kCartesianMotionGeneratorElbowLimitViolation)];
}

const bool& Errors::cartesian_motion_generator_velocity_limits_violation() const {
  return errors_[static_cast<size_t>(Error::kCartesianMotionGeneratorVelocityLimitsViolation)];
}

const bool& Errors::cartesian_motion_generator_velocity_discontinuity() const {
  return errors_[static_cast<size_t>(Error::kCartesianMotionGeneratorVelocityDiscontinuity)];
}

const bool& Errors::cartesian_motion_generator_acceleration_discontinuity() const {
  return errors_[static_cast<size_t>(Error::kCartesianMotionGeneratorAccelerationDiscontinuity)];
}

const bool& Errors::cartesian_motion_generator_elbow_sign_inconsistent() const {
  return errors_[static_cast<size_t>(Error::kCartesianMotionGeneratorElbowSignInconsistent)];
}

const bool& Errors::cartesian_motion_generator_start_elbow_invalid() const {
  return errors_[static_cast<size_t>(Error::kCartesianMotionGeneratorStartElbowInvalid)];
}

const bool& Errors::cartesian_motion_generator_joint_position_limits_violation() const {
  return errors_[static_cast<size_t>(Error::kCartesianMotionGeneratorJointPositionLimitsViolation)];
}

const bool& Errors::cartesian_motion_generator_joint_velocity_limits_violation() const {
  return errors_[static_cast<size_t>(Error::kCartesianMotionGeneratorJointVelocityLimitsViolation)];
}

const bool& Errors::cartesian_motion_generator_joint_velocity_discontinuity() const {
  return errors_[static_cast<size_t>(Error::kCartesianMotionGeneratorJointVelocityDiscontinuity)];
}

const bool& Errors::cartesian_motion_generator_joint_acceleration_discontinuity() const {
  return errors_[static_cast<size_t>(
      Error::kCartesianMotionGeneratorJointAccelerationDiscontinuity)];
}

const bool& Errors::cartesian_position_motion_generator_invalid_frame() const {
  return errors_[static_cast<size_t>(Error::kCartesianPositionMotionGeneratorInvalidFrame)];
}

const bool& Errors::force_controller_desired_force_tolerance_violation() const {
  return errors_[static_cast<size_t>(Error::kForceControllerDesiredForceToleranceViolation)];
}

const bool& Errors::controller_torque_discontinuity() const {
  return errors_[static_cast<size_t>(Error::kControllerTorqueDiscontinuity)];
}

const bool& Errors::start_elbow_sign_inconsistent() const {
  return errors_[static_cast<size_t>(Error::kStartElbowSignInconsistent)];
}

const bool& Errors::communication_constraints_violation() const {
  return errors_[static_cast<size_t>(Error::kCommunicationConstraintsViolation)];
}

const bool& Errors::power_limit_violation() const {
  return errors_[static_cast<size_t>(Error::kPowerLimitViolation)];
}

const bool& Errors::joint_p2p_insufficient_torque_for_planning() const {
  return errors_[static_cast<size_t>(Error::kJointP2PInsufficientTorqueForPlanning)];
}

const bool& Errors::tau_j_range_violation() const {
  return errors_[static_cast<size_t>(Error::kTauJRangeViolation)];
}

const bool& Errors::instability_detected() const {
  return errors_[static_cast<size_t>(Error::kInstabilityDetection)];
}

const bool& Errors::joint_move_in_wrong_direction() const {
  return errors_[static_cast<size_t>(Error::kJointMoveInWrongDirection)];
}

std::ostream& operator<<(std::ostream& ostream, const Errors& errors) {
  ostream << static_cast<std::string>(errors);
  return ostream;
}

}  // namespace franka
