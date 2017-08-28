#include <franka/errors.h>

#include <algorithm>
#include <iterator>
#include <sstream>

#include <research_interface/robot/error.h>

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)
using Error = research_interface::robot::Error;

namespace franka {

Errors::Errors(std::array<bool, 33> errors)
    : joint_position_limits_violation(
          errors[static_cast<size_t>(Error::kJointPositionLimitsViolation)]),
      cartesian_position_limits_violation(
          errors[static_cast<size_t>(Error::kCartesianPositionLimitsViolation)]),
      self_collision_avoidance_violation(
          errors[static_cast<size_t>(Error::kSelfcollisionAvoidanceViolation)]),
      joint_velocity_violation(errors[static_cast<size_t>(Error::kJointVelocityViolation)]),
      cartesian_velocity_violation(errors[static_cast<size_t>(Error::kCartesianVelocityViolation)]),
      force_control_safety_violation(
          errors[static_cast<size_t>(Error::kForceControlSafetyViolation)]),
      joint_reflex(errors[static_cast<size_t>(Error::kJointReflex)]),
      cartesian_reflex(errors[static_cast<size_t>(Error::kCartesianReflex)]),
      max_goal_pose_deviation_violation(
          errors[static_cast<size_t>(Error::kMaxGoalPoseDeviationViolation)]),
      max_path_pose_deviation_violation(
          errors[static_cast<size_t>(Error::kMaxPathPoseDeviationViolation)]),
      cartesian_velocity_profile_safety_violation(
          errors[static_cast<size_t>(Error::kCartesianVelocityProfileSafetyViolation)]),
      joint_position_motion_generator_start_pose_invalid(
          errors[static_cast<size_t>(Error::kJointPositionMotionGeneratorStartPoseInvalid)]),
      joint_motion_generator_position_limits_violation(
          errors[static_cast<size_t>(Error::kJointMotionGeneratorPositionLimitsViolation)]),
      joint_motion_generator_velocity_limits_violation(
          errors[static_cast<size_t>(Error::kJointMotionGeneratorVelocityLimitsViolation)]),
      joint_motion_generator_velocity_discontinuity(
          errors[static_cast<size_t>(Error::kJointMotionGeneratorVelocityDiscontinuity)]),
      joint_motion_generator_acceleration_discontinuity(
          errors[static_cast<size_t>(Error::kJointMotionGeneratorAccelerationDiscontinuity)]),
      cartesian_position_motion_generator_start_pose_invalid(
          errors[static_cast<size_t>(Error::kCartesianPositionMotionGeneratorStartPoseInvalid)]),
      cartesian_motion_generator_elbow_limit_violation(
          errors[static_cast<size_t>(Error::kCartesianMotionGeneratorElbowLimitViolation)]),
      cartesian_motion_generator_velocity_limits_violation(
          errors[static_cast<size_t>(Error::kCartesianMotionGeneratorVelocityLimitsViolation)]),
      cartesian_motion_generator_velocity_discontinuity(
          errors[static_cast<size_t>(Error::kCartesianMotionGeneratorVelocityDiscontinuity)]),
      cartesian_motion_generator_acceleration_discontinuity(
          errors[static_cast<size_t>(Error::kCartesianMotionGeneratorAccelerationDiscontinuity)]),
      cartesian_motion_generator_elbow_sign_inconsistent(
          errors[static_cast<size_t>(Error::kCartesianMotionGeneratorElbowSignInconsistent)]),
      cartesian_motion_generator_start_elbow_invalid(
          errors[static_cast<size_t>(Error::kCartesianMotionGeneratorStartElbowInvalid)]),
      cartesian_motion_generator_joint_position_limits_violation(errors[static_cast<size_t>(
          Error::kCartesianMotionGeneratorJointPositionLimitsViolation)]),
      cartesian_motion_generator_joint_velocity_limits_violation(errors[static_cast<size_t>(
          Error::kCartesianMotionGeneratorJointVelocityLimitsViolation)]),
      cartesian_motion_generator_joint_velocity_discontinuity(
          errors[static_cast<size_t>(Error::kCartesianMotionGeneratorJointVelocityDiscontinuity)]),
      cartesian_motion_generator_joint_acceleration_discontinuity(errors[static_cast<size_t>(
          Error::kCartesianMotionGeneratorJointAccelerationDiscontinuity)]),
      cartesian_position_motion_generator_invalid_frame(
          errors[static_cast<size_t>(Error::kCartesianPositionMotionGeneratorInvalidFrame)]),
      force_controller_desired_force_tolerance_violation(
          errors[static_cast<size_t>(Error::kForceControllerDesiredForceToleranceViolation)]),
      controller_torque_discontinuity(
          errors[static_cast<size_t>(Error::kControllerTorqueDiscontinuity)]),
      start_elbow_sign_inconsistent(
          errors[static_cast<size_t>(Error::kStartElbowSignInconsistent)]),
      communication_constraints_violation(
          errors[static_cast<size_t>(Error::kCommunicationConstraintsViolation)]),
      power_limit_violation(errors[static_cast<size_t>(Error::kPowerLimitViolation)]) {}

Errors::operator bool() const noexcept {
  return joint_position_limits_violation || cartesian_position_limits_violation ||
         self_collision_avoidance_violation || joint_velocity_violation ||
         cartesian_velocity_violation || force_control_safety_violation || joint_reflex ||
         cartesian_reflex || max_goal_pose_deviation_violation ||
         max_path_pose_deviation_violation || cartesian_velocity_profile_safety_violation ||
         joint_position_motion_generator_start_pose_invalid ||
         joint_motion_generator_position_limits_violation ||
         joint_motion_generator_velocity_limits_violation ||
         joint_motion_generator_velocity_discontinuity ||
         joint_motion_generator_acceleration_discontinuity ||
         cartesian_position_motion_generator_start_pose_invalid ||
         cartesian_motion_generator_elbow_limit_violation ||
         cartesian_motion_generator_velocity_limits_violation ||
         cartesian_motion_generator_velocity_discontinuity ||
         cartesian_motion_generator_acceleration_discontinuity ||
         cartesian_motion_generator_elbow_sign_inconsistent ||
         cartesian_motion_generator_start_elbow_invalid ||
         force_controller_desired_force_tolerance_violation || start_elbow_sign_inconsistent ||
         communication_constraints_violation || power_limit_violation ||
         cartesian_motion_generator_joint_position_limits_violation ||
         cartesian_motion_generator_joint_velocity_limits_violation ||
         cartesian_motion_generator_joint_velocity_discontinuity ||
         cartesian_motion_generator_joint_acceleration_discontinuity ||
         cartesian_position_motion_generator_invalid_frame || controller_torque_discontinuity;
}

Errors::operator std::string() const {
  std::string error_string = "[";

  error_string += joint_position_limits_violation ? "\"joint_position_limits_violation\", " : "";
  error_string +=
      cartesian_position_limits_violation ? "\"cartesian_position_limits_violation\", " : "";
  error_string +=
      self_collision_avoidance_violation ? "\"self_collision_avoidance_violation\", " : "";
  error_string += joint_velocity_violation ? "\"joint_velocity_violation\", " : "";
  error_string += cartesian_velocity_violation ? "\"cartesian_velocity_violation\", " : "";
  error_string += force_control_safety_violation ? "\"force_control_safety_violation\", " : "";
  error_string += joint_reflex ? "\"joint_reflex\", " : "";
  error_string += cartesian_reflex ? "\"cartesian_reflex\", " : "";
  error_string +=
      max_goal_pose_deviation_violation ? "\"max_goal_pose_deviation_violation\", " : "";
  error_string +=
      max_path_pose_deviation_violation ? "\"max_path_pose_deviation_violation\", " : "";
  error_string += cartesian_velocity_profile_safety_violation
                      ? "\"cartesian_velocity_profile_safety_violation\", "
                      : "";
  error_string += joint_position_motion_generator_start_pose_invalid
                      ? "\"joint_position_motion_generator_start_pose_invalid\", "
                      : "";
  error_string += joint_motion_generator_position_limits_violation
                      ? "\"joint_motion_generator_position_limits_violation\", "
                      : "";
  error_string += joint_motion_generator_velocity_limits_violation
                      ? "\"joint_motion_generator_velocity_limits_violation\", "
                      : "";
  error_string += joint_motion_generator_velocity_discontinuity
                      ? "\"joint_motion_generator_velocity_discontinuity\", "
                      : "";
  error_string += joint_motion_generator_acceleration_discontinuity
                      ? "\"joint_motion_generator_acceleration_discontinuity\", "
                      : "";
  error_string += cartesian_position_motion_generator_start_pose_invalid
                      ? "\"cartesian_position_motion_generator_start_pose_invalid\", "
                      : "";
  error_string += cartesian_motion_generator_elbow_limit_violation
                      ? "\"cartesian_motion_generator_elbow_limit_violation\", "
                      : "";
  error_string += cartesian_motion_generator_velocity_limits_violation
                      ? "\"cartesian_motion_generator_velocity_limits_violation\", "
                      : "";
  error_string += cartesian_motion_generator_velocity_discontinuity
                      ? "\"cartesian_motion_generator_velocity_discontinuity\", "
                      : "";
  error_string += cartesian_motion_generator_acceleration_discontinuity
                      ? "\"cartesian_motion_generator_acceleration_discontinuity\", "
                      : "";
  error_string += cartesian_motion_generator_elbow_sign_inconsistent
                      ? "\"cartesian_motion_generator_elbow_sign_inconsistent\", "
                      : "";
  error_string += cartesian_motion_generator_start_elbow_invalid
                      ? "\"cartesian_motion_generator_start_elbow_invalid\", "
                      : "";
  error_string += cartesian_motion_generator_joint_position_limits_violation
                      ? "\"cartesian_motion_generator_joint_position_limits_violation\", "
                      : "";
  error_string += cartesian_motion_generator_joint_velocity_limits_violation
                      ? "\"cartesian_motion_generator_joint_velocity_limits_violation\", "
                      : "";
  error_string += cartesian_motion_generator_joint_velocity_discontinuity
                      ? "\"cartesian_motion_generator_joint_velocity_discontinuity\", "
                      : "";
  error_string += cartesian_motion_generator_joint_acceleration_discontinuity
                      ? "\"cartesian_motion_generator_joint_acceleration_discontinuity\", "
                      : "";
  error_string += cartesian_position_motion_generator_invalid_frame
                      ? "\"cartesian_position_motion_generator_invalid_frame_flag\", "
                      : "";
  error_string += controller_torque_discontinuity ? "\"controller_torque_discontinuity\", " : "";
  error_string += force_controller_desired_force_tolerance_violation
                      ? "\"force_controller_desired_force_tolerance_violation\", "
                      : "";
  error_string += start_elbow_sign_inconsistent ? "\"start_elbow_sign_inconsistent\", " : "";
  error_string +=
      communication_constraints_violation ? "\"communication_constraints_violation\", " : "";
  error_string += power_limit_violation ? "\"power_limit_violation\", " : "";

  if (error_string.size() > 1) {
    error_string.erase(error_string.end() - 2, error_string.end());
  }

  error_string += "]";

  return error_string;
}

std::ostream& operator<<(std::ostream& ostream, const Errors& errors) {
  ostream << static_cast<std::string>(errors);
  return ostream;
}

}  // namespace franka
