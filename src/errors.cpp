#include <franka/errors.h>

#include <algorithm>
#include <iterator>
#include <sstream>

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

Errors::Errors(std::array<bool, 27> errors)
    : joint_position_limits_violation(errors[0]),
      cartesian_position_limits_violation(errors[1]),
      self_collision_avoidance_violation(errors[2]),
      joint_velocity_violation(errors[3]),
      cartesian_velocity_violation(errors[4]),
      force_control_safety_violation(errors[5]),
      joint_reflex(errors[6]),
      cartesian_reflex(errors[7]),
      max_goal_pose_deviation_violation(errors[8]),
      max_path_pose_deviation_violation(errors[9]),
      cartesian_velocity_profile_safety_violation(errors[10]),
      joint_position_motion_generator_start_pose_invalid(errors[11]),
      joint_motion_generator_position_limits_violation(errors[12]),
      joint_motion_generator_velocity_limits_violation(errors[13]),
      joint_motion_generator_velocity_discontinuity(errors[14]),
      joint_motion_generator_acceleration_discontinuity(errors[15]),
      cartesian_position_motion_generator_start_pose_invalid(errors[16]),
      cartesian_motion_generator_elbow_limit_violation(errors[17]),
      cartesian_motion_generator_velocity_limits_violation(errors[18]),
      cartesian_motion_generator_velocity_discontinuity(errors[19]),
      cartesian_motion_generator_acceleration_discontinuity(errors[20]),
      cartesian_motion_generator_elbow_sign_inconsistent(errors[21]),
      cartesian_motion_generator_start_elbow_invalid(errors[22]),
      force_controller_desired_force_tolerance_violation(errors[23]),
      start_elbow_sign_inconsistent(errors[24]),
      communication_constraints_violation(errors[25]),
      power_limit_violation(errors[26]) {}

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
         communication_constraints_violation || power_limit_violation;
}

Errors::operator std::string() const {
  std::string error_string = "{";

  error_string += joint_position_limits_violation ? "joint_position_limits_violation, " : "";
  error_string +=
      cartesian_position_limits_violation ? "cartesian_position_limits_violation, " : "";
  error_string += self_collision_avoidance_violation ? "self_collision_avoidance_violation, " : "";
  error_string += joint_velocity_violation ? "joint_velocity_violation, " : "";
  error_string += cartesian_velocity_violation ? "cartesian_velocity_violation, " : "";
  error_string += force_control_safety_violation ? "force_control_safety_violation, " : "";
  error_string += joint_reflex ? "joint_reflex, " : "";
  error_string += cartesian_reflex ? "cartesian_reflex, " : "";
  error_string += max_goal_pose_deviation_violation ? "max_goal_pose_deviation_violation, " : "";
  error_string += max_path_pose_deviation_violation ? "max_path_pose_deviation_violation, " : "";
  error_string += cartesian_velocity_profile_safety_violation
                      ? "cartesian_velocity_profile_safety_violation, "
                      : "";
  error_string += joint_position_motion_generator_start_pose_invalid
                      ? "joint_position_motion_generator_start_pose_invalid, "
                      : "";
  error_string += joint_motion_generator_position_limits_violation
                      ? "joint_motion_generator_position_limits_violation, "
                      : "";
  error_string += joint_motion_generator_velocity_limits_violation
                      ? "joint_motion_generator_velocity_limits_violation, "
                      : "";
  error_string += joint_motion_generator_velocity_discontinuity
                      ? "joint_motion_generator_velocity_discontinuity, "
                      : "";
  error_string += joint_motion_generator_acceleration_discontinuity
                      ? "joint_motion_generator_acceleration_discontinuity, "
                      : "";
  error_string += cartesian_position_motion_generator_start_pose_invalid
                      ? "cartesian_position_motion_generator_start_pose_invalid, "
                      : "";
  error_string += cartesian_motion_generator_elbow_limit_violation
                      ? "cartesian_motion_generator_elbow_limit_violation, "
                      : "";
  error_string += cartesian_motion_generator_velocity_limits_violation
                      ? "cartesian_motion_generator_velocity_limits_violation, "
                      : "";
  error_string += cartesian_motion_generator_velocity_discontinuity
                      ? "cartesian_motion_generator_velocity_discontinuity, "
                      : "";
  error_string += cartesian_motion_generator_acceleration_discontinuity
                      ? "cartesian_motion_generator_acceleration_discontinuity, "
                      : "";
  error_string += cartesian_motion_generator_elbow_sign_inconsistent
                      ? "cartesian_motion_generator_elbow_sign_inconsistent, "
                      : "";
  error_string += cartesian_motion_generator_start_elbow_invalid
                      ? "cartesian_motion_generator_start_elbow_invalid, "
                      : "";
  error_string += force_controller_desired_force_tolerance_violation
                      ? "force_controller_desired_force_tolerance_violation, "
                      : "";
  error_string += start_elbow_sign_inconsistent ? "start_elbow_sign_inconsistent, " : "";
  error_string +=
      communication_constraints_violation ? "communication_constraints_violation, " : "";
  error_string += power_limit_violation ? "power_limit_violation, " : "";

  if (error_string.size() > 1) {
    error_string.erase(error_string.end() - 2, error_string.end());
  }

  error_string += "}";

  return error_string;
}

std::ostream& operator<<(std::ostream& ostream, const Errors& errors) {
  ostream << static_cast<std::string>(errors);
  return ostream;
}

}  // namespace franka
