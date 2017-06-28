#include <franka/errors.h>

#include <algorithm>

namespace franka {

Errors::Errors(std::array<bool, 24> errors)
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
      force_controller_desired_force_tolerance_violation(errors[23]) {}

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
         force_controller_desired_force_tolerance_violation;
}

std::string activeErrorsString(Errors& errors) {
  std::string names;

  names += errors.joint_position_limits_violation ? "joint_position_limits_violation, " : "";
  names +=
      errors.cartesian_position_limits_violation ? "cartesian_position_limits_violation, " : "";
  names += errors.self_collision_avoidance_violation ? "self_collision_avoidance_violation, " : "";
  names += errors.joint_velocity_violation ? "joint_velocity_violation, " : "";
  names += errors.cartesian_velocity_violation ? "cartesian_velocity_violation, " : "";
  names += errors.force_control_safety_violation ? "force_control_safety_violation, " : "";
  names += errors.joint_reflex ? "joint_reflex, " : "";
  names += errors.cartesian_reflex ? "cartesian_reflex, " : "";
  names += errors.max_goal_pose_deviation_violation ? "max_goal_pose_deviation_violation, " : "";
  names += errors.max_path_pose_deviation_violation ? "max_path_pose_deviation_violation, " : "";
  names += errors.cartesian_velocity_profile_safety_violation
               ? "cartesian_velocity_profile_safety_violation, "
               : "";
  names += errors.joint_position_motion_generator_start_pose_invalid
               ? "joint_position_motion_generator_start_pose_invalid, "
               : "";
  names += errors.joint_motion_generator_position_limits_violation
               ? "joint_motion_generator_position_limits_violation, "
               : "";
  names += errors.joint_motion_generator_velocity_limits_violation
               ? "joint_motion_generator_velocity_limits_violation, "
               : "";
  names += errors.joint_motion_generator_velocity_discontinuity
               ? "joint_motion_generator_velocity_discontinuity, "
               : "";
  names += errors.joint_motion_generator_acceleration_discontinuity
               ? "joint_motion_generator_acceleration_discontinuity, "
               : "";
  names += errors.cartesian_position_motion_generator_start_pose_invalid
               ? "cartesian_position_motion_generator_start_pose_invalid, "
               : "";
  names += errors.cartesian_motion_generator_elbow_limit_violation
               ? "cartesian_motion_generator_elbow_limit_violation, "
               : "";
  names += errors.cartesian_motion_generator_velocity_limits_violation
               ? "cartesian_motion_generator_velocity_limits_violation, "
               : "";
  names += errors.cartesian_motion_generator_velocity_discontinuity
               ? "cartesian_motion_generator_velocity_discontinuity, "
               : "";
  names += errors.cartesian_motion_generator_acceleration_discontinuity
               ? "cartesian_motion_generator_acceleration_discontinuity, "
               : "";
  names += errors.cartesian_motion_generator_elbow_sign_inconsistent
               ? "cartesian_motion_generator_elbow_sign_inconsistent, "
               : "";
  names += errors.cartesian_motion_generator_start_elbow_invalid
               ? "cartesian_motion_generator_start_elbow_invalid, "
               : "";
  names += errors.force_controller_desired_force_tolerance_violation
               ? "force_controller_desired_force_tolerance_violation, "
               : "";

  if (names.size() > 0) {
    names.erase(names.end() - 2, names.end());
  }

  return names;
}

std::ostream& operator<<(std::ostream& ostream, const Errors& errors) {
  ostream << "{joint_position_limits_violation : " << errors.joint_position_limits_violation
          << ", cartesian_position_limits_violation : "
          << errors.cartesian_position_limits_violation
          << ", self_collision_avoidance_violation : " << errors.self_collision_avoidance_violation
          << ", joint_velocity_violation : " << errors.joint_velocity_violation
          << ", cartesian_velocity_violation : " << errors.cartesian_velocity_violation
          << ", force_control_safety_violation : " << errors.force_control_safety_violation
          << ", joint_reflex : " << errors.joint_reflex
          << ", cartesian_reflex : " << errors.cartesian_reflex
          << ", max_goal_pose_deviation_violation : " << errors.max_goal_pose_deviation_violation
          << ", max_path_pose_deviation_violation : " << errors.max_path_pose_deviation_violation
          << ", cartesian_velocity_profile_safety_violation : "
          << errors.cartesian_velocity_profile_safety_violation
          << ", joint_position_motion_generator_start_pose_invalid : "
          << errors.joint_position_motion_generator_start_pose_invalid
          << ", joint_motion_generator_position_limits_violation : "
          << errors.joint_motion_generator_position_limits_violation
          << ", joint_motion_generator_velocity_limits_violation : "
          << errors.joint_motion_generator_velocity_limits_violation
          << ", joint_motion_generator_velocity_discontinuity : "
          << errors.joint_motion_generator_velocity_discontinuity
          << ", joint_motion_generator_acceleration_discontinuity : "
          << errors.joint_motion_generator_acceleration_discontinuity
          << ", cartesian_position_motion_generator_start_pose_invalid : "
          << errors.cartesian_position_motion_generator_start_pose_invalid
          << ", cartesian_motion_generator_elbow_limit_violation : "
          << errors.cartesian_motion_generator_elbow_limit_violation
          << ", cartesian_motion_generator_velocity_limits_violation : "
          << errors.cartesian_motion_generator_velocity_limits_violation
          << ", cartesian_motion_generator_velocity_discontinuity : "
          << errors.cartesian_motion_generator_velocity_discontinuity
          << ", cartesian_motion_generator_acceleration_discontinuity : "
          << errors.cartesian_motion_generator_acceleration_discontinuity
          << ", cartesian_motion_generator_elbow_sign_inconsistent : "
          << errors.cartesian_motion_generator_elbow_sign_inconsistent
          << ", cartesian_motion_generator_start_elbow_invalid : "
          << errors.cartesian_motion_generator_start_elbow_invalid
          << ", force_controller_desired_force_tolerance_violation : "
          << errors.force_controller_desired_force_tolerance_violation << "}";
  return ostream;
}

}  // namespace franka
