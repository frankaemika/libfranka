#include <franka/errors.h>

#include <algorithm>
#include <iterator>
#include <sstream>

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

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

namespace {

void addName(std::vector<std::string>& vector,
             bool error,
             const std::string& name,
             bool only_active,
             bool include_value) {
  if (!only_active || error) {
    std::string value_string = include_value ? (": "s + (error ? "true"s : "false"s)) : ""s;
    vector.push_back(name + value_string);
  }
}

std::string errorNames(const Errors& errors, bool only_active, bool include_value) {
  std::vector<std::string> names;

  addName(names, errors.joint_position_limits_violation, "joint_position_limits_violation",
          only_active, include_value);
  addName(names, errors.cartesian_position_limits_violation, "cartesian_position_limits_violation",
          only_active, include_value);
  addName(names, errors.self_collision_avoidance_violation, "self_collision_avoidance_violation",
          only_active, include_value);
  addName(names, errors.joint_velocity_violation, "joint_velocity_violation", only_active,
          include_value);
  addName(names, errors.cartesian_velocity_violation, "cartesian_velocity_violation", only_active,
          include_value);
  addName(names, errors.force_control_safety_violation, "force_control_safety_violation",
          only_active, include_value);
  addName(names, errors.joint_reflex, "joint_reflex", only_active, include_value);
  addName(names, errors.cartesian_reflex, "cartesian_reflex", only_active, include_value);
  addName(names, errors.max_goal_pose_deviation_violation, "max_goal_pose_deviation_violation",
          only_active, include_value);
  addName(names, errors.max_path_pose_deviation_violation, "max_path_pose_deviation_violation",
          only_active, include_value);
  addName(names, errors.cartesian_velocity_profile_safety_violation,
          "cartesian_velocity_profile_safety_violation", only_active, include_value);
  addName(names, errors.joint_position_motion_generator_start_pose_invalid,
          "joint_position_motion_generator_start_pose_invalid", only_active, include_value);
  addName(names, errors.joint_motion_generator_position_limits_violation,
          "joint_motion_generator_position_limits_violation", only_active, include_value);
  addName(names, errors.joint_motion_generator_velocity_limits_violation,
          "joint_motion_generator_velocity_limits_violation", only_active, include_value);
  addName(names, errors.joint_motion_generator_velocity_discontinuity,
          "joint_motion_generator_velocity_discontinuity", only_active, include_value);
  addName(names, errors.joint_motion_generator_acceleration_discontinuity,
          "joint_motion_generator_acceleration_discontinuity", only_active, include_value);
  addName(names, errors.cartesian_position_motion_generator_start_pose_invalid,
          "cartesian_position_motion_generator_start_pose_invalid", only_active, include_value);
  addName(names, errors.cartesian_motion_generator_elbow_limit_violation,
          "cartesian_motion_generator_elbow_limit_violation", only_active, include_value);
  addName(names, errors.cartesian_motion_generator_velocity_limits_violation,
          "cartesian_motion_generator_velocity_limits_violation", only_active, include_value);
  addName(names, errors.cartesian_motion_generator_velocity_discontinuity,
          "cartesian_motion_generator_velocity_discontinuity", only_active, include_value);
  addName(names, errors.cartesian_motion_generator_acceleration_discontinuity,
          "cartesian_motion_generator_acceleration_discontinuity", only_active, include_value);
  addName(names, errors.cartesian_motion_generator_elbow_sign_inconsistent,
          "cartesian_motion_generator_elbow_sign_inconsistent", only_active, include_value);
  addName(names, errors.cartesian_motion_generator_start_elbow_invalid,
          "cartesian_motion_generator_start_elbow_invalid", only_active, include_value);
  addName(names, errors.force_controller_desired_force_tolerance_violation,
          "force_controller_desired_force_tolerance_violation", only_active, include_value);

  if (!names.empty()) {
    std::ostringstream string_stream;
    std::copy(names.cbegin(), names.cend() - 1,
              std::ostream_iterator<std::string>(string_stream, ", "));
    std::copy(names.cend() - 1, names.cend(), std::ostream_iterator<std::string>(string_stream));
    return string_stream.str();
  }
  return "";
}

}  // anonymous namespace

std::string activeErrorsString(Errors& errors) {
  return errorNames(errors, true, false);
}

std::ostream& operator<<(std::ostream& ostream, const Errors& errors) {
  ostream << "{" << errorNames(errors, false, true) << "}";
  return ostream;
}

}  // namespace franka
