#include <franka/control_types.h>
#include <franka/exception.h>

namespace franka {

Torques::Torques(const std::array<double, 7>&  // NOLINT (modernize-pass-by-value)
                     torques) noexcept
    : tau_J(torques) {}

Torques::Torques(std::initializer_list<double> torques) {
  if (torques.size() != tau_J.size()) {
    throw ControlException("Invalid number of elements in tau_J.");
  }
  std::copy(torques.begin(), torques.end(), tau_J.begin());
}

JointValues::JointValues(
    const std::array<double, 7>&  // NOLINT (modernize-pass-by-value)
        joint_values) noexcept
    : q(joint_values) {}

JointValues::JointValues(std::initializer_list<double> joint_values) {
  if (joint_values.size() != q.size()) {
    throw ControlException("Invalid number of elements in joint_values.");
  }
  std::copy(joint_values.begin(), joint_values.end(), q.begin());
}

JointVelocities::JointVelocities(
    const std::array<double, 7>&  // NOLINT (modernize-pass-by-value)
        joint_velocities) noexcept
    : dq(joint_velocities) {}

JointVelocities::JointVelocities(
    std::initializer_list<double> joint_velocities) {
  if (joint_velocities.size() != dq.size()) {
    throw ControlException("Invalid number of elements in joint_velocities.");
  }
  std::copy(joint_velocities.begin(), joint_velocities.end(), dq.begin());
}

CartesianPose::CartesianPose(
    const std::array<double, 16>&  // NOLINT (modernize-pass-by-value)
        cartesian_pose) noexcept
    : O_T_EE(cartesian_pose) {}

CartesianPose::CartesianPose(std::initializer_list<double> cartesian_pose) {
  if (cartesian_pose.size() != O_T_EE.size()) {
    throw ControlException("Invalid number of elements in cartesian_pose.");
  }
  std::copy(cartesian_pose.begin(), cartesian_pose.end(), O_T_EE.begin());
}

CartesianVelocities::CartesianVelocities(
    const std::array<double, 6>&  // NOLINT (modernize-pass-by-value)
        cartesian_velocities)
    : O_dP_EE(cartesian_velocities) {}

CartesianVelocities::CartesianVelocities(
    std::initializer_list<double> cartesian_velocities) {
  if (cartesian_velocities.size() != O_dP_EE.size()) {
    throw ControlException(
        "Invalid number of elements in cartesian_velocities.");
  }
  std::copy(cartesian_velocities.begin(), cartesian_velocities.end(),
            O_dP_EE.begin());
}

}  // namespace franka
