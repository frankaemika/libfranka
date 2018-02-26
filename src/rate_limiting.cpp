#include <franka/rate_limiting.h>

#include <Eigen/Dense>

#include <franka/exception.h>

namespace franka {

std::array<double, 7> limitRate(const std::array<double, 7>& max_derivatives,
                                const std::array<double, 7>& desired_values,
                                const std::array<double, 7>& last_desired_values) {
  std::array<double, 7> limited_values{};
  for (size_t i = 0; i < 7; i++) {
    double desired_difference = (desired_values[i] - last_desired_values[i]) / 1e-3;
    limited_values[i] =
        last_desired_values[i] +
        std::max(std::min(desired_difference, max_derivatives[i]), -max_derivatives[i]) * 1e-3;
  }
  return limited_values;
}

std::array<double, 7> limitRate(const std::array<double, 7>& max_velocity,
                                const std::array<double, 7>& max_acceleration,
                                const std::array<double, 7>& desired_velocities,
                                const std::array<double, 7>& last_desired_velocities) {
  std::array<double, 7> limited_desired_velocities = desired_velocities;

  // 1. Calculate limited desired velocity
  for (size_t i = 0; i < 7; i++) {
    limited_desired_velocities[i] =
        std::max(std::min(limited_desired_velocities[i], max_velocity[i]), -max_velocity[i]);
  }
  // 2. Calculate desired acceleration from desired velocity and current velocity
  std::array<double, 7> desired_acceleration{};
  for (size_t i = 0; i < 7; i++) {
    desired_acceleration[i] = (limited_desired_velocities[i] - last_desired_velocities[i]) / 1e-3;
  }
  // 3. Calculate limited desired acceleration
  std::array<double, 7> limited_desired_acceleration{};
  for (size_t i = 0; i < 7; i++) {
    limited_desired_acceleration[i] =
        std::max(std::min(desired_acceleration[i], max_acceleration[i]), -max_acceleration[i]);
  }
  // 4. Calculate limited desired velocity
  for (size_t i = 0; i < 7; i++) {
    limited_desired_velocities[i] =
        last_desired_velocities[i] + limited_desired_acceleration[i] * 1e-3;
  }

  return limited_desired_velocities;
}

std::array<double, 7> limitRate(const std::array<double, 7>& max_velocity,
                                const std::array<double, 7>& max_acceleration,
                                const std::array<double, 7>& desired_positions,
                                const std::array<double, 7>& last_desired_positions,
                                const std::array<double, 7>& last_desired_velocities) {
  // 1. Calculate desired velocity from two joint positions
  std::array<double, 7> desired_velocities{};
  for (size_t i = 0; i < 7; i++) {
    desired_velocities[i] = (desired_positions[i] - last_desired_positions[i]) / 1e-3;
  }

  // 2. Limit the velocity and acceleration
  std::array<double, 7> limited_desired_velocities{};
  limited_desired_velocities =
      limitRate(max_velocity, max_acceleration, desired_velocities, last_desired_velocities);

  // 3. Calculate the next joint position
  std::array<double, 7> limited_joint_positions{};
  for (size_t i = 0; i < 7; i++) {
    limited_joint_positions[i] = last_desired_positions[i] + limited_desired_velocities[i] * 1e-3;
  }

  return limited_joint_positions;
}

std::array<double, 6> limitRate(
    const double max_translational_velocity,
    const double max_translational_acceleration,
    const double max_rotational_velocity,
    const double max_rotational_acceleration,
    const std::array<double, 6>& O_dP_EE_d,         // NOLINT (readability-identifier-naming)
    const std::array<double, 6>& last_O_dP_EE_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 6, 1> dx(O_dP_EE_d.data());
  Eigen::Matrix<double, 6, 1> last_dx(last_O_dP_EE_d.data());
  Eigen::Matrix<double, 6, 1> ddx;

  // Compute acceleration
  ddx << (dx - last_dx) / 1e-3;
  //  Limit ddx
  if (ddx.head(3).norm() > max_translational_acceleration) {
    ddx.head(3) << (ddx.head(3) / ddx.head(3).norm()) * max_translational_acceleration;
  }
  if (ddx.tail(3).norm() > max_rotational_acceleration) {
    ddx.tail(3) << (ddx.tail(3) / ddx.tail(3).norm()) * max_rotational_acceleration;
  }

  dx = last_dx + ddx * 1e-3;
  // Saturate dx
  if (dx.head(3).norm() > max_translational_velocity) {
    dx.head(3) << (dx.head(3) / dx.head(3).norm()) * max_translational_velocity;
  }
  if (dx.tail(3).norm() > max_rotational_velocity) {
    dx.tail(3) << (dx.tail(3) / dx.tail(3).norm()) * max_rotational_velocity;
  }

  std::array<double, 6> limited_values{};
  Eigen::Map<Eigen::Matrix<double, 6, 1>>(&limited_values[0], 6, 1) = dx;
  return limited_values;
}

std::array<double, 16> limitRate(
    const double max_translational_velocity,
    const double max_translational_acceleration,
    const double max_rotational_velocity,
    const double max_rotational_acceleration,
    const std::array<double, 16>& O_T_EE_d,         // NOLINT (readability-identifier-naming)
    const std::array<double, 16>& last_O_T_EE_d,    // NOLINT (readability-identifier-naming)
    const std::array<double, 6>& last_O_dP_EE_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 6, 1> dx;
  Eigen::Matrix<double, 6, 1> ddx;
  Eigen::Matrix<double, 6, 1> last_dx(
      last_O_dP_EE_d.data());  // NOLINT (readability-identifier-naming)
  Eigen::Affine3d desired_pose(Eigen::Matrix4d::Map(O_T_EE_d.data()));
  Eigen::Affine3d limited_desired_pose = Eigen::Affine3d::Identity();
  Eigen::Affine3d last_desired_pose(Eigen::Matrix4d::Map(last_O_T_EE_d.data()));

  // Compute translational velocity
  dx.head(3) << (desired_pose.translation() - last_desired_pose.translation()) / 1e-3;

  // Compute rotational velocity
  auto delta_rotation = (desired_pose.linear() - last_desired_pose.linear()) / 1e-3;
  Eigen::Matrix3d rotational_twist = delta_rotation * last_desired_pose.linear();
  dx.tail(3) << rotational_twist(2, 1), rotational_twist(0, 2), rotational_twist(1, 0);

  // Limit the rate of the previously calculated cartesian velocity
  std::array<double, 6> current_O_dP_EE_d{};  // NOLINT (readability-identifier-naming)
  Eigen::Map<Eigen::Matrix<double, 6, 1>>(&current_O_dP_EE_d[0], 6, 1) = dx;
  current_O_dP_EE_d =
      limitRate(max_translational_velocity, max_translational_acceleration, max_rotational_velocity,
                max_rotational_acceleration, current_O_dP_EE_d, last_O_dP_EE_d);
  dx = Eigen::Matrix<double, 6, 1>(current_O_dP_EE_d.data());

  // Integrate to get new conform values
  limited_desired_pose.translation() << last_desired_pose.translation() + dx.head(3) * 1e-3;
  limited_desired_pose.linear() << last_desired_pose.linear();
  if (dx.tail(3).norm() > 0.0) {
    Eigen::Matrix3d omega_skew;
    Eigen::Vector3d w_norm(dx.tail(3) / dx.tail(3).norm());
    double theta = 1e-3 * dx.tail(3).norm();
    omega_skew << 0, -w_norm(2), w_norm(1), w_norm(2), 0, -w_norm(0), -w_norm(1), w_norm(0), 0;
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity() + sin(theta) * omega_skew +
                               (1.0 - cos(theta)) * (omega_skew * omega_skew);
    limited_desired_pose.linear() << rotation * last_desired_pose.linear();
  }

  std::array<double, 16> limited_values{};
  Eigen::Map<Eigen::Matrix4d>(&limited_values[0], 4, 4) = limited_desired_pose.matrix();
  return limited_values;
}

}  // namespace franka
