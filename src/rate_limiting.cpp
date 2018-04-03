#include <franka/rate_limiting.h>

#include <Eigen/Dense>

#include <franka/exception.h>

namespace franka {

std::array<double, 7> limitRate(const std::array<double, 7>& max_derivatives,
                                const std::array<double, 7>& desired_values,
                                const std::array<double, 7>& last_desired_values) {
  std::array<double, 7> limited_values{};
  for (size_t i = 0; i < 7; i++) {
    double desired_derivative = (desired_values[i] - last_desired_values[i]) / kDeltaT;
    limited_values[i] =
        last_desired_values[i] +
        std::max(std::min(desired_derivative, max_derivatives[i]), -max_derivatives[i]) * kDeltaT;
  }
  return limited_values;
}

double limitRate(double max_velocity,
                 double max_acceleration,
                 double max_jerk,
                 double desired_velocity,
                 double last_desired_velocity,
                 double last_desired_acceleration) {
  // Differentiate to get jerk
  double desired_jerk =
      (((desired_velocity - last_desired_velocity) / kDeltaT) - last_desired_acceleration) /
      kDeltaT;

  // Limit jerk
  double desired_acceleration =
      last_desired_acceleration + std::max(std::min(desired_jerk, max_jerk), -max_jerk) * kDeltaT;

  // Compute acceleration limits
  double safe_max_acceleration = std::min(
      (max_jerk / max_acceleration) * (max_velocity - last_desired_velocity), max_acceleration);
  double safe_min_acceleration = std::max(
      (max_jerk / max_acceleration) * (-max_velocity - last_desired_velocity), -max_acceleration);

  // Limit acceleration and integrate to get desired velocities
  return last_desired_velocity +
         std::max(std::min(desired_acceleration, safe_max_acceleration), safe_min_acceleration) *
             kDeltaT;
}

double limitRate(double max_velocity,
                 double max_acceleration,
                 double max_jerk,
                 double desired_position,
                 double last_desired_position,
                 double last_desired_velocity,
                 double last_desired_acceleration) {
  return last_desired_position +
         limitRate(max_velocity, max_acceleration, max_jerk,
                   (desired_position - last_desired_position) / kDeltaT, last_desired_velocity,
                   last_desired_acceleration) *
             kDeltaT;
}

std::array<double, 7> limitRate(const std::array<double, 7>& max_velocity,
                                const std::array<double, 7>& max_acceleration,
                                const std::array<double, 7>& max_jerk,
                                const std::array<double, 7>& desired_velocities,
                                const std::array<double, 7>& last_desired_velocities,
                                const std::array<double, 7>& last_desired_accelerations) {
  std::array<double, 7> limited_desired_velocities{};

  for (size_t i = 0; i < 7; i++) {
    limited_desired_velocities[i] =
        limitRate(max_velocity[i], max_acceleration[i], max_jerk[i], desired_velocities[i],
                  last_desired_velocities[i], last_desired_accelerations[i]);
  }

  return limited_desired_velocities;
}

std::array<double, 7> limitRate(const std::array<double, 7>& max_velocity,
                                const std::array<double, 7>& max_acceleration,
                                const std::array<double, 7>& max_jerk,
                                const std::array<double, 7>& desired_positions,
                                const std::array<double, 7>& last_desired_positions,
                                const std::array<double, 7>& last_desired_velocities,
                                const std::array<double, 7>& last_desired_accelerations) {
  std::array<double, 7> limited_desired_positions{};
  for (size_t i = 0; i < 7; i++) {
    limited_desired_positions[i] = limitRate(
        max_velocity[i], max_acceleration[i], max_jerk[i], desired_positions[i],
        last_desired_positions[i], last_desired_velocities[i], last_desired_accelerations[i]);
  }
  return limited_desired_positions;
}

namespace {
Eigen::Vector3d limitRate(double max_velocity,
                          double max_acceleration,
                          double max_jerk,
                          Eigen::Vector3d desired_velocity,
                          Eigen::Vector3d last_desired_velocity,
                          Eigen::Vector3d last_desired_acceleration) {
  // Differentiate to get jerk
  Eigen::Vector3d desired_jerk =
      (((desired_velocity - last_desired_velocity) / kDeltaT) - last_desired_acceleration) /
      kDeltaT;

  // Limit jerk and get desired acceleration
  Eigen::Vector3d desired_acceleration = last_desired_acceleration;
  if (desired_jerk.norm() > kNormEps) {
    desired_acceleration += (desired_jerk / desired_jerk.norm()) *
                            std::max(std::min(desired_jerk.norm(), max_jerk), -max_jerk) * kDeltaT;
  }

  // Compute acceleration limits
  double safe_max_acceleration =
      std::min((max_jerk / max_acceleration) * (max_velocity - last_desired_velocity.norm()),
               max_acceleration);
  double safe_min_acceleration =
      std::max((max_jerk / max_acceleration) * (-max_velocity - last_desired_velocity.norm()),
               -max_acceleration);

  // Limit acceleration and integrate to get desired velocities
  Eigen::Vector3d limited_desired_velocity = last_desired_velocity;

  if (desired_acceleration.norm() > kNormEps) {
    limited_desired_velocity +=
        (desired_acceleration / desired_acceleration.norm()) *
        std::max(std::min(desired_acceleration.norm(), safe_max_acceleration),
                 safe_min_acceleration) *
        kDeltaT;
  }

  return limited_desired_velocity;
}
}

std::array<double, 6> limitRate(
    double max_translational_velocity,
    double max_translational_acceleration,
    double max_translational_jerk,
    double max_rotational_velocity,
    double max_rotational_acceleration,
    double max_rotational_jerk,
    const std::array<double, 6>& O_dP_EE_d,          // NOLINT (readability-identifier-naming)
    const std::array<double, 6>& last_O_dP_EE_d,     // NOLINT (readability-identifier-naming)
    const std::array<double, 6>& last_O_ddP_EE_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 6, 1> dx(O_dP_EE_d.data());
  Eigen::Matrix<double, 6, 1> last_dx(last_O_dP_EE_d.data());
  Eigen::Matrix<double, 6, 1> last_ddx(last_O_ddP_EE_d.data());

  dx.head(3) << limitRate(max_translational_velocity, max_translational_acceleration,
                          max_translational_jerk, dx.head(3), last_dx.head(3), last_ddx.head(3));
  dx.tail(3) << limitRate(max_rotational_velocity, max_rotational_acceleration, max_rotational_jerk,
                          dx.tail(3), last_dx.tail(3), last_ddx.tail(3));

  std::array<double, 6> limited_values{};
  Eigen::Map<Eigen::Matrix<double, 6, 1>>(&limited_values[0], 6, 1) = dx;
  return limited_values;
}

std::array<double, 16> limitRate(
    double max_translational_velocity,
    double max_translational_acceleration,
    double max_translational_jerk,
    double max_rotational_velocity,
    double max_rotational_acceleration,
    double max_rotational_jerk,
    const std::array<double, 16>& O_T_EE_d,          // NOLINT (readability-identifier-naming)
    const std::array<double, 16>& last_O_T_EE_d,     // NOLINT (readability-identifier-naming)
    const std::array<double, 6>& last_O_dP_EE_d,     // NOLINT (readability-identifier-naming)
    const std::array<double, 6>& last_O_ddP_EE_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 6, 1> dx;
  Eigen::Affine3d desired_pose(Eigen::Matrix4d::Map(O_T_EE_d.data()));
  Eigen::Affine3d limited_desired_pose = Eigen::Affine3d::Identity();
  Eigen::Affine3d last_desired_pose(Eigen::Matrix4d::Map(last_O_T_EE_d.data()));

  // Compute translational velocity
  dx.head(3) << (desired_pose.translation() - last_desired_pose.translation()) / kDeltaT;

  // Compute rotational velocity
  auto delta_rotation = (desired_pose.linear() - last_desired_pose.linear()) / kDeltaT;
  Eigen::Matrix3d rotational_twist = delta_rotation * last_desired_pose.linear();
  dx.tail(3) << rotational_twist(2, 1), rotational_twist(0, 2), rotational_twist(1, 0);

  // Limit the rate of the twist
  std::array<double, 6> desired_O_dP_EE_d{};  // NOLINT (readability-identifier-naming)
  Eigen::Map<Eigen::Matrix<double, 6, 1>>(&desired_O_dP_EE_d[0], 6, 1) = dx;
  desired_O_dP_EE_d =
      limitRate(max_translational_velocity, max_translational_acceleration, max_translational_jerk,
                max_rotational_velocity, max_rotational_acceleration, max_rotational_jerk,
                desired_O_dP_EE_d, last_O_dP_EE_d, last_O_ddP_EE_d);
  dx = Eigen::Matrix<double, 6, 1>(desired_O_dP_EE_d.data());

  // Integrate limited twist
  Eigen::Matrix3d omega_skew;
  omega_skew << 0, -dx(5), dx(4), dx(5), 0, -dx(3), -dx(4), dx(3), 0;
  limited_desired_pose.linear() << last_desired_pose.linear() +
                                       omega_skew * last_desired_pose.linear() * kDeltaT;
  limited_desired_pose.translation() << last_desired_pose.translation() + dx.head(3) * kDeltaT;

  std::array<double, 16> limited_values{};
  Eigen::Map<Eigen::Matrix4d>(&limited_values[0], 4, 4) = limited_desired_pose.matrix();
  return limited_values;
}

}  // namespace franka
