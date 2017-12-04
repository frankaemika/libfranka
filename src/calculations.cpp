// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "calculations.h"

#include <algorithm>

namespace franka {

std::array<double, 3> combineCenterOfMass(
    double m_ee,
    const std::array<double, 3>& F_x_Cee,  // NOLINT (readability-identifier-naming)
    double m_load,
    const std::array<double, 3>& F_x_Cload) {  // NOLINT (readability-identifier-naming)
  std::array<double, 3> F_x_Ctotal{};          // NOLINT (readability-identifier-naming)
  if (m_load + m_ee > 0) {
    std::transform(
        F_x_Cload.cbegin(), F_x_Cload.cend(), F_x_Cee.cbegin(), F_x_Ctotal.begin(),
        [&m_load, &m_ee](double current_center_of_mass_load,
                         double current_center_of_mass_ee) -> double {
          return ((m_load * current_center_of_mass_load + m_ee * current_center_of_mass_ee) /
                  (m_load + m_ee));
        });
  }
  return F_x_Ctotal;
}

Eigen::Matrix3d skewSymmetricMatrixFromVector(const Eigen::Vector3d& input) {
  Eigen::Matrix3d input_hat;
  input_hat << 0, -input(2), input(1), input(2), 0, -input(0), -input(1), input(0), 0;
  return input_hat;
}

std::array<double, 9> combineInertiaTensor(
    double m_ee,
    const std::array<double, 3>& F_x_Cee,  // NOLINT (readability-identifier-naming)
    const std::array<double, 9>& I_ee,     // NOLINT (readability-identifier-naming)
    double m_load,
    const std::array<double, 3>& F_x_Cload,  // NOLINT (readability-identifier-naming)
    const std::array<double, 9>& I_load,     // NOLINT (readability-identifier-naming)
    double m_total,
    const std::array<double, 3>& F_x_Ctotal) {  // NOLINT (readability-identifier-naming)
  // If the combined mass equals to zero, the combined inertia is also zero.
  if (m_total == 0) {
    return std::array<double, 9>{};
  }

  Eigen::Vector3d center_of_mass_load(F_x_Cload.data());
  Eigen::Vector3d center_of_mass_ee(F_x_Cee.data());
  Eigen::Vector3d center_of_mass_total(F_x_Ctotal.data());

  Eigen::Matrix3d inertia_load(I_load.data());
  Eigen::Matrix3d inertia_load_flange = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d inertia_ee(I_ee.data());
  Eigen::Matrix3d inertia_ee_flange = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d inertia_total = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d inertia_total_flange = Eigen::Matrix3d::Zero();

  // Check if the mass equals zero, the inertia should then be zero as well.
  if (m_load == 0) {
    inertia_load = Eigen::Matrix3d::Zero();
  }
  if (m_ee == 0) {
    inertia_ee = Eigen::Matrix3d::Zero();
  }

  // Calculate inertia tensor of load and EE in flange coordinates.
  inertia_load_flange = inertia_load -
                        m_load * (skewSymmetricMatrixFromVector(center_of_mass_load) *
                                  skewSymmetricMatrixFromVector(center_of_mass_load));
  inertia_ee_flange = inertia_ee -
                      m_ee * (skewSymmetricMatrixFromVector(center_of_mass_ee) *
                              skewSymmetricMatrixFromVector(center_of_mass_ee));

  // Calculate combined inertia tensor in flange coordinate.
  inertia_total_flange = inertia_load_flange + inertia_ee_flange;

  // Calculate combined inertia tensor in combined body center of mass coordinate.
  inertia_total = inertia_total_flange +
                  m_total * (skewSymmetricMatrixFromVector(center_of_mass_total) *
                             skewSymmetricMatrixFromVector(center_of_mass_total));

  std::array<double, 9> I_total;  // NOLINT (readability-identifier-naming)
  Eigen::Map<Eigen::Matrix3d>(I_total.data(), 3, 3) = inertia_total;
  return I_total;
}

}  // namespace franka