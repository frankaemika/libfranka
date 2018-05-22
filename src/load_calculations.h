// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>

#include <Eigen/Core>

namespace franka {

std::array<double, 3> combineCenterOfMass(
    double m_ee,
    const std::array<double, 3>& F_x_Cee,  // NOLINT(readability-identifier-naming)
    double m_load,
    const std::array<double, 3>& F_x_Cload);  // NOLINT(readability-identifier-naming)

Eigen::Matrix3d skewSymmetricMatrixFromVector(const Eigen::Vector3d& input);

std::array<double, 9> combineInertiaTensor(
    double m_ee,
    const std::array<double, 3>& F_x_Cee,  // NOLINT(readability-identifier-naming)
    const std::array<double, 9>& I_ee,     // NOLINT(readability-identifier-naming)
    double m_load,
    const std::array<double, 3>& F_x_Cload,  // NOLINT(readability-identifier-naming)
    const std::array<double, 9>& I_load,     // NOLINT(readability-identifier-naming)
    double m_total,
    const std::array<double, 3>& F_x_Ctotal);  // NOLINT(readability-identifier-naming)

}  // namespace franka