// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/model.h>

#include <sstream>

#include <research_interface/robot/service_types.h>

#include "model_library.h"
#include "network.h"

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace {

/**
 * Computes matrix multiplication C=A*B
 */
inline std::array<double, 16> matMul(const std::array<double, 16>& mat_a,
                                     const std::array<double, 16>& mat_b) {
  std::array<double, 16> mat_c;

  const double(&a)[4][4] = *reinterpret_cast<const double(*)[4][4]>(mat_a.data());
  const double(&b)[4][4] = *reinterpret_cast<const double(*)[4][4]>(mat_b.data());
  double(&c)[4][4] = *reinterpret_cast<double(*)[4][4]>(mat_c.data());

  c[0][0] = a[0][0] * b[0][0] + a[1][0] * b[0][1] + a[2][0] * b[0][2] + a[3][0] * b[0][3];
  c[0][1] = a[0][1] * b[0][0] + a[1][1] * b[0][1] + a[2][1] * b[0][2] + a[3][1] * b[0][3];
  c[0][2] = a[0][2] * b[0][0] + a[1][2] * b[0][1] + a[2][2] * b[0][2] + a[3][2] * b[0][3];
  c[0][3] = a[0][3] * b[0][0] + a[1][3] * b[0][1] + a[2][3] * b[0][2] + a[3][3] * b[0][3];
  c[1][0] = a[0][0] * b[1][0] + a[1][0] * b[1][1] + a[2][0] * b[1][2] + a[3][0] * b[1][3];
  c[1][1] = a[0][1] * b[1][0] + a[1][1] * b[1][1] + a[2][1] * b[1][2] + a[3][1] * b[1][3];
  c[1][2] = a[0][2] * b[1][0] + a[1][2] * b[1][1] + a[2][2] * b[1][2] + a[3][2] * b[1][3];
  c[1][3] = a[0][3] * b[1][0] + a[1][3] * b[1][1] + a[2][3] * b[1][2] + a[3][3] * b[1][3];
  c[2][0] = a[0][0] * b[2][0] + a[1][0] * b[2][1] + a[2][0] * b[2][2] + a[3][0] * b[2][3];
  c[2][1] = a[0][1] * b[2][0] + a[1][1] * b[2][1] + a[2][1] * b[2][2] + a[3][1] * b[2][3];
  c[2][2] = a[0][2] * b[2][0] + a[1][2] * b[2][1] + a[2][2] * b[2][2] + a[3][2] * b[2][3];
  c[2][3] = a[0][3] * b[2][0] + a[1][3] * b[2][1] + a[2][3] * b[2][2] + a[3][3] * b[2][3];
  c[3][0] = a[0][0] * b[3][0] + a[1][0] * b[3][1] + a[2][0] * b[3][2] + a[3][0] * b[3][3];
  c[3][1] = a[0][1] * b[3][0] + a[1][1] * b[3][1] + a[2][1] * b[3][2] + a[3][1] * b[3][3];
  c[3][2] = a[0][2] * b[3][0] + a[1][2] * b[3][1] + a[2][2] * b[3][2] + a[3][2] * b[3][3];
  c[3][3] = a[0][3] * b[3][0] + a[1][3] * b[3][1] + a[2][3] * b[3][2] + a[3][3] * b[3][3];

  return mat_c;
}

}  // anonymous namespace

namespace franka {

Frame operator++(Frame& frame, int /* dummy */) noexcept {
  Frame original = frame;
  frame = static_cast<Frame>(static_cast<std::underlying_type_t<Frame>>(frame) + 1);
  return original;
}

Model::Model(Network& network) : library_{new ModelLibrary(network)} {}

// Has to be declared here, as the ModelLibrary type is incomplete in the header
Model::~Model() noexcept = default;
Model::Model(Model&&) noexcept = default;
Model& Model::operator=(Model&&) noexcept = default;

std::array<double, 16> Model::pose(Frame frame, const franka::RobotState& robot_state) const {
  std::array<double, 16> output;
  switch (frame) {
    case Frame::kJoint1:
      library_->joint1(robot_state.q.data(), output.data());
      break;
    case Frame::kJoint2:
      library_->joint2(robot_state.q.data(), output.data());
      break;
    case Frame::kJoint3:
      library_->joint3(robot_state.q.data(), output.data());
      break;
    case Frame::kJoint4:
      library_->joint4(robot_state.q.data(), output.data());
      break;
    case Frame::kJoint5:
      library_->joint5(robot_state.q.data(), output.data());
      break;
    case Frame::kJoint6:
      library_->joint6(robot_state.q.data(), output.data());
      break;
    case Frame::kJoint7:
      library_->joint7(robot_state.q.data(), output.data());
      break;
    case Frame::kFlange:
      library_->flange(robot_state.q.data(), output.data());
      break;
    case Frame::kEndEffector:
      library_->ee(robot_state.q.data(), robot_state.F_T_EE.data(), output.data());
      break;
    case Frame::kStiffness:
      library_->ee(robot_state.q.data(), matMul(robot_state.F_T_EE, robot_state.EE_T_K).data(),
                   output.data());
      break;
    default:
      throw std::invalid_argument("Invalid frame given.");
  }

  return output;
}

std::array<double, 42> Model::bodyJacobian(Frame frame,
                                           const franka::RobotState& robot_state) const {
  std::array<double, 42> output;
  switch (frame) {
    case Frame::kJoint1:
      library_->body_jacobian_joint1(output.data());
      break;
    case Frame::kJoint2:
      library_->body_jacobian_joint2(robot_state.q.data(), output.data());
      break;
    case Frame::kJoint3:
      library_->body_jacobian_joint3(robot_state.q.data(), output.data());
      break;
    case Frame::kJoint4:
      library_->body_jacobian_joint4(robot_state.q.data(), output.data());
      break;
    case Frame::kJoint5:
      library_->body_jacobian_joint5(robot_state.q.data(), output.data());
      break;
    case Frame::kJoint6:
      library_->body_jacobian_joint6(robot_state.q.data(), output.data());
      break;
    case Frame::kJoint7:
      library_->body_jacobian_joint7(robot_state.q.data(), output.data());
      break;
    case Frame::kFlange:
      library_->body_jacobian_flange(robot_state.q.data(), output.data());
      break;
    case Frame::kEndEffector:
      library_->body_jacobian_ee(robot_state.q.data(), robot_state.F_T_EE.data(), output.data());
      break;
    case Frame::kStiffness:
      library_->body_jacobian_ee(robot_state.q.data(),
                                 matMul(robot_state.F_T_EE, robot_state.EE_T_K).data(),
                                 output.data());
      break;
    default:
      throw std::invalid_argument("Invalid frame given.");
  }

  return output;
}

std::array<double, 42> Model::zeroJacobian(Frame frame,
                                           const franka::RobotState& robot_state) const {
  std::array<double, 42> output;
  switch (frame) {
    case Frame::kJoint1:
      library_->zero_jacobian_joint1(output.data());
      break;
    case Frame::kJoint2:
      library_->zero_jacobian_joint2(robot_state.q.data(), output.data());
      break;
    case Frame::kJoint3:
      library_->zero_jacobian_joint3(robot_state.q.data(), output.data());
      break;
    case Frame::kJoint4:
      library_->zero_jacobian_joint4(robot_state.q.data(), output.data());
      break;
    case Frame::kJoint5:
      library_->zero_jacobian_joint5(robot_state.q.data(), output.data());
      break;
    case Frame::kJoint6:
      library_->zero_jacobian_joint6(robot_state.q.data(), output.data());
      break;
    case Frame::kJoint7:
      library_->zero_jacobian_joint7(robot_state.q.data(), output.data());
      break;
    case Frame::kFlange:
      library_->zero_jacobian_flange(robot_state.q.data(), output.data());
      break;
    case Frame::kEndEffector:
      library_->zero_jacobian_ee(robot_state.q.data(), robot_state.F_T_EE.data(), output.data());
      break;
    case Frame::kStiffness:
      library_->zero_jacobian_ee(robot_state.q.data(),
                                 matMul(robot_state.F_T_EE, robot_state.EE_T_K).data(),
                                 output.data());
      break;
    default:
      throw std::invalid_argument("Invalid frame given.");
  }

  return output;
}

std::array<double, 49> franka::Model::mass(
    const franka::RobotState& robot_state,
    const std::array<double, 9>& load_inertia,
    double load_mass,
    const std::array<double, 3>& F_x_Cload)  // NOLINT (readability-identifier-naming)
    const noexcept {
  std::array<double, 49> output;
  library_->mass(robot_state.q.data(), load_inertia.data(), load_mass, F_x_Cload.data(),
                 output.data());

  return output;
}

std::array<double, 7> franka::Model::coriolis(
    const franka::RobotState& robot_state,
    const std::array<double, 9>& load_inertia,
    double load_mass,
    const std::array<double, 3>& F_x_Cload)  // NOLINT (readability-identifier-naming)
    const noexcept {
  std::array<double, 7> output;
  library_->coriolis(robot_state.q.data(), robot_state.dq.data(), load_inertia.data(), load_mass,
                     F_x_Cload.data(), output.data());

  return output;
}

std::array<double, 7> franka::Model::gravity(
    const franka::RobotState& robot_state,
    double load_mass,
    const std::array<double, 3>& F_x_Cload,  // NOLINT (readability-identifier-naming)
    const std::array<double, 3>& gravity_earth) const noexcept {
  std::array<double, 7> output;
  library_->gravity(robot_state.q.data(), gravity_earth.data(), load_mass, F_x_Cload.data(),
                    output.data());

  return output;
}

}  // namespace franka
