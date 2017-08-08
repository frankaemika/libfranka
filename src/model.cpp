#include <franka/model.h>

#include <sstream>

#include <research_interface/robot/service_types.h>

#include "model_library.h"
#include "network.h"

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

Model::Model(Network& network) : library_{new ModelLibrary(network)} {}

// Has to be declared here, as the ModelLibrary type is incomplete in the header
Model::~Model() noexcept = default;
Model::Model(Model&&) noexcept = default;
Model& Model::operator=(Model&&) noexcept = default;

std::array<double, 16> Model::jointPose(Frame frame, const franka::RobotState& robot_state) const {
  std::array<double, 16> output;

  std::array<double, 7>::const_pointer q = robot_state.q.data();

  switch (frame) {
    case Frame::kJoint1:
      library_->joint1(q, output.data());
      break;
    case Frame::kJoint2:
      library_->joint2(q, output.data());
      break;
    case Frame::kJoint3:
      library_->joint3(q, output.data());
      break;
    case Frame::kJoint4:
      library_->joint4(q, output.data());
      break;
    case Frame::kJoint5:
      library_->joint5(q, output.data());
      break;
    case Frame::kJoint6:
      library_->joint6(q, output.data());
      break;
    case Frame::kJoint7:
      library_->joint7(q, output.data());
      break;
    case Frame::kFlange:
      library_->flange(q, output.data());
      break;
    case Frame::kEndEffector:
      library_->ee(q, robot_state.O_T_EE.data(), output.data());
      break;
    default:
      throw std::invalid_argument("Invalid frame given.");
  }

  return output;
}

std::array<double, 42> Model::bodyJacobian(Frame frame,
                                           const franka::RobotState& robot_state) const {
  std::array<double, 42> output;

  std::array<double, 7>::const_pointer q = robot_state.q.data();

  switch (frame) {
    case Frame::kJoint1:
      library_->body_jacobian_joint1(output.data());
      break;
    case Frame::kJoint2:
      library_->body_jacobian_joint2(q, output.data());
      break;
    case Frame::kJoint3:
      library_->body_jacobian_joint3(q, output.data());
      break;
    case Frame::kJoint4:
      library_->body_jacobian_joint4(q, output.data());
      break;
    case Frame::kJoint5:
      library_->body_jacobian_joint5(q, output.data());
      break;
    case Frame::kJoint6:
      library_->body_jacobian_joint6(q, output.data());
      break;
    case Frame::kJoint7:
      library_->body_jacobian_joint7(q, output.data());
      break;
    case Frame::kFlange:
      library_->body_jacobian_flange(q, output.data());
      break;
    case Frame::kEndEffector:
      library_->body_jacobian_ee(q, robot_state.O_T_EE.data(), output.data());
      break;
    default:
      throw std::invalid_argument("Invalid frame given.");
  }

  return output;
}

std::array<double, 42> Model::zeroJacobian(Frame frame,
                                           const franka::RobotState& robot_state) const {
  std::array<double, 42> output;

  std::array<double, 7>::const_pointer q = robot_state.q.data();

  switch (frame) {
    case Frame::kJoint1:
      library_->zero_jacobian_joint1(output.data());
      break;
    case Frame::kJoint2:
      library_->zero_jacobian_joint2(q, output.data());
      break;
    case Frame::kJoint3:
      library_->zero_jacobian_joint3(q, output.data());
      break;
    case Frame::kJoint4:
      library_->zero_jacobian_joint4(q, output.data());
      break;
    case Frame::kJoint5:
      library_->zero_jacobian_joint5(q, output.data());
      break;
    case Frame::kJoint6:
      library_->zero_jacobian_joint6(q, output.data());
      break;
    case Frame::kJoint7:
      library_->zero_jacobian_joint7(q, output.data());
      break;
    case Frame::kFlange:
      library_->zero_jacobian_flange(q, output.data());
      break;
    case Frame::kEndEffector:
      library_->zero_jacobian_ee(q, robot_state.O_T_EE.data(), output.data());
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
