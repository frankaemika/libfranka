#include <franka/model.h>

#include <sstream>

#include <research_interface/service_types.h>

#include "model_library.h"

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

Model::Model(
    franka::Robot& /* robot */)  // NOLINT (readability-named-parameter)
    : library_{new ModelLibrary} {}

// Has to be declared here, as the ModelLibrary type is incomplete in the
// header
Model::~Model() noexcept = default;

std::array<double, 16> Model::jointPose(
    Frame joint,
    const franka::RobotState& robot_state) const {
  std::array<double, 16> output;

  std::array<double, 7>::const_pointer q = robot_state.q.data();
  std::array<double, 16>::const_pointer end_effector =
      robot_state.O_T_EE.data();

  switch (joint) {
    case Frame::kJoint1:
      library_->joint1(q, end_effector, output.data());
      break;
    case Frame::kJoint2:
      library_->joint2(q, end_effector, output.data());
      break;
    case Frame::kJoint3:
      library_->joint3(q, end_effector, output.data());
      break;
    case Frame::kJoint4:
      library_->joint4(q, end_effector, output.data());
      break;
    case Frame::kJoint5:
      library_->joint5(q, end_effector, output.data());
      break;
    case Frame::kJoint6:
      library_->joint6(q, end_effector, output.data());
      break;
    case Frame::kJoint7:
      library_->joint7(q, end_effector, output.data());
      break;
    case Frame::kFlange:
      library_->flange(q, end_effector, output.data());
      break;
    case Frame::kEndEffector:
      library_->ee(q, end_effector, output.data());
      break;
    default:
      throw std::invalid_argument("Invalid joint given.");
  }

  return output;
}

std::array<double, 49> franka::Model::mass(
    const franka::RobotState& robot_state,
    const std::array<double, 9>& load_inertia,
    double load_mass,
    const std::array<double, 3>&
        F_x_Cload)  // NOLINT (readability-identifier-naming)
    const noexcept {
  std::array<double, 49> output;
  library_->mass(robot_state.q.data(), load_inertia.data(), load_mass,
                 F_x_Cload.data(), output.data());

  return output;
}

std::array<double, 7> franka::Model::coriolis(
    const franka::RobotState& robot_state,
    const std::array<double, 9>& load_inertia,
    double load_mass,
    const std::array<double, 3>&
        F_x_Cload)  // NOLINT (readability-identifier-naming)
    const noexcept {
  std::array<double, 7> output;
  library_->coriolis(robot_state.q.data(), robot_state.dq.data(),
                     load_inertia.data(), load_mass, F_x_Cload.data(),
                     output.data());

  return output;
}

std::array<double, 7> franka::Model::gravity(
    const franka::RobotState& robot_state,
    double load_mass,
    const std::array<double, 3>&
        F_x_Cload,  // NOLINT (readability-identifier-naming)
    const std::array<double, 3>& gravity_earth) const noexcept {
  std::array<double, 7> output;
  library_->gravity(robot_state.q.data(), gravity_earth.data(), load_mass,
                    F_x_Cload.data(), output.data());

  return output;
}

}  // namespace franka
