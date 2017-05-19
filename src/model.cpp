#include <sstream>

#include <franka/model.h>
#include <research_interface/service_types.h>

#include "libfcimodels.h"

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

franka::Model::Model(franka::Robot& robot) {
  try {
    library_.load(std::string{"libfcimodels"} + Poco::SharedLibrary::suffix());

    M_NE_file_pointer_ = library_.getSymbol("M_NE_file");
    O_T_J1_file_pointer_ = library_.getSymbol("O_T_J1_file");
    O_T_J2_file_pointer_ = library_.getSymbol("O_T_J2_file");
    O_T_J3_file_pointer_ = library_.getSymbol("O_T_J3_file");
    O_T_J4_file_pointer_ = library_.getSymbol("O_T_J4_file");
    O_T_J5_file_pointer_ = library_.getSymbol("O_T_J5_file");
    O_T_J6_file_pointer_ = library_.getSymbol("O_T_J6_file");
    O_T_J7_file_pointer_ = library_.getSymbol("O_T_J7_file");
    O_T_J8_file_pointer_ = library_.getSymbol("O_T_J8_file");
    O_T_J9_file_pointer_ = library_.getSymbol("O_T_J9_file");
    c_NE_file_pointer_ = library_.getSymbol("c_NE_file");
    g_NE_file_pointer_ = library_.getSymbol("g_NE_file");

    std::stringstream loaded_robot_version;
    loaded_robot_version << robotType() << "_";
    if (robotId() != "") {
      loaded_robot_version << robotId() << "_";
    }
    loaded_robot_version << "v" << versionMajor() << "." << versionMinor()
                         << "." << versionPatch();

    if (loaded_robot_version.str() != robot.robotVersion()) {
      throw ModelLibraryException("Robot version mismatch - loaded: "s +
                                  loaded_robot_version.str() + ", expected: " +
                                  robot.robotVersion());
    }
  } catch (const Poco::LibraryAlreadyLoadedException& e) {
    throw ModelLibraryException("libfranka: model library already loaded"s);
  } catch (const Poco::LibraryLoadException& e) {
    throw ModelLibraryException("libfranka: cannot load model library: "s +
                                e.what());
  } catch (const Poco::NotFoundException& e) {
    throw ModelLibraryException("libfranka: symbol cannot be found: "s +
                                e.what());
  } catch (const Poco::Exception& e) {
    throw ModelLibraryException("libfranka: error while loading library: "s +
                                e.what());
  }
}

franka::Model::~Model() {
  try {
    library_.unload();
  } catch (...) {
  }
}

std::array<double, 16> franka::Model::jointPose(
    Joint joint,
    const franka::RobotState& robot_state) const {
  std::array<double, 16> output;

  std::array<double, 7>::const_pointer q = robot_state.q.data();
  std::array<double, 16>::const_pointer end_effector =
      robot_state.O_T_EE.data();

  switch (joint) {
    case Joint::kJ0:
      reinterpret_cast<decltype(&O_T_J1_file)>(O_T_J1_file_pointer_)(
          q, end_effector, output.data());
      break;
    case Joint::kJ1:
      reinterpret_cast<decltype(&O_T_J2_file)>(O_T_J2_file_pointer_)(
          q, end_effector, output.data());
      break;
    case Joint::kJ2:
      reinterpret_cast<decltype(&O_T_J3_file)>(O_T_J3_file_pointer_)(
          q, end_effector, output.data());
      break;
    case Joint::kJ3:
      reinterpret_cast<decltype(&O_T_J4_file)>(O_T_J4_file_pointer_)(
          q, end_effector, output.data());
      break;
    case Joint::kJ4:
      reinterpret_cast<decltype(&O_T_J5_file)>(O_T_J5_file_pointer_)(
          q, end_effector, output.data());
      break;
    case Joint::kJ5:
      reinterpret_cast<decltype(&O_T_J6_file)>(O_T_J6_file_pointer_)(
          q, end_effector, output.data());
      break;
    case Joint::kJ6:
      reinterpret_cast<decltype(&O_T_J7_file)>(O_T_J7_file_pointer_)(
          q, end_effector, output.data());
      break;
    case Joint::kFlange:
      reinterpret_cast<decltype(&O_T_J8_file)>(O_T_J8_file_pointer_)(
          q, end_effector, output.data());
      break;
    case Joint::kEndEffector:
      reinterpret_cast<decltype(&O_T_J9_file)>(O_T_J9_file_pointer_)(
          q, end_effector, output.data());
      break;
    default:
      throw std::invalid_argument("joint number must be between 0 and 8");
  }

  return output;
}

std::array<double, 49> franka::Model::mass(
    const franka::RobotState& robot_state,
    const std::array<double, 7> load_inertia,
    double load_mass,
    std::array<double, 3> F_T_Cload) const noexcept {
  std::array<double, 49> output;
  auto function = reinterpret_cast<decltype(&M_NE_file)>(M_NE_file_pointer_);
  function(robot_state.q.data(), load_inertia.data(), load_mass,
           F_T_Cload.data(), output.data());

  return output;
}

std::array<double, 7> franka::Model::coriolis(
    const franka::RobotState& robot_state,
    const std::array<double, 7> load_inertia,
    double load_mass,
    std::array<double, 3> F_x_Cload) const noexcept {
  std::array<double, 7> output;
  auto function = reinterpret_cast<decltype(&c_NE_file)>(c_NE_file_pointer_);
  function(robot_state.q.data(), robot_state.dq.data(), load_inertia.data(),
           load_mass, F_x_Cload.data(), output.data());

  return output;
}
std::array<double, 7> franka::Model::gravity(
    const franka::RobotState& robot_state,
    double load_mass,
    std::array<double, 3> F_x_Cload,
    std::array<double, 3> gravity_earth) const noexcept {
  std::array<double, 7> output;
  auto function = reinterpret_cast<decltype(&g_NE_file)>(g_NE_file_pointer_);
  function(robot_state.q.data(), gravity_earth.data(), load_mass,
           F_x_Cload.data(), output.data());

  return output;
}
