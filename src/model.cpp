#include <sstream>

#include <franka/model.h>
#include <research_interface/service_types.h>

#include "libfcimodels.h"

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

franka::Model::Model(franka::Robot& robot) try {
  library_.load(std::string{"libfcimodels"} + Poco::SharedLibrary::suffix());

  mass_function_ = library_.getSymbol("M_NE_file");
  joint0_function_ = library_.getSymbol("O_T_J1_file");
  joint1_function_ = library_.getSymbol("O_T_J2_file");
  joint2_function_ = library_.getSymbol("O_T_J3_file");
  joint3_function_ = library_.getSymbol("O_T_J4_file");
  joint4_function_ = library_.getSymbol("O_T_J5_file");
  joint5_function_ = library_.getSymbol("O_T_J6_file");
  joint6_function_ = library_.getSymbol("O_T_J7_file");
  flange_function_ = library_.getSymbol("O_T_J8_file");
  ee_function_ = library_.getSymbol("O_T_J9_file");
  coriolis_function_ = library_.getSymbol("c_NE_file");
  gravity_function_ = library_.getSymbol("g_NE_file");

  std::stringstream loaded_robot_revision;
  loaded_robot_revision << robotType() << "_";
  if (robotId() != "") {
    loaded_robot_revision << robotId() << "_";
  }
  loaded_robot_revision << "v" << versionMajor() << "." << versionMinor() << "."
                        << versionPatch();

  if (loaded_robot_revision.str() != robot.robotRevision()) {
    throw ModelLibraryException("Robot version mismatch - loaded: "s +
                                loaded_robot_revision.str() + ", expected: " +
                                robot.robotRevision());
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

franka::Model::~Model() noexcept try { library_.unload(); } catch (...) {
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
      reinterpret_cast<decltype(&O_T_J1_file)>(joint0_function_)(
          q, end_effector, output.data());
      break;
    case Joint::kJ1:
      reinterpret_cast<decltype(&O_T_J2_file)>(joint1_function_)(
          q, end_effector, output.data());
      break;
    case Joint::kJ2:
      reinterpret_cast<decltype(&O_T_J3_file)>(joint2_function_)(
          q, end_effector, output.data());
      break;
    case Joint::kJ3:
      reinterpret_cast<decltype(&O_T_J4_file)>(joint3_function_)(
          q, end_effector, output.data());
      break;
    case Joint::kJ4:
      reinterpret_cast<decltype(&O_T_J5_file)>(joint4_function_)(
          q, end_effector, output.data());
      break;
    case Joint::kJ5:
      reinterpret_cast<decltype(&O_T_J6_file)>(joint5_function_)(
          q, end_effector, output.data());
      break;
    case Joint::kJ6:
      reinterpret_cast<decltype(&O_T_J7_file)>(joint6_function_)(
          q, end_effector, output.data());
      break;
    case Joint::kFlange:
      reinterpret_cast<decltype(&O_T_J8_file)>(flange_function_)(
          q, end_effector, output.data());
      break;
    case Joint::kEndEffector:
      reinterpret_cast<decltype(&O_T_J9_file)>(ee_function_)(q, end_effector,
                                                             output.data());
      break;
    default:
      throw std::invalid_argument("Invalid joint given.");
  }

  return output;
}

std::array<double, 49> franka::Model::mass(
    const franka::RobotState& robot_state,
    const std::array<double, 7> load_inertia,
    double load_mass,
    std::array<double, 3> F_x_Cload) const noexcept {
  std::array<double, 49> output;
  auto function = reinterpret_cast<decltype(&M_NE_file)>(mass_function_);
  function(robot_state.q.data(), load_inertia.data(), load_mass,
           F_x_Cload.data(), output.data());

  return output;
}

std::array<double, 7> franka::Model::coriolis(
    const franka::RobotState& robot_state,
    const std::array<double, 7> load_inertia,
    double load_mass,
    std::array<double, 3> F_x_Cload) const noexcept {
  std::array<double, 7> output;
  auto function = reinterpret_cast<decltype(&c_NE_file)>(coriolis_function_);
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
  auto function = reinterpret_cast<decltype(&g_NE_file)>(gravity_function_);
  function(robot_state.q.data(), gravity_earth.data(), load_mass,
           F_x_Cload.data(), output.data());

  return output;
}
