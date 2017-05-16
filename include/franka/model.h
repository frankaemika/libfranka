#pragma once

#include <Poco/SharedLibrary.h>
#include <array>

#include <franka/robot.h>
#include "../../src/libfcimodels.h"

/**
 * @file model.h
 * Contains the Model class.
 */

namespace franka {

/**
 * Calculates poses of joints and dynamic properties of the robot.
 */
class Model {
 public:
  /**
   * Creates a new model instance given a connected robot.
   * @param robot connected robot instance.
   * @throw ModelException if the model library cannot be loaded, or if the
   * versions do not match.
   */
  Model(franka::Robot& robot);

  /**
   * Gets the 4x4 pose matrix for the given joint.
   * Pose is represented as a 4x4 matrix in column-major format.
   * @param joint number of the desired joint
   * @param robot_state state from which the pose should be calculated.
   *
   * @return 4x4 pose matrix
   */
  std::array<double, 16> jointPose(int joint,
                                   const franka::RobotState& robot_state);

  /**
   * Gets the 4x4 pose matrix for the flange.
   * Pose is represented as a 4x4 matrix in column-major format.
   * @param robot_state state from which the pose should be calculated.
   *
   * @return 4x4 pose matrix
   */
  std::array<double, 16> flangePose(const franka::RobotState& robot_state);

  /**
   * Gets the 4x4 pose matrix for the end effector.
   * Pose is represented as a 4x4 matrix in column-major format.
   * @param robot_state state from which the pose should be calculated.
   *
   * @return 4x4 pose matrix
   */
  std::array<double, 16> eePose(const franka::RobotState& robot_state);

  /**
   * Calculates the 7x7 mass matrix. Unit: \f$[kg \times m^2]\f$
   * @param robot_state state from which the pose should be calculated.
   * @param load_inertia Inertia of the load, relative to center of mass. Unit:
   * \f$[kg \times m^2]\f$
   * @param load_mass Weight of the load. Unit: \f$[kg]\f$
   * @param F_T_Cload Translation from flange to center of mass of load. Unit:
   * \f$[m]\f$
   *
   * @return 7x7 mass matrix
   */
  std::array<double, 49> mass(const franka::RobotState& robot_state,
                              const std::array<double, 7> load_inertia,
                              double load_mass,
                              std::array<double, 3> F_T_Cload);

  /**
 * Calculates the Coriolis Force vector (state-space equation): \f$ c= C \times
 * dq\f$, in \f$[Nm]\f$
 * @param robot_state state from which the pose should be calculated.
 * @param load_inertia Inertia of the load, relative to center of mass. Unit:
 * \f$[kg \times m^2]\f$
 * @param load_mass Weight of the load. Unit: \f$[kg]\f$
 * @param F_T_Cload Translation from flange to center of mass of load. Unit:
 * \f$[m]\f$
 *
 * @return Coriolis Force vector
 */
  std::array<double, 7> coriolis(const franka::RobotState& robot_state,
                                 const std::array<double, 7> load_inertia,
                                 double load_mass,
                                 std::array<double, 3> F_T_Cload);
  /**
  * Calculates the gravity vector. Unit: \f$[Nm]\f$
  * @param robot_state state from which the pose should be calculated.
  * @param load_mass Weight of the load. Unit: \f$[kg]\f$
  * @param F_T_Cload Translation from flange to center of mass of load. Unit:
  * \f$[m]\f$
  * @param gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$
  *
  * @return The gravity vector.
  */
  std::array<double, 7> gravity(const franka::RobotState& robot_state,
                                double load_mass,
                                std::array<double, 3> F_T_Cload,
                                std::array<double, 3> gravity_earth);

 private:
  Poco::SharedLibrary library_;

  void* M_NE_file_pointer_;    // NOLINT
  void* O_T_J1_file_pointer_;  // NOLINT
  void* O_T_J2_file_pointer_;  // NOLINT
  void* O_T_J3_file_pointer_;  // NOLINT
  void* O_T_J4_file_pointer_;  // NOLINT
  void* O_T_J5_file_pointer_;  // NOLINT
  void* O_T_J6_file_pointer_;  // NOLINT
  void* O_T_J7_file_pointer_;  // NOLINT
  void* O_T_J8_file_pointer_;  // NOLINT
  void* O_T_J9_file_pointer_;  // NOLINT
  void* c_NE_file_pointer_;    // NOLINT
  void* g_NE_file_pointer_;    // NOLINT
};

}  // namespace franka