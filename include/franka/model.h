#pragma once

#include <array>

#include <Poco/SharedLibrary.h>

#include <franka/robot.h>

/**
 * @file model.h
 * Contains the Model class and the joint enum.
 */

namespace franka {

/**
 * Enumerates the seven Franka's joints, the Flange and the End Effector.
 */
enum Joint : uint8_t { J0, J1, J2, J3, J4, J5, J6, FLANGE, EE };

/**
 * Calculates poses of joints and dynamic properties of the robot.
 */
class Model {
 public:
  /**
   * Creates a new model instance given a connected robot.
   * @param robot connected robot instance.
   * @throw ModelLibraryException if the model library cannot be loaded, or if
   * the
   * versions do not match.
   */
  Model(franka::Robot& robot);
  ~Model();

  /**
   * Gets the 4x4 pose matrix for the given joint.
   * Pose is represented as a 4x4 matrix in column-major format.
   * @param joint number of the desired joint
   * @param robot_state state from which the pose should be calculated.
   *
   * @return 4x4 pose matrix
   */
  std::array<double, 16> jointPose(Joint joint,
                                   const franka::RobotState& robot_state) const
      noexcept;

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
                              std::array<double, 3> F_T_Cload) const noexcept;

  /**
 * Calculates the Coriolis Force vector (state-space equation): \f$ c= C \times
 * dq\f$, in \f$[Nm]\f$
 * @param robot_state state from which the pose should be calculated.
 * @param load_inertia Inertia of the load, relative to center of mass. Unit:
 * \f$[kg \times m^2]\f$
 * @param load_mass Weight of the load. Unit: \f$[kg]\f$
 * @param F_x_Cload Translation from flange to center of mass of load. Unit:
 * \f$[m]\f$
 *
 * @return Coriolis Force vector
 */
  std::array<double, 7> coriolis(const franka::RobotState& robot_state,
                                 const std::array<double, 7> load_inertia,
                                 double load_mass,
                                 std::array<double, 3> F_x_Cload) const
      noexcept;
  /**
  * Calculates the gravity vector. Unit: \f$[Nm]\f$
  * @param robot_state state from which the pose should be calculated.
  * @param load_mass Weight of the load. Unit: \f$[kg]\f$
  * @param F_x_Cload Translation from flange to center of mass of load. Unit:
  * \f$[m]\f$
  * @param gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$
  *
  * @return The gravity vector.
  */
  std::array<double, 7> gravity(const franka::RobotState& robot_state,
                                double load_mass,
                                std::array<double, 3> F_x_Cload,
                                std::array<double, 3> gravity_earth) const
      noexcept;

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