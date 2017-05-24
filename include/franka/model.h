#pragma once

#include <array>

#include <franka/robot.h>

/**
 * @file model.h
 * Contains the Model class and the Joint enum.
 */

namespace franka {

/**
 * Enumerates FRANKA's seven joints, the flange and the end effector.
 */
enum Frame : uint8_t {
  kJoint1,
  kJoint2,
  kJoint3,
  kJoint4,
  kJoint5,
  kJoint6,
  kJoint7,
  kFlange,
  kEndEffector
};

class LibraryLoader;

/**
 * Calculates poses of joints and dynamic properties of the robot.
 */
class Model {
 public:
  /**
   * Creates a new model for a given franka::Robot.
   *
   * @param[in] robot Robot to create model for.
   *
   * @throw ModelException if the model library cannot be loaded.
   */
  Model(franka::Robot& robot);

  /**
   * Unloads the model library.
   */
  ~Model() noexcept;

  /**
   * Gets the 4x4 pose matrix for the given joint in world coordinates.
   *
   * The pose is represented as a 4x4 matrix in column-major format.
   *
   * @param[in] joint number of the desired joint.
   * @param[in] robot_state state from which the pose should be calculated.
   *
   * @return Vectorized 4x4 pose matrix, column-major.
   */
  std::array<double, 16> jointPose(Frame joint,
                                   const franka::RobotState& robot_state) const;

  /**
   * Calculates the 7x7 mass matrix. Unit: \f$[kg \times m^2]\f$.
   *
   * @param[in] robot_state State from which the pose should be calculated.
   * @param[in] load_inertia Inertia of the load, relative to center of mass,
   * given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
   * @param[in] load_mass Weight of the load. Unit: \f$[kg]\f$.
   * @param[in] F_x_Cload Translation from flange to center of mass of load.
   * Unit: \f$[m]\f$.
   *
   * @return Vectorized 7x7 mass matrix, column-major.
   */
  std::array<double, 49> mass(
      const franka::RobotState& robot_state,
      const std::array<double, 9>& load_inertia,
      double load_mass,
      const std::array<double, 3>&
          F_x_Cload)  // NOLINT (readability-identifier-naming)
      const noexcept;

  /**
 * Calculates the Coriolis force vector (state-space equation): \f$ c= C \times
 * dq\f$, in \f$[Nm]\f$.
 *
 * @param[in] robot_state State from which the Coriolis force vector should be
 * calculated.
 * @param[in] load_inertia Inertia of the load, relative to center of mass,
 * given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
 * @param[in] load_mass Weight of the load. Unit: \f$[kg]\f$.
 * @param[in] F_x_Cload Translation from flange to center of mass of load.
 * Unit: \f$[m]\f$.
 *
 * @return Coriolis force vector.
 */
  std::array<double, 7> coriolis(
      const franka::RobotState& robot_state,
      const std::array<double, 9>& load_inertia,
      double load_mass,
      const std::array<double, 3>&
          F_x_Cload)  // NOLINT (readability-identifier-naming)
      const noexcept;

  /**
  * Calculates the gravity vector. Unit: \f$[Nm]\f$.
  *
  * @param[in] robot_state State from which the gravity vector should be
  * calculated.
  * @param[in] load_mass Weight of the load. Unit: \f$[kg]\f$.
  * @param[in] F_x_Cload Translation from flange to center of mass of load.
  * Unit: \f$[m]\f$.
  * @param[in] gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$.
  * Default to {0.0, 0.0, -9.81}.
  *
  * @return Gravity vector.
  */
  std::array<double, 7> gravity(
      const franka::RobotState& robot_state,
      double load_mass,
      const std::array<double, 3>&
          F_x_Cload,  // NOLINT (readability-identifier-naming)
      const std::array<double, 3>& gravity_earth = {{0., 0., -9.81}}) const
      noexcept;

 private:
  std::unique_ptr<LibraryLoader> library_;

  void* mass_function_;
  void* joint0_function_;
  void* joint1_function_;
  void* joint2_function_;
  void* joint3_function_;
  void* joint4_function_;
  void* joint5_function_;
  void* joint6_function_;
  void* flange_function_;
  void* ee_function_;
  void* coriolis_function_;
  void* gravity_function_;
};

}  // namespace franka
