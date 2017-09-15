// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>

#include <franka/robot.h>
#include <franka/robot_state.h>

/**
 * @file model.h
 * Contains model library types.
 */

namespace franka {

/**
 * Enumerates the seven joints, the flange, and the end effector of a robot.
 */
enum class Frame {
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

/**
 * Post-increments the given Frame by one.
 *
 * For example, Frame::kJoint2++ results in Frame::kJoint3.
 *
 * @param[in] frame Frame to increment.
 *
 * @return Original Frame.
 */
Frame operator++(Frame& frame, int /* dummy */) noexcept;

class ModelLibrary;
class Network;

/**
 * Calculates poses of joints and dynamic properties of the robot.
 */
class Model {
 public:
  /**
   * Creates a new Model instance.
   *
   * This constructor is for internal use only.
   *
   * @see Robot::loadModel
   *
   * @param[in] network For internal use.
   *
   * @throw ModelException if the model library cannot be loaded.
   */
  explicit Model(franka::Network& network);

  /**
   * Move-constructs a new Model instance.
   *
   * @param[in] model Other Model instance.
   */
  Model(Model&& model) noexcept;

  /**
   * Move-assigns this Model from another Model instance.
   *
   * @param[in] model Other Model instance.
   *
   * @return Model instance.
   */
  Model& operator=(Model&& model) noexcept;

  /**
   * Unloads the model library.
   */
  ~Model() noexcept;

  /**
   * Gets the 4x4 pose matrix for the given frame in base frame.
   *
   * The pose is represented as a 4x4 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 4x4 pose matrix, column-major.
   */
  std::array<double, 16> pose(Frame frame, const franka::RobotState& robot_state) const;

  /**
   * Gets the 6x7 Jacobian for the given frame, relative to that frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  std::array<double, 42> bodyJacobian(Frame frame, const franka::RobotState& robot_state) const;

  /**
   * Gets the 6x7 Jacobian for the given joint relative to the base frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  std::array<double, 42> zeroJacobian(Frame frame, const franka::RobotState& robot_state) const;

  /**
   * Calculates the 7x7 mass matrix. Unit: \f$[kg \times m^2]\f$.
   *
   * @param[in] robot_state State from which the pose should be calculated.
   * @param[in] load_inertia Inertia of the load, relative to center of mass, given as vectorized
   * 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
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
      const std::array<double, 3>& F_x_Cload)  // NOLINT (readability-identifier-naming)
      const noexcept;

  /**
   * Calculates the Coriolis force vector (state-space equation): \f$ c= C \times
   * dq\f$, in \f$[Nm]\f$.
   *
   * @param[in] robot_state State from which the Coriolis force vector should be calculated.
   * @param[in] load_inertia Inertia of the load, relative to center of mass, given as vectorized
   * 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
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
      const std::array<double, 3>& F_x_Cload)  // NOLINT (readability-identifier-naming)
      const noexcept;

  /**
   * Calculates the gravity vector. Unit: \f$[Nm]\f$.
   *
   * @param[in] robot_state State from which the gravity vector should be calculated.
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
      const std::array<double, 3>& F_x_Cload,  // NOLINT (readability-identifier-naming)
      const std::array<double, 3>& gravity_earth = {{0., 0., -9.81}}) const noexcept;

  Model(const Model&) = delete;
  Model& operator=(const Model&) = delete;

 private:
  std::unique_ptr<ModelLibrary> library_;
};

}  // namespace franka
