#pragma once

#include <array>
#include <initializer_list>

namespace franka {

struct Torques {
  Torques(std::array<double, 7> torques);

  /**
   * @throw std::invalid_argument Wrong size of torques.
   */
  Torques(std::initializer_list<double> torques);

  std::array<double, 7> tau_J;

 protected:
  Torques() = default;
};

struct JointValues {
  JointValues(std::array<double, 7> joint_values);

  /**
   * @throw std::invalid_argument Wrong size of joint_values.
   */
  JointValues(std::initializer_list<double> joint_values);

  /*
   * Desired joint angles in [rad]
   */
  std::array<double, 7> q;

 protected:
  JointValues() = default;
};

struct JointVelocities {
  JointVelocities(std::array<double, 7> joint_velocities);

  /**
   * @throw std::invalid_argument Wrong size of joint_velocities.
   */
  JointVelocities(std::initializer_list<double> joint_velocities);

  /*
   * Desired joint velocities in [rad/s]
   */
  std::array<double, 7> dq;

 protected:
  JointVelocities() = default;
};

struct CartesianPose {
  CartesianPose(std::array<double, 16> pose);

  /**
   * @throw std::invalid_argument Wrong size of pose.
   */
  CartesianPose(std::initializer_list<double> pose);

  /*
   * Homogeneous transformation \f${}_O \mathbf{T}_{EE,d}\f$, column major, that transforms from the end-effector frame \f$EE\f$ to base frame \f$O\f$
   */
  std::array<double, 16> O_T_EE;

 protected:
  CartesianPose() = default;
};

struct CartesianVelocities {
  CartesianVelocities(std::array<double, 6> velocities);

  /**
   * @throw std::invalid_argument Wrong size of velocities.
   */
  CartesianVelocities(std::initializer_list<double> velocities);

  /*
   * Desired Cartesian velocity w.r.t. O-frame {dx in [m/s], dx in [m/s], dz in [m/s], omegax in [rad/s], omegay in [rad/s], omegaz in [rad/s]}
   */
  std::array<double, 6> O_dP_EE;

 protected:
  CartesianVelocities() = default;
};

namespace {
  struct StopType : Torques, JointValues, JointVelocities, CartesianPose, CartesianVelocities {
    StopType() = default;
  };
}  // anonymous namespace

static constexpr StopType Stop = StopType();

}  // namespace franka
