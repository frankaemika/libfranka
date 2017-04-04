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
};

struct JointValues {

};

struct JointVelocities {

};

struct CartesianPose {

};

struct CartesianVelocities {

};

struct Stop : Torques, JointValues, JointVelocities, CartesianPose, CartesianVelocities {
};

}  // namespace franka
