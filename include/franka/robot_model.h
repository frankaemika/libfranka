// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <string>

#include "franka/robot_model_base.h"

namespace franka {

/**
 * Implements RobotModelBase using Pinocchio.
 */
class RobotModel : public RobotModelBase {
 public:
  RobotModel(const std::string& urdf);
  void coriolis(const std::array<double, 7>& q,
                const std::array<double, 7>& dq,
                const std::array<double, 9>& i_total,
                double m_total,
                const std::array<double, 3>& f_x_ctotal,
                std::array<double, 7>& c_ne) override;
  void gravity(const std::array<double, 7>& q,
               const std::array<double, 3>& g_earth,
               double m_total,
               const std::array<double, 3>& f_x_ctotal,
               std::array<double, 7>& g_ne) override;
  void mass(const std::array<double, 7>& q,
            const std::array<double, 9>& i_total,
            double m_total,
            const std::array<double, 3>& f_x_ctotal,
            std::array<double, 49>& m_ne) override;

 private:
  /**
   * @brief Adds the inertia of the attached total load including end effector to the last link of
   * the robot.
   */
  void addInertiaToLastLink(const std::array<double, 9>& i_total,
                            double m_total,
                            const std::array<double, 3>& f_x_ctotal);
  /**
   * @brief Computes the dynamics of the robot using the given function.
   *        Adds the inertia of the load and restores it after the computation.
   */
  void computeDynamics(
      const std::array<double, 9>& i_total,
      double m_total,
      const std::array<double, 3>& f_x_ctotal,
      pinocchio::Data& data,
      const std::function<void(pinocchio::Model&, pinocchio::Data&)>& compute_func);

  pinocchio::Model pinocchio_model_;
  pinocchio::Inertia initial_last_link_inertia_;
  pinocchio::FrameIndex last_link_frame_index_;
  pinocchio::JointIndex last_joint_index_;
};

}  // namespace franka
