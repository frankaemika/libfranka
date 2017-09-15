// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <functional>

#include "libfcimodels.h"
#include "library_loader.h"
#include "network.h"

namespace franka {

class ModelLibrary {
 public:
  ModelLibrary(Network& network);

 private:
  LibraryLoader loader_;

 public:
  const std::function<decltype(Ji_J_J1)> body_jacobian_joint1;
  const std::function<decltype(Ji_J_J2)> body_jacobian_joint2;
  const std::function<decltype(Ji_J_J3)> body_jacobian_joint3;
  const std::function<decltype(Ji_J_J4)> body_jacobian_joint4;
  const std::function<decltype(Ji_J_J5)> body_jacobian_joint5;
  const std::function<decltype(Ji_J_J6)> body_jacobian_joint6;
  const std::function<decltype(Ji_J_J7)> body_jacobian_joint7;
  const std::function<decltype(Ji_J_J8)> body_jacobian_flange;
  const std::function<decltype(Ji_J_J9)> body_jacobian_ee;

  const std::function<decltype(M_NE)> mass;

  const std::function<decltype(O_J_J1)> zero_jacobian_joint1;
  const std::function<decltype(O_J_J2)> zero_jacobian_joint2;
  const std::function<decltype(O_J_J3)> zero_jacobian_joint3;
  const std::function<decltype(O_J_J4)> zero_jacobian_joint4;
  const std::function<decltype(O_J_J5)> zero_jacobian_joint5;
  const std::function<decltype(O_J_J6)> zero_jacobian_joint6;
  const std::function<decltype(O_J_J7)> zero_jacobian_joint7;
  const std::function<decltype(O_J_J8)> zero_jacobian_flange;
  const std::function<decltype(O_J_J9)> zero_jacobian_ee;

  const std::function<decltype(O_T_J1)> joint1;
  const std::function<decltype(O_T_J2)> joint2;
  const std::function<decltype(O_T_J3)> joint3;
  const std::function<decltype(O_T_J4)> joint4;
  const std::function<decltype(O_T_J5)> joint5;
  const std::function<decltype(O_T_J6)> joint6;
  const std::function<decltype(O_T_J7)> joint7;
  const std::function<decltype(O_T_J8)> flange;
  const std::function<decltype(O_T_J9)> ee;

  const std::function<decltype(c_NE)> coriolis;
  const std::function<decltype(g_NE)> gravity;
};

}  // namespace franka
