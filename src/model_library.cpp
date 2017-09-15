// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "model_library.h"

#include "library_downloader.h"

namespace franka {

ModelLibrary::ModelLibrary(franka::Network& network)
    : loader_(LibraryDownloader(network).path()),
      body_jacobian_joint1{reinterpret_cast<decltype(&Ji_J_J1)>(loader_.getSymbol("Ji_J_J1"))},
      body_jacobian_joint2{reinterpret_cast<decltype(&Ji_J_J2)>(loader_.getSymbol("Ji_J_J2"))},
      body_jacobian_joint3{reinterpret_cast<decltype(&Ji_J_J3)>(loader_.getSymbol("Ji_J_J3"))},
      body_jacobian_joint4{reinterpret_cast<decltype(&Ji_J_J4)>(loader_.getSymbol("Ji_J_J4"))},
      body_jacobian_joint5{reinterpret_cast<decltype(&Ji_J_J5)>(loader_.getSymbol("Ji_J_J5"))},
      body_jacobian_joint6{reinterpret_cast<decltype(&Ji_J_J6)>(loader_.getSymbol("Ji_J_J6"))},
      body_jacobian_joint7{reinterpret_cast<decltype(&Ji_J_J7)>(loader_.getSymbol("Ji_J_J7"))},
      body_jacobian_flange{reinterpret_cast<decltype(&Ji_J_J8)>(loader_.getSymbol("Ji_J_J8"))},
      body_jacobian_ee{reinterpret_cast<decltype(&Ji_J_J9)>(loader_.getSymbol("Ji_J_J9"))},
      mass{reinterpret_cast<decltype(&M_NE)>(loader_.getSymbol("M_NE"))},
      zero_jacobian_joint1{reinterpret_cast<decltype(&O_J_J1)>(loader_.getSymbol("O_J_J1"))},
      zero_jacobian_joint2{reinterpret_cast<decltype(&O_J_J2)>(loader_.getSymbol("O_J_J2"))},
      zero_jacobian_joint3{reinterpret_cast<decltype(&O_J_J3)>(loader_.getSymbol("O_J_J3"))},
      zero_jacobian_joint4{reinterpret_cast<decltype(&O_J_J4)>(loader_.getSymbol("O_J_J4"))},
      zero_jacobian_joint5{reinterpret_cast<decltype(&O_J_J5)>(loader_.getSymbol("O_J_J5"))},
      zero_jacobian_joint6{reinterpret_cast<decltype(&O_J_J6)>(loader_.getSymbol("O_J_J6"))},
      zero_jacobian_joint7{reinterpret_cast<decltype(&O_J_J7)>(loader_.getSymbol("O_J_J7"))},
      zero_jacobian_flange{reinterpret_cast<decltype(&O_J_J8)>(loader_.getSymbol("O_J_J8"))},
      zero_jacobian_ee{reinterpret_cast<decltype(&O_J_J9)>(loader_.getSymbol("O_J_J9"))},
      joint1{reinterpret_cast<decltype(&O_T_J1)>(loader_.getSymbol("O_T_J1"))},
      joint2{reinterpret_cast<decltype(&O_T_J2)>(loader_.getSymbol("O_T_J2"))},
      joint3{reinterpret_cast<decltype(&O_T_J3)>(loader_.getSymbol("O_T_J3"))},
      joint4{reinterpret_cast<decltype(&O_T_J4)>(loader_.getSymbol("O_T_J4"))},
      joint5{reinterpret_cast<decltype(&O_T_J5)>(loader_.getSymbol("O_T_J5"))},
      joint6{reinterpret_cast<decltype(&O_T_J6)>(loader_.getSymbol("O_T_J6"))},
      joint7{reinterpret_cast<decltype(&O_T_J7)>(loader_.getSymbol("O_T_J7"))},
      flange{reinterpret_cast<decltype(&O_T_J8)>(loader_.getSymbol("O_T_J8"))},
      ee{reinterpret_cast<decltype(&O_T_J9)>(loader_.getSymbol("O_T_J9"))},
      coriolis{reinterpret_cast<decltype(&c_NE)>(loader_.getSymbol("c_NE"))},
      gravity{reinterpret_cast<decltype(&g_NE)>(loader_.getSymbol("g_NE"))} {}

}  // namespace franka
