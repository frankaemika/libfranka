#include "model_library.h"

namespace franka {

ModelLibrary::ModelLibrary()
    : loader_("libfcimodels"),
      mass{reinterpret_cast<decltype(&M_NE_file)>(
          loader_.getSymbol("M_NE_file"))},
      joint1{reinterpret_cast<decltype(&O_T_J1_file)>(
          loader_.getSymbol("O_T_J1_file"))},
      joint2{reinterpret_cast<decltype(&O_T_J2_file)>(
          loader_.getSymbol("O_T_J2_file"))},
      joint3{reinterpret_cast<decltype(&O_T_J3_file)>(
          loader_.getSymbol("O_T_J3_file"))},
      joint4{reinterpret_cast<decltype(&O_T_J4_file)>(
          loader_.getSymbol("O_T_J4_file"))},
      joint5{reinterpret_cast<decltype(&O_T_J5_file)>(
          loader_.getSymbol("O_T_J5_file"))},
      joint6{reinterpret_cast<decltype(&O_T_J6_file)>(
          loader_.getSymbol("O_T_J6_file"))},
      joint7{reinterpret_cast<decltype(&O_T_J7_file)>(
          loader_.getSymbol("O_T_J7_file"))},
      flange{reinterpret_cast<decltype(&O_T_J8_file)>(
          loader_.getSymbol("O_T_J8_file"))},
      ee{reinterpret_cast<decltype(&O_T_J9_file)>(
          loader_.getSymbol("O_T_J9_file"))},
      coriolis{reinterpret_cast<decltype(&c_NE_file)>(
          loader_.getSymbol("c_NE_file"))},
      gravity{reinterpret_cast<decltype(&g_NE_file)>(
          loader_.getSymbol("g_NE_file"))} {}

}  // namespace franka
