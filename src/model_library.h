#pragma once

#include <functional>

#include "libfcimodels.h"
#include "library_loader.h"

namespace franka {

class ModelLibrary {
 public:
  ModelLibrary();

 private:
  LibraryLoader loader_;

 public:
  const std::function<decltype(M_NE_file)> mass;
  const std::function<decltype(O_T_J1_file)> joint1;
  const std::function<decltype(O_T_J2_file)> joint2;
  const std::function<decltype(O_T_J3_file)> joint3;
  const std::function<decltype(O_T_J4_file)> joint4;
  const std::function<decltype(O_T_J5_file)> joint5;
  const std::function<decltype(O_T_J6_file)> joint6;
  const std::function<decltype(O_T_J7_file)> joint7;
  const std::function<decltype(O_T_J8_file)> flange;
  const std::function<decltype(O_T_J9_file)> ee;
  const std::function<decltype(c_NE_file)> coriolis;
  const std::function<decltype(g_NE_file)> gravity;
};

}  // namespace franka
