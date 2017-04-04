#include <franka/control_types.h>

namespace franka {

Torques::Torques(std::array<double, 7> torques)
  : tau_J(std::move(torques)) {
}

Torques::Torques(std::initializer_list<double> torques) {
  if (torques.size() != 7) {
    throw std::invalid_argument("Invalid number of elements in tau_J.");
  }

  std::copy(torques.begin(), torques.end(), tau_J.begin());
}

} // namespace franka
