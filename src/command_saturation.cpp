#include <franka/command_saturation.h>

namespace franka {

std::array<double, 7> limitRate(const std::array<double, 7>& max_derivatives,
                                const std::array<double, 7>& desired_values,
                                const std::array<double, 7>& last_desired_values) {
  std::array<double, 7> limited_values{};
  for (size_t i = 0; i < 7; i++) {
    double desired_difference = (desired_values[i] - last_desired_values[i]) / 1e-3;
    limited_values[i] =
        last_desired_values[i] +
        std::max(std::min(desired_difference, max_derivatives[i]), -max_derivatives[i]) * 1e-3;
  }
  return limited_values;
}

}  // namespace franka
