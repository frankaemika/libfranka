#include <franka/rate_limiting.h>

namespace {

template <std::size_t size>
std::array<double, size> limitRate(const std::array<double, size>& max_derivatives,
                                   const std::array<double, size>& desired_values,
                                   const std::array<double, size>& last_desired_values) {
  std::array<double, size> limited_values{};
  for (size_t i = 0; i < size; i++) {
    double desired_difference = (desired_values[i] - last_desired_values[i]) / 1e-3;
    limited_values[i] =
        last_desired_values[i] +
        std::max(std::min(desired_difference, max_derivatives[i]), -max_derivatives[i]) * 1e-3;
  }
  return limited_values;
}

}  // anonymous namespace

namespace franka {

std::array<double, 7> limitRate(const std::array<double, 7>& max_derivatives,
                                const std::array<double, 7>& desired_values,
                                const std::array<double, 7>& last_desired_values) {
  return ::limitRate(max_derivatives, desired_values, last_desired_values);
}

std::array<double, 6> limitRate(const std::array<double, 6>& max_derivatives,
                                const std::array<double, 6>& desired_values,
                                const std::array<double, 6>& last_desired_values) {
  return ::limitRate(max_derivatives, desired_values, last_desired_values);
}

}  // namespace franka
