#pragma once

#include <chrono>
#include <cstdint>
#include <ratio>

namespace franka {

/**
 * Represents a duration with millisecond resolution.
 */
class Duration {
 public:
  Duration() = default;

  Duration(const Duration&) = default;
  Duration& operator=(const Duration&) = default;

  Duration(Duration&&) = default;
  Duration& operator=(Duration&&) = default;

  Duration(std::chrono::duration<uint64_t, std::milli> duration);

  /**
   * Creates a new Duration instance from the given number of milliseconds.
   *
   * @param[in] milliseconds Number of milliseconds.
   */
  explicit Duration(uint64_t milliseconds);

  /**
   * Returns the stored duration as an std::chrono::duration.
   *
   * @return Duration as std::chrono::duration.
   */
  operator std::chrono::duration<uint64_t, std::milli>() const noexcept;

  /**
   * Returns the stored duration in \f$[s]\f$.
   *
   * @return Duration in \f$[s]\f$.
   */
  double s() const noexcept;

  /**
   * Returns the stored duration in \f$[ms]\f$.
   *
   * @return Duration in \f$[ms]\f$.
   */
  uint64_t ms() const noexcept;

  Duration operator+(const Duration& other) const noexcept;
  Duration& operator+=(const Duration& other) noexcept;
  Duration operator-(const Duration& other) const noexcept;
  Duration& operator-=(const Duration& other) noexcept;
  Duration operator*(uint64_t scalar) const noexcept;
  Duration& operator*=(uint64_t scalar) noexcept;
  Duration operator/(uint64_t scalar) const noexcept;
  Duration& operator/=(uint64_t scalar) noexcept;
  Duration operator%(const Duration& other) const noexcept;
  Duration& operator%=(const Duration& other) noexcept;

  bool operator==(const Duration& other) const noexcept;
  bool operator!=(const Duration& other) const noexcept;

  bool operator<(const Duration& other) const noexcept;
  bool operator<=(const Duration& other) const noexcept;

  bool operator>(const Duration& other) const noexcept;
  bool operator>=(const Duration& other) const noexcept;

 private:
  std::chrono::duration<uint64_t, std::milli> duration_;
};

}  // namespace franka
