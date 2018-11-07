// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <chrono>
#include <cstdint>
#include <ratio>

/**
 * @file duration.h
 * Contains the franka::Duration type.
 */

namespace franka {

/**
 * Represents a duration with millisecond resolution.
 */
class Duration {
 public:
  /**
   * Creates a new Duration instance with zero milliseconds.
   */
  Duration() noexcept;

  /**
   * Creates a new Duration instance from the given number of milliseconds.
   *
   * @param[in] milliseconds Number of milliseconds.
   */
  explicit Duration(uint64_t milliseconds) noexcept;

  /**
   * Creates a new Duration instance from an std::chrono::duration.
   *
   * @param[in] duration Duration.
   */
  Duration(std::chrono::duration<uint64_t, std::milli> duration) noexcept;

  /**
   * Creates a copy of a Duration instance.
   */
  Duration(const Duration&) = default;

  /**
   * Assigns the contents of one Duration to another.
   *
   * @return Result of the operation.
   */
  Duration& operator=(const Duration&) = default;

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
  double toSec() const noexcept;

  /**
   * Returns the stored duration in \f$[ms]\f$.
   *
   * @return Duration in \f$[ms]\f$.
   */
  uint64_t toMSec() const noexcept;

  /**
   * @name Arithmetic operators
   * @{
   */

  /**
   * Performs addition.
   *
   * @param[in] rhs Right-hand side of the operation.
   *
   * @return Result of the operation.
   */
  Duration operator+(const Duration& rhs) const noexcept;
  /**
   * Performs addition.
   *
   * @param[in] rhs Right-hand side of the operation.
   *
   * @return This duration.
   */
  Duration& operator+=(const Duration& rhs) noexcept;

  /**
   * Performs subtraction.
   *
   * @param[in] rhs Right-hand side of the operation.
   *
   * @return Result of the operation.
   */
  Duration operator-(const Duration& rhs) const noexcept;
  /**
   * Performs subtraction.
   *
   * @param[in] rhs Right-hand side of the operation.
   *
   * @return This duration.
   */
  Duration& operator-=(const Duration& rhs) noexcept;

  /**
   * Performs multiplication.
   *
   * @param[in] rhs Right-hand side of the operation.
   *
   * @return Result of the operation.
   */
  Duration operator*(uint64_t rhs) const noexcept;
  /**
   * Performs multiplication.
   *
   * @param[in] rhs Right-hand side of the operation.
   *
   * @return This duration.
   */
  Duration& operator*=(uint64_t rhs) noexcept;

  /**
   * Performs division.
   *
   * @param[in] rhs Right-hand side of the operation.
   *
   * @return Result of the operation.
   */
  uint64_t operator/(const Duration& rhs) const noexcept;
  /**
   * Performs division.
   *
   * @param[in] rhs Right-hand side of the operation.
   *
   * @return Result of the operation.
   */
  Duration operator/(uint64_t rhs) const noexcept;
  /**
   * Performs division.
   *
   * @param[in] rhs Right-hand side of the operation.
   *
   * @return This duration.
   */
  Duration& operator/=(uint64_t rhs) noexcept;

  /**
   * Performs the modulo operation.
   *
   * @param[in] rhs Right-hand side of the operation.
   *
   * @return Result of the operation.
   */
  Duration operator%(const Duration& rhs) const noexcept;
  /**
   * Performs the modulo operation.
   *
   * @param[in] rhs Right-hand side of the operation.
   *
   * @return Result of the operation.
   */
  Duration operator%(uint64_t rhs) const noexcept;
  /**
   * Performs the modulo operation.
   *
   * @param[in] rhs Right-hand side of the operation.
   *
   * @return This duration.
   */
  Duration& operator%=(const Duration& rhs) noexcept;
  /**
   * Performs the modulo operation.
   *
   * @param[in] rhs Right-hand side of the operation.
   *
   * @return This duration.
   */
  Duration& operator%=(uint64_t rhs) noexcept;

  /**
   * @}
   */

  /**
   * @name Comparison operators
   * @{
   */

  /**
   * Compares two durations for equality.
   *
   * @param[in] rhs Right-hand side of the comparison.
   *
   * @return True if the duration are equal, false otherwise.
   */
  bool operator==(const Duration& rhs) const noexcept;
  /**
   * Compares two durations for inequality.
   *
   * @param[in] rhs Right-hand side of the comparison.
   *
   * @return True if the duration are not equal, false otherwise.
   */
  bool operator!=(const Duration& rhs) const noexcept;

  /**
   * Compares two durations.
   *
   * @param[in] rhs Right-hand side of the comparison.
   *
   * @return True if this duration is shorter than rhs, false otherwise.
   */
  bool operator<(const Duration& rhs) const noexcept;
  /**
   * Compares two durations.
   *
   * @param[in] rhs Right-hand side of the comparison.
   *
   * @return True if this duration is shorter than or equal to rhs, false otherwise.
   */
  bool operator<=(const Duration& rhs) const noexcept;

  /**
   * Compares two durations.
   *
   * @param[in] rhs Right-hand side of the comparison.
   *
   * @return True if this duration is longer than rhs, false otherwise.
   */
  bool operator>(const Duration& rhs) const noexcept;
  /**
   * Compares two durations.
   *
   * @param[in] rhs Right-hand side of the comparison.
   *
   * @return True if this duration is longer than or equal to rhs, false otherwise.
   */
  bool operator>=(const Duration& rhs) const noexcept;

  /**
   * @}
   */

 private:
  std::chrono::duration<uint64_t, std::milli> duration_;
};

/**
 * Performs multiplication.
 *
 * @param[in] lhs Left-hand side of the multiplication.
 * @param[in] rhs Right-hand side of the multiplication.
 *
 * @return Result of the multiplication.
 */
Duration operator*(uint64_t lhs, const Duration& rhs) noexcept;

}  // namespace franka
