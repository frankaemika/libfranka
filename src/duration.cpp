#include <franka/duration.h>

namespace franka {

Duration::Duration(std::chrono::duration<uint64_t, std::milli> duration) : duration_{duration} {}

Duration::Duration(uint64_t milliseconds) : duration_{milliseconds} {}

Duration::operator std::chrono::duration<uint64_t, std::milli>() const noexcept {
  return duration_;
}

double Duration::s() const noexcept {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration_).count();
}

uint64_t Duration::ms() const noexcept {
  return duration_.count();
}

Duration Duration::operator+(const Duration& other) const noexcept {
  return duration_ + other.duration_;
}

Duration& Duration::operator+=(const Duration& other) noexcept {
  duration_ += other.duration_;
  return *this;
}

Duration Duration::operator-(const Duration& other) const noexcept {
  return duration_ - other.duration_;
}

Duration& Duration::operator-=(const Duration& other) noexcept {
  duration_ -= other.duration_;
  return *this;
}

Duration Duration::operator*(uint64_t scalar) const noexcept {
  return duration_ * scalar;
}

Duration& Duration::operator*=(uint64_t scalar) noexcept {
  duration_ *= scalar;
  return *this;
}

Duration Duration::operator/(uint64_t scalar) const noexcept {
  return duration_ / scalar;
}

Duration& Duration::operator/=(uint64_t scalar) noexcept {
  duration_ /= scalar;
  return *this;
}

Duration Duration::operator%(const Duration& other) const noexcept {
  return duration_ % other.duration_;
}

Duration& Duration::operator%=(const Duration& other) noexcept {
  duration_ %= other.duration_;
  return *this;
}

bool Duration::operator==(const Duration& other) const noexcept {
  return duration_ == other.duration_;
}

bool Duration::operator!=(const Duration& other) const noexcept {
  return duration_ != other.duration_;
}

bool Duration::operator<(const Duration& other) const noexcept {
  return duration_ < other.duration_;
}

bool Duration::operator<=(const Duration& other) const noexcept {
  return duration_ <= other.duration_;
}

bool Duration::operator>(const Duration& other) const noexcept {
  return duration_ > other.duration_;
}

bool Duration::operator>=(const Duration& other) const noexcept {
  return duration_ >= other.duration_;
}

}  // namespace franka
