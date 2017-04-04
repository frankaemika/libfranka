#include <gtest/gtest.h>

#include <franka/control_types.h>
#include <franka/exception.h>

TEST(Torques, CanConstructFromArray) {
  std::array<double, 7> array {0, 1, 2, 3, 4, 5, 6};
  franka::Torques t(array);
  EXPECT_EQ(array, t.tau_J);
}

TEST(Torques, CanConstructFromInitializerList) {
  std::array<double, 7> array {0, 1, 2, 3, 4, 5, 6};
  franka::Torques t({0, 1, 2, 3, 4, 5, 6});
  EXPECT_EQ(array, t.tau_J);
}

TEST(Torques, CanNotConstructFromTooSmallInitializerList) {
  EXPECT_THROW(franka::Torques({0, 1, 2, 3, 4, 5}), franka::ControlException);
}
