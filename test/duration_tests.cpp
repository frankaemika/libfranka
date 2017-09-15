// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <utility>

#include <gtest/gtest.h>

#include <franka/duration.h>

using franka::Duration;

TEST(Duration, CanDefaultConstruct) {
  Duration d;
  EXPECT_EQ(0u, d.toMSec());
  EXPECT_EQ(0.0, d.toSec());
}

TEST(Duration, CanConstructFromMilliseconds) {
  Duration d(12345u);
  EXPECT_EQ(12345u, d.toMSec());
  EXPECT_EQ(12.345, d.toSec());
}

TEST(Duration, CanConstructFromChrono) {
  std::chrono::duration<uint64_t, std::milli> chrono_duration(12345u);
  Duration d(chrono_duration);
  EXPECT_EQ(12345u, d.toMSec());
  EXPECT_EQ(12.345, d.toSec());
}

TEST(Duration, CanConvertToChrono) {
  Duration d(12345u);
  std::chrono::duration<uint64_t, std::milli> chrono_duration = d;
  EXPECT_EQ(12345u, chrono_duration.count());
}

TEST(Duration, CanCopy) {
  Duration d(12345u);
  Duration d2(d);
  EXPECT_EQ(12345u, d2.toMSec());

  Duration d3;
  d3 = d;
  EXPECT_EQ(12345u, d2.toMSec());
}

TEST(Duration, CanMove) {
  Duration d(12345u);
  Duration d2(std::move(d));
  EXPECT_EQ(12345u, d2.toMSec());

  Duration d3;
  d3 = std::move(d);
  EXPECT_EQ(12345u, d2.toMSec());
}

TEST(Duration, CanUseArithmeticOperations) {
  Duration d1(4u);
  Duration d2(3u);

  EXPECT_EQ(7u, (d1 + d2).toMSec());
  EXPECT_EQ(7u, (d2 + d1).toMSec());

  EXPECT_EQ(1u, (d1 - d2).toMSec());

  EXPECT_EQ(8u, (d1 * 2).toMSec());
  EXPECT_EQ(8u, (2 * d1).toMSec());

  EXPECT_EQ(2u, (d1 / 2).toMSec());
  EXPECT_EQ(1u, d1 / d2);

  EXPECT_EQ(1u, (d1 % d2).toMSec());
  EXPECT_EQ(1u, (d1 % 3u).toMSec());

  d1 += d2;
  EXPECT_EQ(7u, d1.toMSec());

  d1 -= d2;
  EXPECT_EQ(4u, d1.toMSec());

  d1 *= 2;
  EXPECT_EQ(8u, d1.toMSec());

  d1 /= 2;
  EXPECT_EQ(4u, d1.toMSec());

  d1 %= 3;
  EXPECT_EQ(1u, d1.toMSec());

  d1 *= 4;
  d1 %= d2;
  EXPECT_EQ(1u, d1.toMSec());
}

TEST(Duration, CanCompare) {
  EXPECT_TRUE(Duration(1) == Duration(1));
  EXPECT_FALSE(Duration(1) == Duration(2));
  EXPECT_FALSE(Duration(2) == Duration(1));

  EXPECT_FALSE(Duration(1) != Duration(1));
  EXPECT_TRUE(Duration(1) != Duration(2));
  EXPECT_TRUE(Duration(2) != Duration(1));

  EXPECT_TRUE(Duration(1) < Duration(2));
  EXPECT_FALSE(Duration(2) < Duration(1));
  EXPECT_FALSE(Duration(2) < Duration(2));

  EXPECT_FALSE(Duration(1) > Duration(2));
  EXPECT_TRUE(Duration(2) > Duration(1));
  EXPECT_FALSE(Duration(2) > Duration(2));

  EXPECT_TRUE(Duration(1) <= Duration(2));
  EXPECT_FALSE(Duration(2) <= Duration(1));
  EXPECT_TRUE(Duration(2) <= Duration(2));

  EXPECT_FALSE(Duration(1) >= Duration(2));
  EXPECT_TRUE(Duration(2) >= Duration(1));
  EXPECT_TRUE(Duration(2) >= Duration(2));
}
