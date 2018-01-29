// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <cstdint>

/**
 * @file command_types.h
 * Contains types for the commands that can be sent from franka::Robot.
 */

namespace franka {

/**
 * Parameters of a cuboid used as virtual wall.
 *
 * @see Robot::getVirtualWall
 */
struct VirtualWallCuboid {
  /**
   * ID of the virtual wall.
   */
  int32_t id;

  /**
   * Corner point of the cuboid in world frame in \f$[m]\f$.
   */
  std::array<double, 3> object_world_size;

  /**
   * 4x4 transformation matrix, column-major.
   */
  std::array<double, 16> p_frame;

  /**
   * True if this Cartesian limit is active, false otherwise.
   */
  bool active;
};

}  // namespace franka
