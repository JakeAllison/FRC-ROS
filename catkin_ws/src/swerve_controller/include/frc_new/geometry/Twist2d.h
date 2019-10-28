/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <units/units.h>

namespace frc_new {
/**
 * A change in distance along arc since the last pose update. We can use ideas
 * from differential calculus to create new Pose2ds from a Twist2d and vise
 * versa.
 *
 * A Twist can be used to represent a difference between two poses.
 */
struct Twist2d {
  /**
   * Linear "dx" component
   */
  units::meter_t dx = 0_m;

  /**
   * Linear "dy" component
   */
  units::meter_t dy = 0_m;

  /**
   * Angular "dtheta" component (radians)
   */
  units::radian_t dtheta = 0_rad;

  /**
   * Checks equality between this Twist2d and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  bool operator==(const Twist2d& other) const {
    return units::math::abs(dx - other.dx) < 1E-9_m &&
           units::math::abs(dy - other.dy) < 1E-9_m &&
           units::math::abs(dtheta - other.dtheta) < 1E-9_rad;
  }

  /**
   * Checks inequality between this Twist2d and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are not equal.
   */
  bool operator!=(const Twist2d& other) const { return !operator==(other); }
};
}  // namespace frc