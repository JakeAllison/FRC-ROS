/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <limits>

#include <units/units.h>

#include "frc_new/geometry/Pose2d.h"
#include "frc_new/spline/Spline.h"

namespace frc_new {
/**
 * An interface for defining user-defined velocity and acceleration constraints
 * while generating trajectories.
 */
class TrajectoryConstraint {
 public:
  TrajectoryConstraint() = default;

  TrajectoryConstraint(const TrajectoryConstraint&) = default;
  TrajectoryConstraint& operator=(const TrajectoryConstraint&) = default;

  TrajectoryConstraint(TrajectoryConstraint&&) = default;
  TrajectoryConstraint& operator=(TrajectoryConstraint&&) = default;

  virtual ~TrajectoryConstraint() = default;

  /**
   * Represents a minimum and maximum acceleration.
   */
  struct MinMax {
    /**
     * The minimum acceleration.
     */
    units::meters_per_second_squared_t minAcceleration{
        -std::numeric_limits<double>::max()};

    /**
     * The maximum acceleration.
     */
    units::meters_per_second_squared_t maxAcceleration{
        std::numeric_limits<double>::max()};
  };

  /**
   * Returns the max velocity given the current pose and curvature.
   *
   * @param pose The pose at the current point in the trajectory.
   * @param curvature The curvature at the current point in the trajectory.
   * @param velocity The velocity at the current point in the trajectory before
   *                                constraints are applied.
   *
   * @return The absolute maximum velocity.
   */
  virtual units::meters_per_second_t MaxVelocity(
      const Pose2d& pose, curvature_t curvature,
      units::meters_per_second_t velocity) = 0;

  /**
   * Returns the minimum and maximum allowable acceleration for the trajectory
   * given pose, curvature, and speed.
   *
   * @param pose The pose at the current point in the trajectory.
   * @param curvature The curvature at the current point in the trajectory.
   * @param speed The speed at the current point in the trajectory.
   *
   * @return The min and max acceleration bounds.
   */
  virtual MinMax MinMaxAcceleration(const Pose2d& pose, curvature_t curvature,
                                    units::meters_per_second_t speed) = 0;
};
}  // namespace frc