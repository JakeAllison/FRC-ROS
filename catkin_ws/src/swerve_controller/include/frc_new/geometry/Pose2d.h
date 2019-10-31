/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Transform2d.h"
#include "Translation2d.h"
#include "Twist2d.h"

namespace frc_new {

/**
 * Represents a 2d pose containing translational and rotational elements.
 */
class Pose2d {
 public:
  /**
   * Constructs a pose at the origin facing toward the positive X axis.
   * (Translation2d{0, 0} and Rotation{0})
   */
  constexpr Pose2d() = default;

  /**
   * Constructs a pose with the specified translation and rotation.
   *
   * @param translation The translational component of the pose.
   * @param rotation The rotational component of the pose.
   */
  Pose2d(Translation2d translation, Rotation2d rotation);

  /**
   * Convenience constructors that takes in x and y values directly instead of
   * having to construct a Translation2d.
   *
   * @param x The x component of the translational component of the pose.
   * @param y The y component of the translational component of the pose.
   * @param rotation The rotational component of the pose.
   */
  Pose2d(units::meter_t x, units::meter_t y, Rotation2d rotation);

  /**
   * Transforms the pose by the given transformation and returns the new
   * transformed pose.
   *
   * [x_new]    [cos, -sin, 0][transform.x]
   * [y_new] += [sin,  cos, 0][transform.y]
   * [t_new]    [0,    0,   1][transform.t]
   *
   * @param other The transform to transform the pose by.
   *
   * @return The transformed pose.
   */
  Pose2d operator+(const Transform2d& other) const;

  /**
   * Transforms the current pose by the transformation.
   *
   * This is similar to the + operator, except that it mutates the current
   * object.
   *
   * @param other The transform to transform the pose by.
   *
   * @return Reference to the new mutated object.
   */
  Pose2d& operator+=(const Transform2d& other);

  /**
   * Returns the Transform2d that maps the one pose to another.
   *
   * @param other The initial pose of the transformation.
   * @return The transform that maps the other pose to the current pose.
   */
  Transform2d operator-(const Pose2d& other) const;

  /**
   * Checks equality between this Pose2d and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  bool operator==(const Pose2d& other) const;

  /**
   * Checks inequality between this Pose2d and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are not equal.
   */
  bool operator!=(const Pose2d& other) const;

  /**
   * Returns the underlying translation.
   *
   * @return Reference to the translational component of the pose.
   */
  const Translation2d& Translation() const { return m_translation; }

  /**
   * Returns the underlying rotation.
   *
   * @return Reference to the rotational component of the pose.
   */
  const Rotation2d& Rotation() const { return m_rotation; }

  /**
   * Transforms the pose by the given transformation and returns the new pose.
   * See + operator for the matrix multiplication performed.
   *
   * @param other The transform to transform the pose by.
   *
   * @return The transformed pose.
   */
  Pose2d TransformBy(const Transform2d& other) const;

  /**
   * Returns the other pose relative to the current pose.
   *
   * This function can often be used for trajectory tracking or pose
   * stabilization algorithms to get the error between the reference and the
   * current pose.
   *
   * @param other The pose that is the origin of the new coordinate frame that
   * the current pose will be converted into.
   *
   * @return The current pose relative to the new origin pose.
   */
  Pose2d RelativeTo(const Pose2d& other) const;

  /**
   * Obtain a new Pose2d from a (constant curvature) velocity.
   *
   * See <https://file.tavsys.net/control/state-space-guide.pdf> section on
   * nonlinear pose estimation for derivation.
   *
   * The twist is a change in pose in the robot's coordinate frame since the
   * previous pose update. When the user runs exp() on the previous known
   * field-relative pose with the argument being the twist, the user will
   * receive the new field-relative pose.
   *
   * "Exp" represents the pose exponential, which is solving a differential
   * equation moving the pose forward in time.
   *
   * @param twist The change in pose in the robot's coordinate frame since the
   * previous pose update. For example, if a non-holonomic robot moves forward
   * 0.01 meters and changes angle by .5 degrees since the previous pose update,
   * the twist would be Twist2d{0.01, 0.0, toRadians(0.5)}
   *
   * @return The new pose of the robot.
   */
  Pose2d Exp(const Twist2d& twist) const;

  /**
   * Returns a Twist2d that maps this pose to the end pose. If c is the output
   * of a.Log(b), then a.Exp(c) would yield b.
   *
   * @param end The end pose for the transformation.
   *
   * @return The twist that maps this to end.
   */
  Twist2d Log(const Pose2d& end) const;

 private:
  Translation2d m_translation;
  Rotation2d m_rotation;
};
}  // namespace frc