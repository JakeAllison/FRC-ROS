/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc_new/geometry/Transform2d.h"

#include "frc_new/geometry/Pose2d.h"

using namespace frc_new;

Transform2d::Transform2d(Pose2d initial, Pose2d final) {
  // We are rotating the difference between the translations
  // using a clockwise rotation matrix. This transforms the global
  // delta into a local delta (relative to the initial pose).
  m_translation = (final.Translation() - initial.Translation())
                      .RotateBy(-initial.Rotation());

  m_rotation = final.Rotation() - initial.Rotation();
}

Transform2d::Transform2d(Translation2d translation, Rotation2d rotation)
    : m_translation(translation), m_rotation(rotation) {}

bool Transform2d::operator==(const Transform2d& other) const {
  return m_translation == other.m_translation && m_rotation == other.m_rotation;
}

bool Transform2d::operator!=(const Transform2d& other) const {
  return !operator==(other);
}
