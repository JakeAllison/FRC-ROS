/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <chrono>
#include <cstddef>
#include <ctime>

#include <units/units.h>

#include "SwerveDriveKinematics.h"
#include "SwerveModuleState.h"
#include "frc_new/geometry/Pose2d.h"

namespace frc_new {

/**
 * Class for swerve drive odometry. Odometry allows you to track the robot's
 * position on the field over a course of a match using readings from your
 * swerve drive encoders and swerve azimuth encoders.
 *
 * Teams can use odometry during the autonomous period for complex tasks like
 * path following. Furthermore, odometry can be used for latency compensation
 * when using computer-vision systems.
 */
class SwerveDriveOdometry {
 public:
  /**
   * Constructs a SwerveDriveOdometry object.
   *
   * @param kinematics The swerve drive kinematics for your drivetrain.
   * @param initialPose The starting position of the robot on the field.
   */
  SwerveDriveOdometry(SwerveDriveKinematics kinematics,
                      const Pose2d& initialPose);

  /**
   * Resets the robot's position on the field.
   *
   * @param pose The position on the field that your robot is at.
   */
  void ResetPosition(const Pose2d& pose) {
    m_pose = pose;
    m_previousAngle = pose.Rotation();
  }

  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose over time. This method takes in the current time as
   * a parameter to calculate period (difference between two timestamps). The
   * period is used to calculate the change in distance from a velocity. This
   * also takes in an angle parameter which is used instead of the
   * angular rate that is calculated from forward kinematics.
   *
   * @param currentTime The current time.
   * @param angle The angle of the robot.
   * @param moduleStates The current state of all swerve modules. Please provide
   *                     the states in the same order in which you instantiated
   *                     your SwerveDriveKinematics.
   *
   * @return The new pose of the robot.
   */
  const Pose2d& UpdateWithTime(units::second_t currentTime,
                               const Rotation2d& angle,
                               SwerveModuleState flState, SwerveModuleState frState, SwerveModuleState blState, SwerveModuleState brState);

  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose over time. This method automatically calculates
   * the current time to calculate period (difference between two timestamps).
   * The period is used to calculate the change in distance from a velocity.
   * This also takes in an angle parameter which is used instead of the
   * angular rate that is calculated from forward kinematics.
   *
   * @param angle The angle of the robot.
   * @param moduleStates The current state of all swerve modules. Please provide
   *                     the states in the same order in which you instantiated
   *                     your SwerveDriveKinematics.
   *
   * @return The new pose of the robot.
   */
  const Pose2d& Update(const Rotation2d& angle,
                       SwerveModuleState flState, SwerveModuleState frState, SwerveModuleState blState, SwerveModuleState brState) {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    units::second_t time{now};
    return UpdateWithTime(time, angle, flState, frState, blState, brState);
  }

 private:
  SwerveDriveKinematics m_kinematics;
  Pose2d m_pose;

  units::second_t m_previousTime = -1_s;
  Rotation2d m_previousAngle;
};

}  // namespace frc