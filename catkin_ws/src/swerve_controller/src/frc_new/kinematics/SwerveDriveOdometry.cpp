/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc_new/kinematics/SwerveDriveOdometry.h>

namespace frc_new {
frc_new::SwerveDriveOdometry::SwerveDriveOdometry(
    SwerveDriveKinematics kinematics, const Pose2d& initialPose)
    : m_kinematics(kinematics), m_pose(initialPose) {
  m_previousAngle = m_pose.Rotation();
}

const Pose2d& frc_new::SwerveDriveOdometry::UpdateWithTime(
    units::second_t currentTime, const Rotation2d& angle,
    SwerveModuleState flState, SwerveModuleState frState, SwerveModuleState blState, SwerveModuleState brState) {
  units::second_t deltaTime =
      (m_previousTime >= 0_s) ? currentTime - m_previousTime : 0_s;
  m_previousTime = currentTime;

  frc_new::ChassisSpeeds speeds = m_kinematics.ToChassisSpeeds(flState, frState, blState, brState);
  units::meters_per_second_t dx = speeds.vx;
  units::meters_per_second_t dy = speeds.vy;
  units::radians_per_second_t dtheta = speeds.omega;
  static_cast<void>(dtheta);

  auto newPose = m_pose.Exp(
      {dx * deltaTime, dy * deltaTime, (angle - m_previousAngle).Radians()});

  m_previousAngle = angle;
  m_pose = {newPose.Translation(), angle};

  return m_pose;
}
}  // namespace frc