/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "tigertronics/SwerveDrivetrain.h"

#ifndef ROS
#include <frc/smartdashboard/SmartDashboard.h>
#endif

SwerveDrivetrain::SwerveDrivetrain() {
    m_imu.ZeroYaw();
    m_frontLeft.InvertDrive(true);
    m_frontRight.InvertDrive(false);
    m_backLeft.InvertDrive(true);
    m_backRight.InvertDrive(false);
    m_frontLeft.InvertRot(true);
    m_frontRight.InvertRot(false);
    m_backLeft.InvertRot(true);
    m_backRight.InvertRot(false);
}

//+x is forward
//+y is left
//+rot is CCW
void SwerveDrivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative) {   
  std::array<frc_new::SwerveModuleState, 4> states = m_kinematics.ToSwerveModuleStates(
      fieldRelative ? frc_new::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, GetAngle())
                    : frc_new::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.NormalizeWheelSpeeds(&states, kMaxSpeed);
#ifndef ROS
  m_kinematics.NormalizeWheelSpeeds(&states, kMaxSpeed);
#endif

  frc_new::SwerveModuleState fl = states.at(0);
  frc_new::SwerveModuleState fr = states.at(1);
  frc_new::SwerveModuleState bl = states.at(2);
  frc_new::SwerveModuleState br = states.at(3);

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

const frc_new::Pose2d& SwerveDrivetrain::UpdateOdometry() {
  return m_odometry.Update(GetAngle(), m_frontLeft.GetState(), m_frontRight.GetState(),
                    m_backLeft.GetState(), m_backRight.GetState());
}

#ifndef ROS
void SwerveDrivetrain::LogModulesToDashboard() {
    frc::SmartDashboard::PutData(&m_frontLeft);
    frc::SmartDashboard::PutData(&m_frontRight);
    frc::SmartDashboard::PutData(&m_backLeft);
    frc::SmartDashboard::PutData(&m_backRight);
}
#endif
