/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#define ROS

#include <frc_new/kinematics/SwerveModuleState.h>
#include "Constants.h"
#include <units/units.h>
#include <frc_new/geometry/Rotation2d.h>

#ifndef ROS

#include <frc/smartdashboard/SendableBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <rev/CANSparkMax.h>

#else

#include "ros_dummies/TalonSRX.h"
#include "ros_dummies/CANSparkMax.h"

#endif

#ifndef ROS
class SwerveModule : public frc::SendableBase {
#else
class SwerveModule {
#endif
public:
    SwerveModule(int driveMotorChannel, int turningMotorChannel, int calibrationValue, std::string name);
    frc_new::SwerveModuleState GetState();
    void SetDesiredState(const frc_new::SwerveModuleState& state);
    void SetDesiredState(units::meters_per_second_t speed, const frc_new::Rotation2d& angle);
    void InvertDrive(bool inverted);
    void InvertRot(bool inverted);
#ifndef ROS
    void InitSendable(frc::SendableBuilder& builder) override;
#endif
private:
    void SetupDriveMotor();
    void SetupTurningMotor();
    units::meters_per_second_t ConvertAngularToLinearVelocity(units::radians_per_second_t rpm, units::meter_t radius);
    units::radians_per_second_t ConvertLinearToAngularVelocity(units::meters_per_second_t velocity, units::meter_t radius);
    units::radians_per_second_t ConvertTalonVelocityToRadiansPerSecond(int ticksPer100ms);
    int ConvertRadiansPerSecondToTalonVelocity(units::radians_per_second_t radPerSec);
    units::radian_t ConvertEncoderUnitsToRadians(int encoderTicks);
    int ConvertRadiansToEncoderTicks(units::radian_t rads);
    static constexpr units::meter_t kWheelRadius = tigertronics::constants::driveWheelRadius;
    static constexpr int kEncoderResolution = tigertronics::constants::ctreEncoderTicksPerRev;
    double kTurnP = tigertronics::constants::swerveTurningkP;
    double kTurnI = tigertronics::constants::swerveTurningkI;
    double kTurnD = tigertronics::constants::swerveTurningkD;
    double kDriveF = tigertronics::constants::swerveDrivekF;
    double kDriveP = tigertronics::constants::swerveDrivekP;
    double kDriveI = tigertronics::constants::swerveDrivekI;
    double kDriveD = tigertronics::constants::swerveDrivekD;
    static constexpr units::radians_per_second_t kMaxTurnVel = tigertronics::constants::swerveTurningMaxVel;
    static constexpr double kMaxTurnAccel = tigertronics::constants::swerveTurningMaxAccel;
    static constexpr int kTurnErrorAllowance = tigertronics::constants::swerveTurningErrorAllowance;
    int kCalibrationValue;
    rev::CANSparkMax m_driveMotor;
    rev::CANPIDController m_drivePIDController;
    rev::CANEncoder m_driveEncoder;
    ctre::phoenix::motorcontrol::can::TalonSRX m_turningMotor;
};
