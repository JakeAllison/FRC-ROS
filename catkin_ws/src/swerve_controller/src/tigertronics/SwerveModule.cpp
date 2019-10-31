#include "tigertronics/SwerveModule.h"

#include <frc_new/geometry/Rotation2d.h>

#ifndef ROS
#include <frc/smartdashboard/SendableBuilder.h>
#endif

#define M_PI 3.14159265358979323846

constexpr units::meter_t SwerveModule::kWheelRadius;
constexpr int SwerveModule::kEncoderResolution;
constexpr int SwerveModule::kTurnErrorAllowance;
constexpr units::radians_per_second_t SwerveModule::kMaxTurnVel;
constexpr double SwerveModule::kMaxTurnAccel;

SwerveModule::SwerveModule(const int driveMotorPort, const int turingMotorPort, const int calibrationValue, const std::string name) 
    : m_driveMotor(driveMotorPort, rev::CANSparkMaxLowLevel::MotorType::kBrushless)
    , m_turningMotor(turingMotorPort), m_drivePIDController(m_driveMotor.GetPIDController())
    , m_driveEncoder(m_driveMotor.GetEncoder())
    , kCalibrationValue(calibrationValue) {
    
#ifndef ROS
    SetName(name + " SwerveModule");
    SetSubsystem("SwerveSubsystem");
#endif
    SetupDriveMotor();
    SetupTurningMotor();
}

void SwerveModule::SetDesiredState(units::meters_per_second_t speed, const frc_new::Rotation2d& angle) {
    frc_new::SwerveModuleState state;
    state.speed = speed;
    state.angle = angle;
    SetDesiredState(state);
}

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void SwerveModule::SetDesiredState(const frc_new::SwerveModuleState& state) {
    frc_new::Rotation2d inverseInput = frc_new::Rotation2d(state.angle.Cos(), -state.angle.Sin());
    frc_new::Rotation2d deltaAngle = GetState().angle.RotateBy(inverseInput);
    frc_new::Rotation2d setAngle = state.angle;
    units::meters_per_second_t setSpeed = state.speed;
    if(deltaAngle.Radians().value() > (M_PI/2) && deltaAngle.Radians().value() < ((3 * M_PI) / 2)) {
        setAngle.RotateBy(frc_new::Rotation2d(units::radian_t(M_PI)));
        setSpeed = setSpeed * -1;
    }

    units::revolutions_per_minute_t setrpm = ConvertLinearToAngularVelocity(state.speed, kWheelRadius);
    //m_drivePIDController.Set(setrpm.value(), rev::ControlType::kVelocity);
    m_driveMotor.Set(map(setSpeed.value(), -3, 3, -1, 1));
    
#ifndef ROS
    m_turningMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, ConvertRadiansToEncoderTicks(setAngle.Radians()));
#else
    m_turningMotor.Set(setAngle.Radians().value());
#endif
}

frc_new::SwerveModuleState SwerveModule::GetState() {
#ifndef ROS
    units::radians_per_second_t wheelSpeed = units::revolutions_per_minute_t(m_driveEncoder.GetVelocity());
    frc_new::SwerveModuleState state;
    state.angle = frc_new::Rotation2d(ConvertEncoderUnitsToRadians(m_turningMotor.GetSelectedSensorPosition()));
#else
    units::radians_per_second_t wheelSpeed = units::radians_per_second_t(m_driveEncoder.GetVelocity());
    frc_new::SwerveModuleState state;
    state.angle = frc_new::Rotation2d(units::radian_t(m_turningMotor.GetEncoderPosition()));
#endif

    state.speed = ConvertAngularToLinearVelocity(wheelSpeed, kWheelRadius);
    return state;
}

void SwerveModule::SetupDriveMotor() {
    m_driveMotor.RestoreFactoryDefaults();
#ifndef ROS
    m_driveMotor.SetInverted(false);
    m_drivePIDController.SetFF(kDriveF);
    m_drivePIDController.SetP(kDriveP);
    m_drivePIDController.SetI(kDriveI);
    m_drivePIDController.SetD(kDriveD);
    m_drivePIDController.SetOutputRange(-1, 1, 0);
    m_driveEncoder.SetPosition(0);
#endif
}

void SwerveModule::SetupTurningMotor() {
    m_turningMotor.ConfigFactoryDefault();
    m_turningMotor.Config_kF(0, 0, 10);
    m_turningMotor.Config_kP(0, kTurnP, 10);
    m_turningMotor.Config_kI(0, kTurnI, 10);
    m_turningMotor.Config_kD(0, kTurnD, 10);
#ifndef ROS
    m_turningMotor.SetInverted(true);
    m_turningMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
    m_turningMotor.SetSensorPhase(false);
    m_turningMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    m_turningMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
    m_turningMotor.ConfigNominalOutputForward(0, 10);
    m_turningMotor.ConfigNominalOutputReverse(0, 10);
    m_turningMotor.ConfigPeakOutputForward(1, 10);
    m_turningMotor.ConfigPeakOutputReverse(-1, 10);
    m_turningMotor.ConfigAllowableClosedloopError(0, kTurnErrorAllowance, 10);
    m_turningMotor.EnableCurrentLimit(true);
    m_turningMotor.ConfigContinuousCurrentLimit(10, 10);
    m_turningMotor.ConfigPeakCurrentLimit(0, 10);
    m_turningMotor.GetSensorCollection().SyncQuadratureWithPulseWidth(0, 0, true, kCalibrationValue, 10);
    m_turningMotor.ConfigMotionCruiseVelocity(ConvertRadiansPerSecondToTalonVelocity(kMaxTurnVel), 10);
    m_turningMotor.ConfigMotionAcceleration(kMaxTurnAccel, 10);
    m_turningMotor.SetSelectedSensorPosition(0);
    m_turningMotor.ConfigForwardLimitSwitchSource(ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated, ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled, 10);
    m_turningMotor.ConfigReverseLimitSwitchSource(ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated, ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled, 10);
#endif
}


int SwerveModule::ConvertRadiansToEncoderTicks(units::radian_t rads) {
    return (rads.value() * kEncoderResolution) / (M_PI * 2);
}

int SwerveModule::ConvertRadiansPerSecondToTalonVelocity(units::radians_per_second_t radPerSec) {
    return ConvertRadiansToEncoderTicks(units::radian_t(radPerSec.value())) / 10;
}

units::radians_per_second_t SwerveModule::ConvertTalonVelocityToRadiansPerSecond(int ticksPer100ms) {
    return units::radians_per_second_t(ConvertEncoderUnitsToRadians(ticksPer100ms).value() * 10);
}

units::radian_t SwerveModule::ConvertEncoderUnitsToRadians(int encoderTicks) {
    return units::radian_t((encoderTicks * (M_PI * 2)) / kEncoderResolution);
}

units::meters_per_second_t SwerveModule::ConvertAngularToLinearVelocity(units::radians_per_second_t wheelSpeed, units::meter_t radius) {
    return units::meters_per_second_t(wheelSpeed.value() * radius.value());
}

units::radians_per_second_t SwerveModule::ConvertLinearToAngularVelocity(units::meters_per_second_t velocity, units::meter_t radius) {
    return units::radians_per_second_t(velocity.value() / radius.value());
}

#ifndef ROS
void SwerveModule::InitSendable(frc::SendableBuilder& builder) {
    builder.SetSmartDashboardType("SwerveModule");
    builder.AddDoubleProperty("turnP", [=]() { return kTurnP; }, [=](double value) { kTurnP = value; });
    builder.AddDoubleProperty("turnI", [=]() { return kTurnI; }, [=](double value) { kTurnI = value; });
    builder.AddDoubleProperty("turnD", [=]() { return kTurnD; }, [=](double value) { kTurnD = value; });
    builder.AddDoubleProperty("driveF", [=]() { return kDriveF; }, [=](double value) { kDriveF = value; });
    builder.AddDoubleProperty("driveP", [=]() { return kDriveP; }, [=](double value) { kDriveP = value; });
    builder.AddDoubleProperty("driveI", [=]() { return kDriveI; }, [=](double value) { kDriveI = value; });
    builder.AddDoubleProperty("driveD", [=]() { return kDriveD; }, [=](double value) { kDriveD = value; });
    builder.AddDoubleProperty("driveSetpoint", [=]() { return GetState().speed.value(); }, 
    [=](double value) { SetDesiredState(units::meters_per_second_t(value), GetState().angle); });
    builder.AddDoubleProperty("angleSetpoint", [=]() { return GetState().angle.Degrees().value(); }, 
    [=](double value) { SetDesiredState(GetState().speed, frc_new::Rotation2d(units::degree_t(value))); });

}
#endif

void SwerveModule::InvertDrive(bool inverted) {
    m_driveMotor.SetInverted(inverted);
}

void SwerveModule::InvertRot(bool inverted) {
    m_turningMotor.SetInverted(inverted);
}

