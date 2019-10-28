/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc_new/kinematics/SwerveDriveKinematics.h>
#include <algorithm>

namespace frc_new {

std::array<frc_new::SwerveModuleState, 4> SwerveDriveKinematics::ToSwerveModuleStates(const ChassisSpeeds& chassisSpeeds, const Translation2d& centerOfRotation) {
  // We have a new center of rotation. We need to compute the matrix again.
  if (centerOfRotation != m_previousCoR) {
    for (size_t i = 0; i < 4; i++) {
      // clang-format off
      m_inverseKinematics.template block<2, 3>(i * 2, 0) <<
        1, 0, (-m_modules[i].Y() + centerOfRotation.Y()).template to<double>(),
        0, 1, (+m_modules[i].X() - centerOfRotation.X()).template to<double>();
      // clang-format on
    }
    m_previousCoR = centerOfRotation;
  }

  Eigen::Vector3d chassisSpeedsVector;
  chassisSpeedsVector << chassisSpeeds.vx.to<double>(),
      chassisSpeeds.vy.to<double>(), chassisSpeeds.omega.to<double>();

  Eigen::Matrix<double, 4 * 2, 1> moduleStatesMatrix =
      m_inverseKinematics * chassisSpeedsVector;

  std::array<SwerveModuleState, 4> moduleStates;

  for (size_t i = 0; i < 4; i++) {
    units::meters_per_second_t x =
        units::meters_per_second_t{moduleStatesMatrix(i * 2, 0)};
    units::meters_per_second_t y =
        units::meters_per_second_t{moduleStatesMatrix(i * 2 + 1, 0)};

    auto speed = units::math::hypot(x, y);
    Rotation2d rotation{x.to<double>(), y.to<double>()};
    moduleStates[i] = {speed, rotation};
  }

  return moduleStates;
}

ChassisSpeeds SwerveDriveKinematics::ToChassisSpeeds(frc_new::SwerveModuleState flState, frc_new::SwerveModuleState frState, frc_new::SwerveModuleState blState, frc_new::SwerveModuleState brState) {
  std::array<SwerveModuleState, 4> moduleStates{flState, frState, blState, brState};
  Eigen::Matrix<double, 4 * 2, 1> moduleStatesMatrix;

  for (size_t i = 0; i < 4; i++) {
    SwerveModuleState module = moduleStates[i];
    moduleStatesMatrix.row(i * 2)
        << module.speed.to<double>() * module.angle.Cos();
    moduleStatesMatrix.row(i * 2 + 1)
        << module.speed.to<double>() * module.angle.Sin();
  }

  Eigen::Vector3d chassisSpeedsVector =
      m_forwardKinematics.solve(moduleStatesMatrix);

  return {units::meters_per_second_t{chassisSpeedsVector(0)},
          units::meters_per_second_t{chassisSpeedsVector(1)},
          units::radians_per_second_t{chassisSpeedsVector(2)}};
}

void SwerveDriveKinematics::NormalizeWheelSpeeds(
    std::array<SwerveModuleState, 4>* moduleStates,
    units::meters_per_second_t attainableMaxSpeed) {
  auto& states = *moduleStates;
  auto realMaxSpeed = std::max_element(states.begin(), states.end(),
                                       [](const auto& a, const auto& b) {
                                         return units::math::abs(a.speed) <
                                                units::math::abs(b.speed);
                                       })
                          ->speed;

  if (realMaxSpeed > attainableMaxSpeed) {
    for (auto& module : states) {
      module.speed = module.speed / realMaxSpeed * attainableMaxSpeed;
    }
  }
}

}  // namespace frc