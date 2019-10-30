#include "swerve_controller/swerve_controller.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "swerve_controller");
    SwerveController swervecontroller1{ tigertronics::ports::swerveFLDrive, tigertronics::ports::swerveFLTurn };
    SwerveController swervecontroller2{ tigertronics::ports::swerveFRDrive, tigertronics::ports::swerveFRTurn };
    SwerveController swervecontroller3{ tigertronics::ports::swerveBLDrive, tigertronics::ports::swerveBLTurn };
    SwerveController swervecontroller4{ tigertronics::ports::swerveBRDrive, tigertronics::ports::swerveBRTurn };
    
    frc::AHRS m_imu{frc::SPI::Port::kMXP};
    ros::spin();
    return 0;
}


SwerveController::SwerveController(int driveMotorPort, int turnMotorPort)
    : m_driveMotor(driveMotorPort, rev::CANSparkMaxLowLevel::MotorType::kBrushless)
    , m_drivePIDController(m_driveMotor.GetPIDController())
    , m_driveEncoder(m_driveMotor.GetEncoder())
    , m_turnMotor(turnMotorPort) {
    std::string temp1 = "cmd_vel_param";
    std::string temp2 = "/cmd_vel";
    _nh.param<std::string>(temp1, _cmd_vel_topic, temp2);
    _sub = _nh.subscribe(_cmd_vel_topic, 1, &SwerveController::CmdVelCallback, this);
}

void SwerveController::CmdVelCallback(const geometry_msgs::Twist& msg) {
    _linx = msg.linear.x;
    _liny = msg.linear.y;
    _linz = msg.linear.z;
    _angx = msg.angular.x;
    _angy = msg.angular.y;
    _angz = msg.angular.z;
    
    m_driveMotor.Set(_linx);
    m_turnMotor.Set(_angz);
    
    ROS_INFO("Vel Received: [%f], [%f], [%f]", _linx, _liny, _angz);
}
