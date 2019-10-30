#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "ros_dummies/CANSparkMax.h"
#include "ros_dummies/TalonSRX.h"
#include "ros_dummies/AHRS.h"
#include "Constants.h"


class SwerveController {
public:
    SwerveController(int driveMotorPort, int turnMotorPort);
    void CmdVelCallback(const geometry_msgs::Twist& msg);

private:
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    std::string _cmd_vel_topic;
    float _linx, _liny, _linz;
    float _angx, _angy, _angz;
    
    rev::CANSparkMax m_driveMotor;
    rev::CANPIDController m_drivePIDController;
    rev::CANEncoder m_driveEncoder;
    ctre::phoenix::motorcontrol::can::TalonSRX m_turnMotor;
    
};
