#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "tigertronics/SwerveDrivetrain.h"
#include "units/units.h"
#include "Constants.h"


class SwerveController {
public:
    SwerveController();
    void CmdVelCallback(const geometry_msgs::Twist& msg);

private:

    units::meters_per_second_t kMeterPerSecond = 1.0_mps;
    units::radians_per_second_t kRadianPerSecond{1.0};
    
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    std::string _cmd_vel_topic;
    float _linx, _liny, _linz;
    float _angx, _angy, _angz;
    
    SwerveDrivetrain m_swerve;
    
};
