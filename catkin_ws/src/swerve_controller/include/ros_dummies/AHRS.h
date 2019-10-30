
#ifndef SRC_AHRS_H_
#define SRC_AHRS_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace frc {
namespace SPI {
    enum Port {
        kMXP
    };
} // spi
} // frc


using namespace frc;

class AHRS {
public:
    AHRS(frc::SPI::Port port);
    
    void    imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void   Reset();
    void   ZeroYaw();
    double  GetPitch();
    double  GetRoll();
    double  GetYaw();
    double  GetPitchRate();
    double  GetRollRate();
    double  GetYawRate();
    double  GetQuaternionW();
    double  GetQuaternionX();
    double  GetQuaternionY();
    double  GetQuaternionZ();

private:
    double   _yawOffset;
    double   _pitch;
    double   _roll;
    double   _yaw;
    double   _yawRate;
    double   _pitchRate;
    double   _rollRate;
    double   _quaternionW;
    double   _quaternionX;
    double   _quaternionY;
    double   _quaternionZ;
    
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    std::string _imuTopic;
};

#endif /* SRC_AHRS_H_ */
