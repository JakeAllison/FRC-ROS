
#ifndef SRC_AHRS_H_
#define SRC_AHRS_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace frc {
namespace spi {
    enum Port {
        kMXP
    }
} // spi

class AHRS {
public:
    AHRS(frc::spi::Port port);
    
    float  GetPitch();
    float  GetRoll();
    float  GetYaw();
    float  GetPitchRate();
    float  GetRollRate();
    float  GetYawRate();
    void   ZeroYaw();
    float  GetQuaternionW();
    float  GetQuaternionX();
    float  GetQuaternionY();
    float  GetQuaternionZ();
    void   Reset();

private:
    void    imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    float   _yawOffset;
    float   _pitch;
    float   _roll;
    float   _yaw;
    float   _yawRate;
    float   _pitchRate;
    float   _rollRate;
    float   _quaternionW;
    float   _quaternionX;
    float   _quaternionY;
    float   _quaternionZ;
    
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    std::string _imuTopic;
};

} // frc

#endif /* SRC_AHRS_H_ */
