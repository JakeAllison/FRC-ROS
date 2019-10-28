#include "AHRS.h"
#include <tf/tf.h>

namespace frc{

AHRS::AHRS(frc::spi::Port port)
   : _yawOffset(0.0)
   , _yaw(0.0)
   , _pitch(0.0)
   , _roll(0.0)
   , _yawRate(0.0)
   , _pitchRate(0.0)
   , _rollRate(0.0)
   , _quaternionW(0.0)
   , _quaternionX(0.0)
   , _quaternionY(0.0)
   , _quaternionZ(0.0) {
   
    std::string temp1 = "imu_topic";
    std::string temp2 = "/imu";
    _nh.param<std::string>(temp1, _imuTopic, temp2);
    _sub = _nh.subscribe(_imu_topic, 1000, imuCallback);
}

void AHRS::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    _quaternionW = msg->orientation.w;
    _quaternionX = msg->orientation.x;
    _quaternionY = msg->orientation.y;
    _quaternionZ = msg->orientation.z;
    
    tf::Quaternion q(_quaternionW, _quaternionX, _quaternionY, _quaternionZ);
    tf::Matrix3x3 m(q);
    m.getRPY(_roll, _pitch, _yaw);
    
    _rollRate   = msg->angular_velocity.x;
    _pitchRate  = msg->angular_velocity.y;
    _yawRate    = msg->angular_velocity.z;
}

float AHRS::GetPitch() {
    return _pitch;
}
float AHRS::GetRoll() {
    return _roll;
}
float AHRS::GetYaw() {
    return _yaw + _yawOffset;
}

float AHRS::GetPitchRate() {
    return _pitchRate;
}
float AHRS::GetRollRate() {
    return _rollRate;
}
float AHRS::GetYawRate() {
    return _yawRate;
}

void AHRS::ZeroYaw() {
    _yawOffset = -_yaw;
    _yaw = 0.0;
}

float AHRS::GetQuaternionW() {
    return _quaternionW;
}
float AHRS::GetQuaternionX() {
    return _quaternionX;
}
float AHRS::GetQuaternionY() {
    return _quaternionY;
}
float AHRS::GetQuaternionZ() {
    return _quaternionY;
}

void AHRS::Reset() {
    ZeroYaw();
}

}

