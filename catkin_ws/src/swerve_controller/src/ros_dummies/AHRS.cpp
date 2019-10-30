#include "ros_dummies/AHRS.h"
#include <tf/tf.h>

namespace frc{

AHRS::AHRS(frc::SPI::Port port)
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
    _sub = _nh.subscribe(_imuTopic, 1, &AHRS::imuCallback, this);
}

void AHRS::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    _quaternionW = msg->orientation.w;
    _quaternionX = msg->orientation.x;
    _quaternionY = msg->orientation.y;
    _quaternionZ = msg->orientation.z;
    
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation, q);
    tf::Matrix3x3(q).getRPY(_roll, _pitch, _yaw);
    
    _rollRate   = msg->angular_velocity.x;
    _pitchRate  = msg->angular_velocity.y;
    _yawRate    = msg->angular_velocity.z;
    
    ROS_INFO("IMU Received: [%f]", _yaw);
}

void AHRS::Reset() {
    ZeroYaw();
}

void AHRS::ZeroYaw() {
    _yawOffset = -_yaw;
    _yaw = 0.0;
}

double AHRS::GetPitch() {
    return _pitch;
}
double AHRS::GetRoll() {
    return _roll;
}
double AHRS::GetYaw() {
    return _yaw + _yawOffset;
}

double AHRS::GetPitchRate() {
    return _pitchRate;
}
double AHRS::GetRollRate() {
    return _rollRate;
}
double AHRS::GetYawRate() {
    return _yawRate;
}

double AHRS::GetQuaternionW() {
    return _quaternionW;
}
double AHRS::GetQuaternionX() {
    return _quaternionX;
}
double AHRS::GetQuaternionY() {
    return _quaternionY;
}
double AHRS::GetQuaternionZ() {
    return _quaternionY;
}

}

