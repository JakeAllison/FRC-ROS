#ifndef CANSPARKMAX_H
#define CANSPARKMAX_H

#include "ros_dummies/CANEncoder.h"
#include "ros_dummies/CANPIDController.h"
#include "Constants.h"

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <vector>
#include <unordered_map>


namespace rev {
    namespace CANSparkMaxLowLevel {
        enum class MotorType { kBrushed = 0, kBrushless = 1 };
    }
    
class CANSparkMax {
public:
    
    explicit CANSparkMax(int deviceID, CANSparkMaxLowLevel::MotorType type = CANSparkMaxLowLevel::MotorType::kBrushless);
    
    void JointStateCallback(const sensor_msgs::JointState& msg);

    CANPIDController GetPIDController();
    CANEncoder GetEncoder(CANEncoder::EncoderType sensorType = CANEncoder::EncoderType::kHallSensor, int cpr = 0);
    
    void RestoreFactoryDefaults();
    
    void Set(double setpoint);
    void SetInverted(bool inverted);
    double Get() const;
    bool GetInverted() const;
    
    void SetEncoderInverted(bool inverted);
    void SetEncoderPosition(double position);
    bool GetEncoderInverted() const;
    double GetEncoderOffset() const;
    double GetEncoderPosition() const;
    double GetEncoderVelocity() const;
    double GetEncoderRawOffset() const;
    double GetEncoderRawPosition() const;
    double GetEncoderRawVelocity() const;
    
    void SetF(double gain);
    void SetP(double gain);
    void SetI(double gain);
    void SetD(double gain);
    void SetOutputRange(double min, double max);

private:
    
    // To do: Need callback functions for incoming topics.
    
    int _deviceID;
    double _setpoint, _rawSetpoint;
    bool _inverted;
    
    bool _encoder_inverted;
    double _encoder_raw_position, _encoder_raw_velocity, _encoder_raw_offset;
    double _motor_effort;
    
    double _kf, _kp, _ki, _kd;
    double _min, _max;
    
    ros::NodeHandle _nh;
    ros::Publisher _command_publisher;
    ros::Subscriber _joint_state_subscriber;
    
    std_msgs::Float64 _command_value;
    std::string _command_topic;
    
    sensor_msgs::JointState _joint_states;
    std::string _joint_state_topic;
    std::string _joint_name;
    double _drive_conversion;
};

}  // namespace rev

#endif
