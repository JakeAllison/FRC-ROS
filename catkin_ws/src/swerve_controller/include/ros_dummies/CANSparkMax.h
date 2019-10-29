#ifndef CANSPARKMAX_H
#define CANSPARKMAX_H

#include "ros_dummies/CANEncoder.h"
#include "ros_dummies/CANPIDController.h"
#include "Constants.h"

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <string>
#include <unordered_map>


namespace rev {
    namespace CANSparkMaxLowLevel {
        enum class MotorType { kBrushed = 0, kBrushless = 1 };
    }
    
class CANSparkMax {
public:
    
    explicit CANSparkMax(int deviceID, CANSparkMaxLowLevel::MotorType type = CANSparkMaxLowLevel::MotorType::kBrushless);
    ~CANSparkMax();

    CANPIDController GetPIDController();
    CANEncoder GetEncoder(CANEncoder::EncoderType sensorType = CANEncoder::EncoderType::kHallSensor, int cpr = 0);
    
    void RestoreRestoreFactoryDefaults() {};
    
    void Set(double setpoint);
    void SetInverted(bool inverted);
    double Get() const;
    bool GetInverted();
    
    void SetEncoderInverted(bool inverted);
    void SetEncoderPosition(double position);
    bool GetEncoderInverted();
    double GetEncoderOffset();
    double GetEncoderPosition();
    double GetEncoderVelocity();
    double GetEncoderRawPosition();
    double GetEncoderRawVelocity();
    
    void SetF(double gain);
    void SetP(double gain);
    void SetI(double gain);
    void SetD(double gain);
    void SetOutputRange(double min, double max);

private:
    
    // To do: Need callback functions for incoming topics.
    
    int _deviceID;
    double _setpoint;
    bool _inverted;
    
    bool _encoder_inverted;
    double _encoder_raw_position, _encoder_raw_velocity, _encoder_raw_offset;
    
    double _kf, _kp, _ki, _kd;
    double _min, _max;
    
    ros::NodeHandle _nh;
    ros::Publisher _command_publisher;
    std_msgs::Float64 _command_value;
    std::string _command_topic;
    double _drive_conversion;
};

}  // namespace rev

#endif
