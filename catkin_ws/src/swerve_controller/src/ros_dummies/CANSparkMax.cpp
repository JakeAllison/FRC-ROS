// For converting adjusted to raw:
// -> raw = (+/- adjusted) - raw offset
//
// For converting raw to adjusted:
// -> adjusted = +/-(raw + raw offset)
//
// To calculate the raw offset from desired position:
// -> raw offset = (+/- new position) - raw
//
// 

#include "ros_dummies/CANSparkMax.h"

#define M_PI 3.14159265358979323846

using namespace rev;
using namespace rev::CANSparkMaxLowLevel;

std::unordered_map<int, std::string> PARAMETER_PREFIXES = {
    {tigertronics::ports::swerveFLTurn, "fl_pivot"},
    {tigertronics::ports::swerveFLDrive, "fl_drive"},
    {tigertronics::ports::swerveFRTurn, "fr_pivot"},
    {tigertronics::ports::swerveFRDrive, "fr_drive"},
    {tigertronics::ports::swerveBLTurn, "rl_pivot"},
    {tigertronics::ports::swerveBLDrive, "rl_drive"},
    {tigertronics::ports::swerveBRTurn, "rr_pivot"},
    {tigertronics::ports::swerveBRDrive, "rr_drive"}
}

CANSparkMax::CANSparkMax(int deviceID, MotorType type)
    : _deviceID(deviceID)
    , _setpoint(0.0)
    , _inverted(false)
    , _encoder_inverted(false)
    , _encoder_raw_position(0.0)
    , _encoder_raw_velocity(0.0)
    , _encoder_raw_offset(0.0)
    , _kf(0.0)
    , _kp(0.0)
    , _ki(0.0)
    , _kd(0.0)
    , _min(0.0)
    , _max(0.0)
{
    std::string temp1 = PARAMETER_PREFIXES[_deviceID] + "_command_param";
    std::string temp2 = "/drivetrain/" + PARAMETER_PREFIXES[_deviceID] + "_controller/command";
    
    std::string temp3 = PARAMETER_PREFIXES[_deviceID] + "_conversion_param";
    double temp4 = 1.0;
    
    _nh.param<std::string>(temp1, _command_topic, temp2);
    _nh.param<double>(temp3, _drive_conversion, temp4);
    
    if (_drive_conversion == 0.0) {
        _nh.setParam(temp3, temp4);
    }
    
    _command_publisher = _nh.advertise<std_msgs::Float64>(_command_topic, 2, true);
}


CANPIDController CANSparkMax::GetPIDController() {
    return CANPIDController{*this};
}


CANEncoder CANSparkMax::GetEncoder(CANEncoder::EncoderType sensorType, int cpr) { 
    return CANEncoder{*this, sensorType, cpr};
}

void CANSparkMax::RestoreRestoreFactoryDefaults() {
    
};

void CANSparkMax::Set(double setpoint) {
    _setpoint = (_inverted ? -setpoint : setpoint) + _raw_offset;
    _command_value = _setpoint * _drive_conversion;
    _command_publisher.publish(_command_value);
}

void CANSparkMax::SetInverted(bool inverted) { 
    _inverted = inverted;
}

void CANSparkMax::SetEncoderInverted(bool inverted) {
    _encoder_inverted = inverted;
}

void CANSparkMax::SetEncoderPosition(double position) {
    double temp = _encoder_inverted ? -position : position;
    _encoder_raw_offset = temp - _encoder_raw_position;
};



double CANSparkMax::Get() const {
    return _setpoint;
}

bool CANEncoder::GetInverted() const {
    return _inverted;
}

bool CANSparkMax::GetEncoderInverted() const { 
    return _encoder_inverted;
}

double CANSparkMax::GetEncoderOffset() const { 
    return _encoder_inverted ? -_encoder_raw_offset : _encoder_raw_offset;
}

double CANSparkMax::GetEncoderPosition() {
    double temp = GetEncoderRawPosition() + _encoder_raw_offset;
    return _encoder_inverted ? -temp : temp;
};

double CANSparkMax::GetEncoderVelocity() {
    return _encoder_inverted ? -GetEncoderRawVelocity() : GetEncoderRawVelocity();
};

double CANSparkMax::GetEncoderRawOffset() const { 
    return _encoder_raw_offset;
}

double CANSparkMax::GetEncoderRawPosition() {
    return _encoder_raw_position;
};
double CANSparkMax::GetEncoderRawVelocity() {
    return _encoder_raw_velocity;
};

void CANSparkMax::SetF(double gain) {
    _kf = gain
};

void CANSparkMax::SetP(double gain) {
    _kp = gain
};

void CANSparkMax::SetI(double gain) {
    _ki = gain
};

void CANSparkMax::SetD(double gain) {
    _kd = gain
};

void CANSparkMax::SetOutputRange(double min, double max) {
    _min = min;
    _max = max
};
