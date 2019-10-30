#include "ros_dummies/TalonSRX.h"

namespace ctre {
namespace phoenix {
namespace motorcontrol {
namespace can {

TalonSRX::TalonSRX(int deviceNumber)
    : CANSparkMax(deviceNumber) {
    
}

ctre::phoenix::ErrorCode TalonSRX::ConfigFactoryDefault(int timeoutMs) {
    RestoreFactoryDefaults();
    return ctre::phoenix::ErrorCode::kOk;
}

ctre::phoenix::ErrorCode TalonSRX::Config_kF(int slotIdx, double value, int timeoutMs) {
    SetF(value);
    return ctre::phoenix::ErrorCode::kOk;
}

ctre::phoenix::ErrorCode TalonSRX::Config_kP(int slotIdx, double value, int timeoutMs) {
    SetP(value);
    return ctre::phoenix::ErrorCode::kOk;
}

ctre::phoenix::ErrorCode TalonSRX::Config_kI(int slotIdx, double value, int timeoutMs) {
    SetI(value);
    return ctre::phoenix::ErrorCode::kOk;
}

ctre::phoenix::ErrorCode TalonSRX::Config_kD(int slotIdx, double value, int timeoutMs) {
    SetD(value);
    return ctre::phoenix::ErrorCode::kOk;
}

} // can
} // motorcontrol
} // phoenix
} // ctre
