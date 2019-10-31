#include "ros_dummies/CANPIDController.h"
#include "ros_dummies/CANSparkMax.h"

using namespace rev;

CANPIDController::CANPIDController(CANSparkMax& device) : _device(&device) {
    
}

CANError CANPIDController::SetP(double gain, int slotID) {
    _device->SetP(gain);
    return CANError::kOk;
}

CANError CANPIDController::SetI(double gain, int slotID) {
    _device->SetI(gain);
    return CANError::kOk;
}

CANError CANPIDController::SetD(double gain, int slotID) {
    _device->SetD(gain);
    return CANError::kOk;
}

CANError CANPIDController::SetOutputRange(double min, double max, int slotID) {
    _device->SetOutputRange(min, max);
    return CANError::kOk;
}
