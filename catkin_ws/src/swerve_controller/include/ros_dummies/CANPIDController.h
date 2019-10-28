#pragma once

#include "ros_dummies/CANEncoder.h"
#include "ros_dummies/ControlType.h"

namespace rev {

class CANSparkMax;

class CANPIDController {

public:
    enum class AccelStrategy { kTrapezoidal = 0, kSCurve = 1 };

    enum class ArbFFUnits { kVoltage = 0, kPercentOut = 1 };

    explicit CANPIDController(CANSparkMax& device);

    CANPIDController(CANPIDController&&) = default;
    CANPIDController& operator=(CANPIDController&&) = default;
    
    CANError SetP(double gain, int slotID = 0);
    CANError SetI(double gain, int slotID = 0);
    CANError SetD(double gain, int slotID = 0);
    CANError SetOutputRange(double min, double max, int slotID = 0);
    
private:
    CANSparkMax* _device;
};

}  // namespace rev
