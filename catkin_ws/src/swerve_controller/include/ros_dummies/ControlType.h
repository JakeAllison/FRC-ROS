#pragma once

namespace rev {

enum ControlType {
    kDutyCycle = 0,
    kVelocity = 1,
    kVoltage = 2,
    kPosition = 3,
    kSmartMotion = 4,
    kCurrent = 5,
    kSmartVelocity = 6
};

}  // namespace rev
