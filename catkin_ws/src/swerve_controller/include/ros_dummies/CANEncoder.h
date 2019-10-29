#ifndef CANENCODER_H_
#define CANENCODER_H_

#include "ros_dummies/CANError.h"
#include "ros_dummies/CANSparkMax.h"

#include <stdint.h>
#include <atomic>
#include <utility>

namespace rev {

class CANSparkMax;

class CANEncoder {
public:

    enum class EncoderType {
        kNoSensor = 0,
        kHallSensor = 1,
        kQuadrature = 2,
        kSensorless = 3,
    };

    explicit CANEncoder(CANSparkMax& device, EncoderType sensorType, int cpr);
    CANEncoder(CANEncoder&& rhs);
    CANEncoder& operator=(CANEncoder&& rhs);
    
    CANError SetPosition(double position);
    CANError SetInverted(bool inverted);
    
    bool GetInverted() const;
    int GetCPR();
    double GetPosition();
    double GetVelocity();

private:
    EncoderType m_sensorType = EncoderType::kHallSensor;
    int m_cpr = 4096;

    std::atomic<bool> encInitialized = ATOMIC_VAR_INIT(false);


protected:
    CANSparkMax* _device;
};

}  // namespace rev

#endif
