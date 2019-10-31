#include "ros_dummies/CANEncoder.h"
#include "ros_dummies/CANSparkMax.h"

using namespace rev;

CANEncoder::CANEncoder(CANSparkMax& device, EncoderType sensorType, int cpr) 
    : m_sensorType(sensorType)
    , _device(&device) {
    
}

CANEncoder::CANEncoder(CANEncoder&& rhs)
    : m_sensorType(std::move(rhs.m_sensorType)),
    m_cpr(std::move(rhs.m_cpr)),
    encInitialized(rhs.encInitialized.load()) {}

CANEncoder& CANEncoder::operator=(CANEncoder&& rhs) {
  _device = std::move(rhs._device);
  m_sensorType = std::move(rhs.m_sensorType);
  m_cpr = std::move(rhs.m_cpr);
  encInitialized = rhs.encInitialized.load();

  return *this;
}

CANError CANEncoder::SetInverted(bool inverted)  {
    _device->SetEncoderInverted(inverted);
    return CANError::kOk;
}

CANError CANEncoder::SetPosition(double position) {
    _device->SetEncoderPosition(position);
    return CANError::kOk;
}

bool CANEncoder::GetInverted() const {
    return _device->GetEncoderInverted();
}

int CANEncoder::GetCPR() const {
    return m_cpr;
}

double CANEncoder::GetPosition() const {
    return static_cast<double>(_device->GetEncoderPosition());
}

double CANEncoder::GetVelocity() const {
    return static_cast<double>(_device->GetEncoderVelocity());
}
