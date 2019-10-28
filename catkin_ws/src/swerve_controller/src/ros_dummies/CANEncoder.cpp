#include "ros_dummies/CANEncoder.h"

using namespace rev;

CANEncoder::CANEncoder(CANSparkMax& device, EncoderType sensorType, int cpr) 
    : m_sensorType(sensorType)
    , m_device(device) {
    
}

CANEncoder::CANEncoder(CANEncoder&& rhs)
    : m_sensorType(std::move(rhs.m_sensorType)),
    m_cpr(std::move(rhs.m_cpr)),
    encInitialized(rhs.encInitialized.load()) {}

CANEncoder& CANEncoder::operator=(CANEncoder&& rhs) {
  CANSensor::operator=(std::move(rhs));

  m_device = std::move(rhs.m_device);
  m_sensorType = std::move(rhs.m_sensorType);
  m_cpr = std::move(rhs.m_cpr);
  encInitialized = rhs.encInitialized.load();

  return *this;
}

CANError CANEncoder::SetInverted(bool inverted)  {
    m_device->SetEncoderInverted(inverted);
    return CANError::kOk;
}

CANError CANEncoder::SetPosition(double position) {
    m_device->SetEncoderPosition(position);
    return CANError::kOk;
}

bool CANEncoder::GetInverted() const {
    return m_device->GetEncoderInverted();
}

int CANEncoder::GetCPR() const {
    return m_cpr;
}

double CANEncoder::GetPosition() {
    return static_cast<double>(m_device->GetEncoderPosition());
}

double CANEncoder::GetVelocity() {
    return return static_cast<double>(m_device->GetEncoderVelocity());
}
