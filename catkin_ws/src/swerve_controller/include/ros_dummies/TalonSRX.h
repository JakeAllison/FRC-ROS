#include "CANSparkMax.h"

namespace ctre {
namespace phoenix {
enum class ErrorCode {
    kOk = 0
};

namespace motorcontrol {
namespace can {
class TalonSRX : public rev::CANSparkMax {
public:
	TalonSRX(int deviceNumber);
    ctre::phoenix::ErrorCode ConfigFactoryDefault(int timeoutMs = 50);
	ctre::phoenix::ErrorCode Config_kF(int slotIdx, double value, int timeoutMs = 0);
	ctre::phoenix::ErrorCode Config_kP(int slotIdx, double value, int timeoutMs = 0);
	ctre::phoenix::ErrorCode Config_kI(int slotIdx, double value, int timeoutMs = 0);
	ctre::phoenix::ErrorCode Config_kD(int slotIdx, double value, int timeoutMs = 0);
	
private:

};

} // can
} // motorcontrol
} // phoenix
} // ctre
