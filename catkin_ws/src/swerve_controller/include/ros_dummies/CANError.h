#pragma once

namespace rev {

enum class CANError {
    kOk = 0,
    kError,
    kTimeout,
    kNotImplmented,
    kHALError,
    kCantFindFirmware,
    kFirmwareTooOld,
    kFirmwareTooNew,
    kParamInvalidID,
    kParamMismatchType,
    kParamAccessMode,
    kParamInvalid,
    kParamNotImplementedDeprecated,
    kFollowConfigMismatch,
    kInvalid,
    kSetpointOutOfRange,
};

}  // namespace rev
