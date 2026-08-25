#include "slider_types.h"

#include <string.h>

namespace slider {

const char* motionModeName(MotionMode value) {
  switch (value) {
    case MotionMode::kDisabled: return "DISABLED";
    case MotionMode::kIdle: return "IDLE";
    case MotionMode::kMoving: return "MOVING";
    case MotionMode::kHomingMin: return "HOMING_MIN";
    case MotionMode::kBackoffMin: return "BACKOFF_MIN";
    case MotionMode::kHomingMax: return "HOMING_MAX";
    case MotionMode::kBackoffMax: return "BACKOFF_MAX";
    case MotionMode::kFaulted: return "FAULTED";
  }
  return "UNKNOWN";
}

const char* faultCodeName(FaultCode value) {
  switch (value) {
    case FaultCode::kNone: return "NONE";
    case FaultCode::kHomeFailed: return "HOME_FAILED";
    case FaultCode::kEncoderHardStop: return "ENCODER_HARD_STOP";
    case FaultCode::kEncoderFault: return "ENCODER_FAULT";
    case FaultCode::kTmcComm: return "TMC_COMM";
    case FaultCode::kTmcDriver: return "TMC_DRIVER";
    case FaultCode::kPowerLoss: return "POWER_LOSS";
    case FaultCode::kTravelLimit: return "TRAVEL_LIMIT";
    case FaultCode::kUnexpectedStall: return "UNEXPECTED_STALL";
  }
  return "UNKNOWN";
}

const char* faultReasonName(FaultReason value) {
  switch (value) {
    case FaultReason::kNone: return "NONE";
    case FaultReason::kTimeout: return "TIMEOUT";
    case FaultReason::kDistanceExceeded: return "DISTANCE_EXCEEDED";
    case FaultReason::kBackoffTimeout: return "BACKOFF_TIMEOUT";
    case FaultReason::kInvalidDiag: return "INVALID_DIAG";
    case FaultReason::kTravelTooShort: return "TRAVEL_TOO_SHORT";
    case FaultReason::kReadFailed: return "READ_FAILED";
    case FaultReason::kVersionMismatch: return "VERSION_MISMATCH";
    case FaultReason::kIfcntFailed: return "IFCNT_FAILED";
    case FaultReason::kOverTemperature: return "OVER_TEMPERATURE";
    case FaultReason::kShortCircuit: return "SHORT_CIRCUIT";
    case FaultReason::kPowerGoodLost: return "POWER_GOOD_LOST";
    case FaultReason::kSoftLimitCrossed: return "SOFT_LIMIT_CROSSED";
    case FaultReason::kEncoderMismatch: return "ENCODER_MISMATCH";
  }
  return "UNKNOWN";
}

const char* standstillModeName(StandstillMode value) {
  switch (value) {
    case StandstillMode::kNormal: return "NORMAL";
    case StandstillMode::kFreewheeling: return "FREEWHEELING";
    case StandstillMode::kBraking: return "BRAKING";
    case StandstillMode::kStrongBraking: return "STRONG_BRAKING";
  }
  return "UNKNOWN";
}

bool parseStandstillMode(const char* value, StandstillMode& output) {
  if (value == nullptr) return false;
  if (strcmp(value, "NORMAL") == 0) output = StandstillMode::kNormal;
  else if (strcmp(value, "FREEWHEELING") == 0) output = StandstillMode::kFreewheeling;
  else if (strcmp(value, "BRAKING") == 0) output = StandstillMode::kBraking;
  else if (strcmp(value, "STRONG_BRAKING") == 0) output = StandstillMode::kStrongBraking;
  else return false;
  return true;
}

}  // namespace slider
