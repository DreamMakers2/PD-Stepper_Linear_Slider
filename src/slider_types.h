#pragma once

#include <stdint.h>

namespace slider {

enum class MotionMode : uint8_t {
  kDisabled,
  kIdle,
  kMoving,
  kHomingMin,
  kBackoffMin,
  kHomingMax,
  kBackoffMax,
  kFaulted,
};

enum class FaultCode : uint8_t {
  kNone,
  kHomeFailed,
  kEncoderHardStop,
  kEncoderFault,
  kTmcComm,
  kTmcDriver,
  kPowerLoss,
  kTravelLimit,
  kUnexpectedStall,
};

enum class FaultReason : uint8_t {
  kNone,
  kTimeout,
  kDistanceExceeded,
  kBackoffTimeout,
  kInvalidDiag,
  kTravelTooShort,
  kReadFailed,
  kVersionMismatch,
  kIfcntFailed,
  kOverTemperature,
  kShortCircuit,
  kPowerGoodLost,
  kSoftLimitCrossed,
  kEncoderMismatch,
};

enum class StandstillMode : uint8_t {
  kNormal = 0,
  kFreewheeling = 1,
  kBraking = 2,
  kStrongBraking = 3,
};

enum class CommandType : uint8_t {
  kHome,
  kMove,
  kVelocity,
  kStop,
  kEnable,
  kDisable,
  kResetFault,
};

struct RuntimeConfig {
  uint8_t pd_voltage_v = 15;
  uint16_t run_current_ma = 800;
  uint16_t microsteps = 4;
  uint8_t stallguard_threshold = 20;
  StandstillMode standstill_mode = StandstillMode::kNormal;
  bool invert_direction = false;
  bool invert_encoder = false;
};

struct Command {
  CommandType type = CommandType::kStop;
  float position_mm = 0.0F;
  float velocity_mm_s = 0.0F;
  float speed_mm_s = 40.0F;
  float acceleration_mm_s2 = 75.0F;
};

struct StateSnapshot {
  MotionMode mode = MotionMode::kDisabled;
  FaultCode fault = FaultCode::kNone;
  FaultReason fault_reason = FaultReason::kNone;
  bool enabled = false;
  bool homed = false;

  float position_mm = 0.0F;
  float commanded_position_mm = 0.0F;
  float target_position_mm = 0.0F;
  float velocity_mm_s = 0.0F;
  float physical_min_mm = 0.0F;
  float physical_max_mm = 0.0F;
  float soft_min_mm = 0.0F;
  float soft_max_mm = 0.0F;

  bool encoder_valid = false;
  uint16_t encoder_raw = 0;
  int32_t encoder_unwrapped_counts = 0;
  int32_t encoder_min_counts = 0;
  int32_t encoder_max_counts = 0;
  int32_t executed_microsteps = 0;

  bool power_good = false;
  float vbus_voltage = 0.0F;
  bool diag = false;
  uint16_t stallguard_result = 0;

  bool tmc_uart_ok = false;
  uint8_t tmc_ifcnt = 0;
  uint8_t tmc_version = 0;
  uint32_t tmc_ioin_raw = 0;
  uint8_t tmc_consecutive_failures = 0;
  uint32_t tmc_last_ok_ms = 0;
  FaultReason tmc_last_failure_reason = FaultReason::kNone;
  uint32_t tmc_status_raw = 0;
};

const char* motionModeName(MotionMode value);
const char* faultCodeName(FaultCode value);
const char* faultReasonName(FaultReason value);
const char* standstillModeName(StandstillMode value);
bool parseStandstillMode(const char* value, StandstillMode& output);

}  // namespace slider
