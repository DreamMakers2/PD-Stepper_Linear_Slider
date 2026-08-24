#pragma once

#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Preferences.h>
#include <TMCStepper.h>
#include <Wire.h>

#include "slider_core.h"
#include "slider_types.h"

namespace slider {

class MotionController {
 public:
  bool begin();
  void tick();

  bool submitCommand(const Command& command, const char*& error_code);
  bool submitConfig(const RuntimeConfig& config, const char*& error_code);
  StateSnapshot snapshot() const;
  RuntimeConfig config() const;

 private:
  static constexpr float kSoftMarginMm = 5.0F;
  static constexpr float kMaxNormalSpeedMmS = 40.0F;
  static constexpr float kMaxNormalAccelerationMmS2 = 75.0F;
  static constexpr float kHomingSpeedMmS = 40.0F;
  static constexpr float kHomingAccelerationMmS2 = 75.0F;
  static constexpr float kHomingDistanceLimitMm = 500.0F;
  static constexpr uint32_t kHomingTimeoutMs = 15000;
  static constexpr uint32_t kBackoffTimeoutMs = 2000;
  static constexpr uint32_t kEncoderSampleMs = 5;
  static constexpr uint8_t kEncoderFailureThreshold = 5;
  static constexpr uint32_t kTelemetrySampleMs = 500;
  static constexpr uint32_t kTmcProbeMs = 500;
  static constexpr uint8_t kExpectedTmcVersion = 0x21;
  static constexpr float kVbusDividerRatio = 0.1189427313F;

  static void IRAM_ATTR diagIsr(void* argument);
  static void IRAM_ATTR powerGoodIsr(void* argument);
  void IRAM_ATTR emergencyIsr(volatile bool& flag);

  void initializePins();
  bool initializeStepper();
  void initializeTmc();
  void loadConfig();
  void saveConfig();
  bool validateConfig(const RuntimeConfig& config) const;
  void applyPdVoltage(uint8_t voltage);
  void applyTmcConfig(const RuntimeConfig& config);
  void applyHomingConfig();
  void setDirectionPolarity();

  bool readEncoder();
  bool captureEncoderAfterStop();
  bool samplePowerGood();
  void updateFastTelemetry(uint32_t now);
  void updateSlowTelemetry(uint32_t now);
  void probeTmc(uint32_t now);
  float encoderPositionMm() const;
  float commandedPositionMm() const;
  int8_t currentMotionDirection() const;
  void setSynchronizationAnchor(float logical_position_mm);
  float positionFromSynchronizationAnchor() const;
  void resynchronizeStepperFromEncoder(float override_position_mm, bool use_override);

  void processEmergencyEvents();
  void processUrgentRequests();
  void processPendingRequest();
  void processMotion(uint32_t now);
  void processRollingHardStop(uint32_t now);

  bool enableDriver();
  void handlePowerLoss();
  void stopMotionAndResynchronize();
  void disableDriver(bool invalidate_home);
  void enterFault(FaultCode code, FaultReason reason);
  void clearFault();

  bool startHome();
  void startHomingLeg(MotionMode mode, int8_t direction);
  void handleHomingDiag();
  void startBackoff(MotionMode mode, float target_mm);
  void finishHoming();
  void startPositionMove(const Command& command);
  void startVelocityMove(const Command& command);
  void setMotionProfile(float speed_mm_s, float acceleration_mm_s2);

  void updateSnapshot();
  bool isHoming() const;
  bool isNormalMotion() const;
  bool isMoving() const;

  mutable portMUX_TYPE data_mux_ = portMUX_INITIALIZER_UNLOCKED;
  StateSnapshot state_{};
  RuntimeConfig config_{};
  RuntimeConfig published_config_{};
  Preferences preferences_{};

  FastAccelStepperEngine engine_{};
  FastAccelStepper* stepper_ = nullptr;
  TMC2209Stepper tmc_{&Serial2, 0.100F, 0};

  core::EncoderUnwrapper encoder_unwrapper_{};
  core::RollingMotionMonitor motion_monitor_{};
  core::SynchronizationAnchor sync_anchor_{};
  bool sync_anchor_valid_ = false;

  volatile bool diag_event_ = false;
  volatile bool power_event_ = false;
  volatile bool motion_armed_ = false;
  bool diag_requires_clear_ = false;
  volatile bool urgent_stop_ = false;
  volatile bool urgent_disable_ = false;

  bool pending_command_ = false;
  Command command_{};
  bool pending_config_ = false;
  RuntimeConfig requested_config_{};

  MotionMode mode_ = MotionMode::kDisabled;
  FaultCode fault_ = FaultCode::kNone;
  FaultReason fault_reason_ = FaultReason::kNone;
  volatile bool driver_enabled_ = false;
  bool homed_ = false;

  bool encoder_valid_ = false;
  uint16_t encoder_raw_ = 0;
  int32_t encoder_counts_ = 0;
  int32_t encoder_min_counts_ = 0;
  int32_t encoder_max_counts_ = 0;
  int8_t calibrated_encoder_sign_ = 1;
  float travel_mm_ = 0.0F;
  float target_position_mm_ = 0.0F;
  float requested_velocity_mm_s_ = 0.0F;

  bool power_good_ = false;
  float vbus_voltage_ = 0.0F;
  bool diag_state_ = false;
  uint16_t stallguard_result_ = 0;
  uint32_t tmc_status_raw_ = 0;
  bool tmc_uart_ok_ = false;
  uint8_t tmc_ifcnt_ = 0;
  uint8_t tmc_version_ = 0;
  uint32_t tmc_ioin_raw_ = 0;
  uint8_t tmc_probe_failures_ = 0;
  uint32_t tmc_last_ok_ms_ = 0;
  FaultReason last_tmc_comm_reason_ = FaultReason::kNone;

  uint16_t active_microsteps_ = 4;
  uint32_t motion_started_ms_ = 0;
  int32_t motion_started_step_ = 0;
  uint32_t diag_arm_after_ms_ = 0;
  uint32_t last_encoder_sample_ms_ = 0;
  uint32_t last_motion_monitor_ms_ = 0;
  uint32_t last_telemetry_sample_ms_ = 0;
  uint32_t last_tmc_probe_ms_ = 0;
  uint8_t encoder_read_failures_ = 0;
};

}  // namespace slider
