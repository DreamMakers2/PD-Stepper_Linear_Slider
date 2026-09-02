#include "motion_controller.h"

#include <algorithm>
#include <cmath>

#include "board_pins.h"

namespace slider {
namespace {

constexpr uint8_t kAs5600Address = 0x36;
constexpr uint8_t kAs5600RawAngleRegister = 0x0C;
constexpr uint32_t kTmcSeriousFaultMask = 0x3EU;  // OT, S2G and S2VS bits.

bool isSupportedMicrosteps(uint16_t value) {
  switch (value) {
    case 1:
    case 2:
    case 4:
    case 8:
    case 16:
    case 32:
    case 64:
    case 128:
    case 256:
      return true;
    default:
      return false;
  }
}

bool canHomeFromFault(FaultCode fault) {
  return fault == FaultCode::kNone || fault == FaultCode::kHomeFailed ||
         fault == FaultCode::kEncoderHardStop ||
         fault == FaultCode::kUnexpectedStall || fault == FaultCode::kTravelLimit;
}

}  // namespace

bool MotionController::begin() {
  initializePins();
  loadConfig();
  applyPdVoltage(config_.pd_voltage_v);

  Wire.begin(pins::kEncoderSda, pins::kEncoderScl, 400000);
  analogSetPinAttenuation(pins::kVbusAdc, ADC_11db);
  readEncoder();

  Serial2.begin(115200, SERIAL_8N1, pins::kTmcRx, pins::kTmcTx);
  if (!initializeStepper()) {
    mode_ = MotionMode::kFaulted;
    fault_ = FaultCode::kTmcDriver;
    fault_reason_ = FaultReason::kReadFailed;
    updateSnapshot();
    return false;
  }

  initializeTmc();
  delay(10);
  const bool tmc_ready = checkTmc();

  attachInterruptArg(digitalPinToInterrupt(pins::kDiag), diagIsr, this, RISING);
  attachInterruptArg(digitalPinToInterrupt(pins::kPowerGood), powerGoodIsr, this, RISING);

  samplePowerGood();
  if (tmc_ready) mode_ = MotionMode::kDisabled;
  updateFastTelemetry(millis());
  updateSlowTelemetry(millis());
  updateSnapshot();
  return tmc_ready;
}

void MotionController::initializePins() {
  pinMode(pins::kTmcEnable, OUTPUT);
  digitalWrite(pins::kTmcEnable, HIGH);
  pinMode(pins::kStep, OUTPUT);
  pinMode(pins::kDirection, OUTPUT);
  pinMode(pins::kMs1, OUTPUT);
  pinMode(pins::kMs2, OUTPUT);
  pinMode(pins::kSpread, OUTPUT);
  digitalWrite(pins::kMs1, LOW);
  digitalWrite(pins::kMs2, LOW);
  digitalWrite(pins::kSpread, LOW);
  pinMode(pins::kDiag, INPUT);
  pinMode(pins::kIndex, INPUT);

  pinMode(pins::kPowerGood, INPUT);
  pinMode(pins::kPdCfg1, OUTPUT);
  pinMode(pins::kPdCfg2, OUTPUT);
  pinMode(pins::kPdCfg3, OUTPUT);
  pinMode(pins::kVbusAdc, INPUT);
  pinMode(pins::kLed1, OUTPUT);
  pinMode(pins::kLed2, OUTPUT);
  digitalWrite(pins::kLed1, LOW);
  digitalWrite(pins::kLed2, LOW);
}

bool MotionController::initializeStepper() {
  engine_.init();
  stepper_ = engine_.stepperConnectToPin(pins::kStep);
  if (stepper_ == nullptr) return false;
  setDirectionPolarity();
  stepper_->setEnablePin(pins::kTmcEnable, true);
  stepper_->setDelayToEnable(2);
  stepper_->disableOutputs();
  return true;
}

void MotionController::initializeTmc() {
  tmc_.begin();
  tmc_.pdn_disable(true);
  tmc_.mstep_reg_select(true);
  tmc_.I_scale_analog(false);
  tmc_.toff(4);
  tmc_.blank_time(24);
  tmc_.pwm_autoscale(true);
  tmc_.pwm_autograd(true);
  tmc_.VACTUAL(0);
  tmc_.intpol(true);
  applyTmcConfig(config_);
}

void MotionController::loadConfig() {
  preferences_.begin("slider", true);
  config_.pd_voltage_v = preferences_.getUChar("pd_v", config_.pd_voltage_v);
  config_.run_current_ma = preferences_.getUShort("run_ma", config_.run_current_ma);
  config_.microsteps = preferences_.getUShort("microsteps", config_.microsteps);
  config_.stallguard_threshold = preferences_.getUChar("sg", config_.stallguard_threshold);
  config_.standstill_mode = static_cast<StandstillMode>(
      preferences_.getUChar("standstill", static_cast<uint8_t>(config_.standstill_mode)));
  config_.invert_direction = preferences_.getBool("inv_dir", config_.invert_direction);
  config_.invert_encoder = preferences_.getBool("inv_enc", config_.invert_encoder);
  preferences_.end();
  if (!validateConfig(config_)) config_ = RuntimeConfig{};
  published_config_ = config_;
  active_microsteps_ = config_.microsteps;
}

void MotionController::saveConfig() {
  preferences_.begin("slider", false);
  preferences_.putUChar("pd_v", config_.pd_voltage_v);
  preferences_.putUShort("run_ma", config_.run_current_ma);
  preferences_.putUShort("microsteps", config_.microsteps);
  preferences_.putUChar("sg", config_.stallguard_threshold);
  preferences_.putUChar("standstill", static_cast<uint8_t>(config_.standstill_mode));
  preferences_.putBool("inv_dir", config_.invert_direction);
  preferences_.putBool("inv_enc", config_.invert_encoder);
  preferences_.end();
}

bool MotionController::validateConfig(const RuntimeConfig& value) const {
  return core::isSupportedPdVoltage(value.pd_voltage_v) &&
         value.run_current_ma >= 100 && value.run_current_ma <= 2000 &&
         isSupportedMicrosteps(value.microsteps) &&
         static_cast<uint8_t>(value.standstill_mode) <= 3;
}

void MotionController::applyPdVoltage(uint8_t voltage) {
  // CH224K: CFG1 high requests 5 V. Other voltages use the CFG2/CFG3 table.
  if (voltage == 5) {
    digitalWrite(pins::kPdCfg1, HIGH);
    digitalWrite(pins::kPdCfg2, LOW);
    digitalWrite(pins::kPdCfg3, LOW);
    return;
  }
  digitalWrite(pins::kPdCfg1, LOW);
  digitalWrite(pins::kPdCfg2, LOW);
  digitalWrite(pins::kPdCfg3, voltage == 12 ? HIGH : LOW);
}

void MotionController::applyTmcConfig(const RuntimeConfig& value) {
  const float hold_multiplier = value.standstill_mode == StandstillMode::kNormal ? 0.5F : 0.0F;
  tmc_.rms_current(value.run_current_ma, hold_multiplier);
  tmc_.microsteps(value.microsteps);
  tmc_.en_spreadCycle(false);
  tmc_.pwm_autoscale(true);
  tmc_.pwm_autograd(true);
  tmc_.TCOOLTHRS(400);
  tmc_.SGTHRS(value.stallguard_threshold);
  tmc_.freewheel(static_cast<uint8_t>(value.standstill_mode));
}

void MotionController::applyHomingConfig() {
  RuntimeConfig homing = config_;
  homing.pd_voltage_v = 12;
  homing.run_current_ma = 800;
  homing.microsteps = 4;
  homing.stallguard_threshold = 20;
  applyTmcConfig(homing);
  // StealthChop AT#1 requires standstill at the run-current scale.
  tmc_.rms_current(homing.run_current_ma, 1.0F);
  active_microsteps_ = 4;
}

void MotionController::restoreHomingHoldCurrent() {
  tmc_.rms_current(
      800, config_.standstill_mode == StandstillMode::kNormal ? 0.5F : 0.0F);
}

void MotionController::setDirectionPolarity() {
  if (stepper_ != nullptr) {
    stepper_->setDirectionPin(pins::kDirection, !config_.invert_direction, 2);
  }
}

void IRAM_ATTR MotionController::diagIsr(void* argument) {
  auto* self = static_cast<MotionController*>(argument);
  if (self != nullptr && self->motion_armed_) self->emergencyIsr(self->diag_event_);
}

void IRAM_ATTR MotionController::powerGoodIsr(void* argument) {
  auto* self = static_cast<MotionController*>(argument);
  if (self != nullptr && self->driver_enabled_) self->emergencyIsr(self->power_event_);
}

void IRAM_ATTR MotionController::emergencyIsr(volatile bool& flag) {
  digitalWrite(pins::kTmcEnable, HIGH);
  if (stepper_ != nullptr) stepper_->forceStop();
  driver_enabled_ = false;
  motion_armed_ = false;
  portENTER_CRITICAL_ISR(&data_mux_);
  flag = true;
  portEXIT_CRITICAL_ISR(&data_mux_);
}

void MotionController::tick() {
  const uint32_t now = millis();
  samplePowerGood();
  if (core::powerGoodInvariantViolated(driver_enabled_, power_good_)) {
    handlePowerLoss();
    updateSnapshot();
    return;
  }

  if (now - last_encoder_sample_ms_ >= kEncoderSampleMs) {
    last_encoder_sample_ms_ = now;
    if (readEncoder()) {
      encoder_read_failures_ = 0;
    } else if (encoder_read_failures_ < UINT8_MAX) {
      ++encoder_read_failures_;
    }
  }

  updateFastTelemetry(now);
  processEmergencyEvents();
  processUrgentRequests();

  if (core::encoderLossRequiresFault(homed_, driver_enabled_, encoder_read_failures_,
                                     kEncoderFailureThreshold)) {
    enterFault(FaultCode::kEncoderFault, FaultReason::kReadFailed);
  }

  if (now - last_telemetry_sample_ms_ >= kTelemetrySampleMs) updateSlowTelemetry(now);

  processRollingHardStop(now);
  processMotion(now);
  processPendingRequest();
  updateSnapshot();
}

bool MotionController::readEncoder() {
  Wire.beginTransmission(kAs5600Address);
  Wire.write(kAs5600RawAngleRegister);
  if (Wire.endTransmission(false) != 0) {
    encoder_valid_ = false;
    return false;
  }
  if (Wire.requestFrom(kAs5600Address, static_cast<uint8_t>(2)) != 2) {
    encoder_valid_ = false;
    return false;
  }
  encoder_raw_ = static_cast<uint16_t>((Wire.read() << 8) | Wire.read()) & 0x0FFFU;
  encoder_counts_ = encoder_unwrapper_.update(encoder_raw_);
  encoder_valid_ = true;
  return true;
}

bool MotionController::captureEncoderAfterStop() {
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    if (readEncoder()) {
      encoder_read_failures_ = 0;
      return true;
    }
    if (attempt < 2) delayMicroseconds(250);
  }
  return false;
}

bool MotionController::samplePowerGood() {
  power_good_ = digitalRead(pins::kPowerGood) == LOW;
  return power_good_;
}

void MotionController::updateFastTelemetry(uint32_t now) {
  (void)now;
  diag_state_ = digitalRead(pins::kDiag) == HIGH;
  digitalWrite(pins::kLed2, diag_state_ ? HIGH : LOW);
  if (isMoving() && !motion_armed_ &&
      static_cast<int32_t>(millis() - diag_arm_after_ms_) >= 0) {
    if (diag_requires_clear_ && diag_state_) return;
    diag_requires_clear_ = false;
    motion_armed_ = true;
    if (diag_state_) {
      digitalWrite(pins::kTmcEnable, HIGH);
      stepper_->forceStop();
      motion_armed_ = false;
      diag_event_ = true;
    }
  }
}

void MotionController::updateSlowTelemetry(uint32_t now) {
  last_telemetry_sample_ms_ = now;
  uint32_t millivolts = 0;
  constexpr uint8_t kSamples = 8;
  for (uint8_t i = 0; i < kSamples; ++i) millivolts += analogReadMilliVolts(pins::kVbusAdc);
  vbus_voltage_ = (static_cast<float>(millivolts) / kSamples / 1000.0F) / kVbusDividerRatio;
}

bool MotionController::checkTmc() {
  const uint32_t status = tmc_.DRV_STATUS();
  const bool status_ok = !tmc_.CRCerror && status != 0xFFFFFFFFU;
  tmc_uart_ok_ = status_ok;
  if (!tmc_uart_ok_) {
    enterFault(FaultCode::kTmcComm, FaultReason::kReadFailed);
    return false;
  }

  tmc_status_raw_ = status;
  const uint16_t stallguard = tmc_.SG_RESULT();
  if (!tmc_.CRCerror) stallguard_result_ = stallguard;
  if ((status & kTmcSeriousFaultMask) != 0) {
    enterFault(FaultCode::kTmcDriver,
               (status & 0x02U) != 0 ? FaultReason::kOverTemperature
                                     : FaultReason::kShortCircuit);
    return false;
  }
  return true;
}

float MotionController::encoderPositionMm() const {
  if (!encoder_valid_) return commandedPositionMm();
  if (homed_ || encoder_min_counts_ != encoder_max_counts_) {
    return core::encoderCountsToMillimetres(
        (encoder_counts_ - encoder_min_counts_) * calibrated_encoder_sign_);
  }
  if (sync_anchor_valid_) return positionFromSynchronizationAnchor();
  return commandedPositionMm();
}

float MotionController::commandedPositionMm() const {
  return stepper_ == nullptr ? 0.0F
                             : core::microstepsToMillimetres(
                                   stepper_->getCurrentPosition(), active_microsteps_);
}

int8_t MotionController::currentMotionDirection() const {
  if (!isMoving()) return 0;
  const float delta = target_position_mm_ - commandedPositionMm();
  if (delta > 0.001F) return 1;
  if (delta < -0.001F) return -1;
  return 0;
}

void MotionController::setSynchronizationAnchor(float logical_position_mm) {
  if (!encoder_valid_) {
    sync_anchor_valid_ = false;
    return;
  }
  sync_anchor_.encoder_counts = encoder_counts_;
  sync_anchor_.logical_position_mm = logical_position_mm;
  sync_anchor_.encoder_sign = homed_ ? calibrated_encoder_sign_
                                    : (config_.invert_encoder ? -1 : 1);
  sync_anchor_valid_ = true;
}

float MotionController::positionFromSynchronizationAnchor() const {
  return sync_anchor_valid_ ? sync_anchor_.positionForEncoder(encoder_counts_)
                            : commandedPositionMm();
}

void MotionController::resynchronizeStepperFromEncoder(float override_position_mm,
                                                        bool use_override) {
  if (stepper_ == nullptr) return;
  float position_mm = commandedPositionMm();
  if (use_override) position_mm = override_position_mm;
  else if (sync_anchor_valid_) position_mm = positionFromSynchronizationAnchor();
  stepper_->forceStopAndNewPosition(
      core::millimetresToMicrosteps(position_mm, active_microsteps_));
  motion_armed_ = false;
  diag_requires_clear_ = false;
  requested_velocity_mm_s_ = 0.0F;
  target_position_mm_ = position_mm;
  motion_monitor_.clear();
  setSynchronizationAnchor(position_mm);
}

void MotionController::processEmergencyEvents() {
  bool power_event = false;
  bool diag_event = false;
  portENTER_CRITICAL(&data_mux_);
  power_event = power_event_;
  diag_event = diag_event_;
  power_event_ = false;
  diag_event_ = false;
  portEXIT_CRITICAL(&data_mux_);

  if (power_event) {
    handlePowerLoss();
    return;
  }

  if (diag_event) {
    // Capture the first usable shaft position after STEP generation and EN were
    // stopped by the ISR. This is the reference used to repair FAS's position.
    captureEncoderAfterStop();
    driver_enabled_ = false;
    resynchronizeStepperFromEncoder(0.0F, false);
    if (!encoder_valid_) enterFault(FaultCode::kEncoderFault, FaultReason::kReadFailed);
    else if (isHoming()) {
      if (checkTmc()) handleHomingDiag();
    } else enterFault(FaultCode::kUnexpectedStall, FaultReason::kInvalidDiag);
  }
}

void MotionController::processUrgentRequests() {
  bool stop = false;
  bool disable = false;
  portENTER_CRITICAL(&data_mux_);
  stop = urgent_stop_;
  disable = urgent_disable_;
  urgent_stop_ = false;
  urgent_disable_ = false;
  if (stop || disable) {
    pending_command_ = false;
    pending_config_ = false;
  }
  portEXIT_CRITICAL(&data_mux_);

  if (disable) {
    stopMotionAndResynchronize();
    disableDriver(true);
    mode_ = fault_ == FaultCode::kNone ? MotionMode::kDisabled : MotionMode::kFaulted;
    return;
  }
  if (!stop) return;

  const bool homing = isHoming();
  stopMotionAndResynchronize();
  if (homing) {
    disableDriver(true);
    mode_ = MotionMode::kDisabled;
  } else if (fault_ == FaultCode::kNone) {
    mode_ = driver_enabled_ ? MotionMode::kIdle : MotionMode::kDisabled;
  }
}

void MotionController::processPendingRequest() {
  Command command;
  RuntimeConfig requested;
  bool has_command = false;
  bool has_config = false;
  portENTER_CRITICAL(&data_mux_);
  if (pending_command_) {
    command = command_;
    pending_command_ = false;
    has_command = true;
  } else if (pending_config_) {
    requested = requested_config_;
    pending_config_ = false;
    has_config = true;
  }
  portEXIT_CRITICAL(&data_mux_);

  if (has_config) {
    if ((mode_ != MotionMode::kDisabled && mode_ != MotionMode::kIdle) ||
        fault_ != FaultCode::kNone) {
      return;
    }
    const bool was_enabled = driver_enabled_;
    const bool invalidates_home = requested.microsteps != config_.microsteps ||
                                  requested.invert_direction != config_.invert_direction ||
                                  requested.invert_encoder != config_.invert_encoder;
    stopMotionAndResynchronize();
    disableDriver(invalidates_home);
    config_ = requested;
    portENTER_CRITICAL(&data_mux_);
    published_config_ = config_;
    portEXIT_CRITICAL(&data_mux_);
    saveConfig();
    applyPdVoltage(config_.pd_voltage_v);
    delay(250);
    setDirectionPolarity();
    active_microsteps_ = config_.microsteps;
    applyTmcConfig(config_);
    if (!checkTmc()) return;
    samplePowerGood();
    if (was_enabled && !invalidates_home && !enableDriver()) {
      return;
    }
    mode_ = driver_enabled_ ? MotionMode::kIdle : MotionMode::kDisabled;
    return;
  }
  if (!has_command) return;

  switch (command.type) {
    case CommandType::kHome: startHome(); break;
    case CommandType::kMove: startPositionMove(command); break;
    case CommandType::kVelocity: startVelocityMove(command); break;
    case CommandType::kEnable:
      if ((mode_ == MotionMode::kDisabled || mode_ == MotionMode::kIdle) &&
          fault_ == FaultCode::kNone) {
        if (!samplePowerGood()) {
          handlePowerLoss();
          break;
        }
        if (enableDriver()) mode_ = MotionMode::kIdle;
      }
      break;
    case CommandType::kResetFault:
      if (mode_ == MotionMode::kFaulted) clearFault();
      break;
    case CommandType::kStop:
    case CommandType::kDisable:
      break;  // Handled as urgent requests in submitCommand().
  }
}

void MotionController::processMotion(uint32_t now) {
  if (!isMoving() || stepper_ == nullptr) return;

  if (isHoming()) {
    const uint32_t timeout = mode_ == MotionMode::kBackoffMin ||
                                     mode_ == MotionMode::kBackoffMax
                                 ? kBackoffTimeoutMs
                                 : kHomingTimeoutMs;
    if (now - motion_started_ms_ > timeout) {
      enterFault(FaultCode::kHomeFailed,
                 mode_ == MotionMode::kBackoffMin || mode_ == MotionMode::kBackoffMax
                     ? FaultReason::kBackoffTimeout
                     : FaultReason::kTimeout);
      return;
    }
  }

  if (mode_ == MotionMode::kHomingMin || mode_ == MotionMode::kHomingMax) {
    const int32_t travelled = std::abs(stepper_->getCurrentPosition() - motion_started_step_);
    const int32_t limit = core::millimetresToMicrosteps(kHomingDistanceLimitMm, 4);
    if (travelled >= limit || !stepper_->isRunning()) {
      enterFault(FaultCode::kHomeFailed, FaultReason::kDistanceExceeded);
    }
    return;
  }

  if (mode_ == MotionMode::kBackoffMin && !stepper_->isRunning()) {
    startHomingLeg(MotionMode::kHomingMax, 1);
    return;
  }
  if (mode_ == MotionMode::kBackoffMax && !stepper_->isRunning()) {
    finishHoming();
    return;
  }

  if (mode_ == MotionMode::kMoving) {
    if (!stepper_->isRunning()) {
      const float final_position = encoderPositionMm();
      if (core::positionOutsideSoftLimits(final_position, kSoftMarginMm,
                                          travel_mm_ - kSoftMarginMm)) {
        enterFault(FaultCode::kTravelLimit, FaultReason::kSoftLimitCrossed);
        return;
      }
      requested_velocity_mm_s_ = 0.0F;
      target_position_mm_ = final_position;
      motion_armed_ = false;
      motion_monitor_.clear();
      setSynchronizationAnchor(final_position);
      mode_ = MotionMode::kIdle;
      return;
    }
  }

  if (mode_ == MotionMode::kMoving) {
    const int8_t direction = currentMotionDirection();
    const float encoder_position = encoderPositionMm();
    const float step_position = commandedPositionMm();
    if (core::softLimitExceeded(direction, encoder_position, step_position,
                                kSoftMarginMm, travel_mm_ - kSoftMarginMm)) {
      enterFault(FaultCode::kTravelLimit, FaultReason::kSoftLimitCrossed);
    }
  }
}

void MotionController::processRollingHardStop(uint32_t now) {
  if (now - last_motion_monitor_ms_ < kEncoderSampleMs) return;
  last_motion_monitor_ms_ = now;
  if (!isMoving() || !driver_enabled_ || !encoder_valid_ || stepper_ == nullptr) {
    motion_monitor_.clear();
    return;
  }
  const int8_t direction = currentMotionDirection();
  const int8_t encoder_sign = homed_ ? calibrated_encoder_sign_
                                    : (config_.invert_encoder ? -1 : 1);
  core::MotionSample sample;
  sample.time_ms = now;
  sample.commanded_encoder_counts =
      core::microstepsToEncoderCounts(stepper_->getCurrentPosition(), active_microsteps_);
  sample.actual_encoder_counts = encoder_counts_ * encoder_sign;
  sample.direction = direction;
  if (motion_monitor_.add(sample)) {
    enterFault(FaultCode::kEncoderHardStop, FaultReason::kEncoderMismatch);
  }
}

bool MotionController::enableDriver() {
  if (stepper_ == nullptr || !checkTmc()) return false;

  // PG may change after an earlier state snapshot. Check it directly before EN,
  // immediately after EN, and once more after publishing the enabled state.
  if (!samplePowerGood()) {
    handlePowerLoss();
    return false;
  }
  stepper_->enableOutputs();
  if (!samplePowerGood()) {
    handlePowerLoss();
    return false;
  }
  driver_enabled_ = true;
  if (!samplePowerGood()) {
    handlePowerLoss();
    return false;
  }
  return driver_enabled_;
}

void MotionController::handlePowerLoss() {
  portENTER_CRITICAL(&data_mux_);
  power_event_ = false;
  portEXIT_CRITICAL(&data_mux_);

  digitalWrite(pins::kTmcEnable, HIGH);
  if (stepper_ != nullptr) stepper_->forceStop();
  driver_enabled_ = false;
  motion_armed_ = false;
  samplePowerGood();
  captureEncoderAfterStop();
  resynchronizeStepperFromEncoder(0.0F, false);
  enterFault(FaultCode::kPowerLoss, FaultReason::kPowerGoodLost);
}

void MotionController::stopMotionAndResynchronize() {
  if (stepper_ == nullptr) return;
  stepper_->forceStop();
  resynchronizeStepperFromEncoder(0.0F, false);
}

void MotionController::disableDriver(bool invalidate_home) {
  motion_armed_ = false;
  diag_requires_clear_ = false;
  digitalWrite(pins::kTmcEnable, HIGH);
  if (stepper_ != nullptr) stepper_->disableOutputs();
  driver_enabled_ = false;
  motion_monitor_.clear();
  if (invalidate_home) {
    homed_ = false;
    travel_mm_ = 0.0F;
    encoder_min_counts_ = 0;
    encoder_max_counts_ = 0;
  }
}

void MotionController::enterFault(FaultCode code, FaultReason reason) {
  digitalWrite(pins::kTmcEnable, HIGH);
  if (stepper_ != nullptr) {
    stepper_->forceStop();
    resynchronizeStepperFromEncoder(0.0F, false);
    stepper_->disableOutputs();
  }
  driver_enabled_ = false;
  homed_ = false;
  motion_armed_ = false;
  diag_requires_clear_ = false;
  motion_monitor_.clear();
  portENTER_CRITICAL(&data_mux_);
  pending_command_ = false;
  pending_config_ = false;
  portEXIT_CRITICAL(&data_mux_);
  fault_ = code;
  fault_reason_ = reason;
  mode_ = MotionMode::kFaulted;
}

void MotionController::clearFault() {
  if (stepper_ == nullptr) return;
  stopMotionAndResynchronize();
  disableDriver(true);
  if (!checkTmc()) return;
  fault_ = FaultCode::kNone;
  fault_reason_ = FaultReason::kNone;
  mode_ = MotionMode::kDisabled;
}

bool MotionController::startHome() {
  if (stepper_ == nullptr || isMoving()) return false;
  if (!canHomeFromFault(fault_)) return false;
  if (!encoder_valid_) {
    enterFault(FaultCode::kEncoderFault, FaultReason::kReadFailed);
    return false;
  }
  stopMotionAndResynchronize();
  disableDriver(true);
  fault_ = FaultCode::kNone;
  fault_reason_ = FaultReason::kNone;
  applyPdVoltage(12);
  delay(300);
  samplePowerGood();
  if (!power_good_) {
    enterFault(FaultCode::kPowerLoss, FaultReason::kPowerGoodLost);
    return false;
  }

  applyHomingConfig();
  stepper_->forceStopAndNewPosition(0);
  target_position_mm_ = 0.0F;
  setSynchronizationAnchor(0.0F);
  if (!enableDriver()) {
    restoreHomingHoldCurrent();
    return false;
  }
  // Let StealthChop complete its >130 ms standstill tuning before AT#2 motion.
  delay(kStealthChopTuningMs);
  // AT#1 is retained after returning to the configured homing hold behavior.
  restoreHomingHoldCurrent();
  if (!driver_enabled_ || !samplePowerGood()) {
    handlePowerLoss();
    return false;
  }
  startHomingLeg(MotionMode::kHomingMin, -1);
  return true;
}

void MotionController::startHomingLeg(MotionMode mode, int8_t direction) {
  setMotionProfile(kHomingSpeedMmS, kHomingAccelerationMmS2);
  mode_ = mode;
  motion_started_ms_ = millis();
  motion_started_step_ = stepper_->getCurrentPosition();
  target_position_mm_ = commandedPositionMm() + direction * kHomingDistanceLimitMm;
  requested_velocity_mm_s_ = direction * kHomingSpeedMmS;
  setSynchronizationAnchor(commandedPositionMm());
  motion_monitor_.clear();
  motion_armed_ = false;
  diag_requires_clear_ = false;
  diag_arm_after_ms_ = millis() + 25;
  stepper_->move(direction * core::millimetresToMicrosteps(kHomingDistanceLimitMm, 4));
}

void MotionController::handleHomingDiag() {
  if (!encoder_valid_) {
    enterFault(FaultCode::kEncoderFault, FaultReason::kReadFailed);
    return;
  }
  if (mode_ == MotionMode::kHomingMin) {
    encoder_min_counts_ = encoder_counts_;
    encoder_max_counts_ = encoder_min_counts_;
    calibrated_encoder_sign_ = config_.invert_encoder ? -1 : 1;
    stepper_->forceStopAndNewPosition(0);
    target_position_mm_ = 0.0F;
    setSynchronizationAnchor(0.0F);
    if (!enableDriver()) return;
    startBackoff(MotionMode::kBackoffMin, kSoftMarginMm);
    return;
  }
  if (mode_ == MotionMode::kHomingMax) {
    encoder_max_counts_ = encoder_counts_;
    const int32_t encoder_delta = encoder_max_counts_ - encoder_min_counts_;
    calibrated_encoder_sign_ = encoder_delta >= 0 ? 1 : -1;
    travel_mm_ = core::encoderCountsToMillimetres(std::abs(encoder_delta));
    if (travel_mm_ <= 2.0F * kSoftMarginMm) {
      enterFault(FaultCode::kHomeFailed, FaultReason::kTravelTooShort);
      return;
    }
    stepper_->forceStopAndNewPosition(core::millimetresToMicrosteps(travel_mm_, 4));
    target_position_mm_ = travel_mm_;
    setSynchronizationAnchor(travel_mm_);
    if (!enableDriver()) return;
    startBackoff(MotionMode::kBackoffMax, travel_mm_ - kSoftMarginMm);
    return;
  }
  enterFault(FaultCode::kHomeFailed, FaultReason::kInvalidDiag);
}

void MotionController::startBackoff(MotionMode mode, float target_mm) {
  setMotionProfile(kHomingSpeedMmS, kHomingAccelerationMmS2);
  mode_ = mode;
  motion_started_ms_ = millis();
  motion_started_step_ = stepper_->getCurrentPosition();
  target_position_mm_ = target_mm;
  requested_velocity_mm_s_ = target_mm > commandedPositionMm() ? kHomingSpeedMmS
                                                               : -kHomingSpeedMmS;
  setSynchronizationAnchor(commandedPositionMm());
  motion_monitor_.clear();
  motion_armed_ = false;
  // StallGuard DIAG commonly remains high briefly after an endpoint stall.
  // Do not arm it for the backoff until movement has made it deassert once.
  diag_requires_clear_ = true;
  diag_arm_after_ms_ = millis() + 25;
  stepper_->moveTo(core::millimetresToMicrosteps(target_mm, 4));
}

void MotionController::finishHoming() {
  requested_velocity_mm_s_ = 0.0F;
  motion_armed_ = false;
  diag_requires_clear_ = false;
  const float final_position = encoderPositionMm();
  digitalWrite(pins::kTmcEnable, HIGH);
  stepper_->disableOutputs();
  driver_enabled_ = false;

  applyPdVoltage(config_.pd_voltage_v);
  delay(250);
  active_microsteps_ = config_.microsteps;
  applyTmcConfig(config_);
  setDirectionPolarity();
  samplePowerGood();
  if (!power_good_) {
    enterFault(FaultCode::kPowerLoss, FaultReason::kPowerGoodLost);
    return;
  }
  stepper_->forceStopAndNewPosition(
      core::millimetresToMicrosteps(final_position, active_microsteps_));
  homed_ = true;
  setSynchronizationAnchor(final_position);
  if (!enableDriver()) return;
  target_position_mm_ = final_position;
  fault_ = FaultCode::kNone;
  fault_reason_ = FaultReason::kNone;
  mode_ = MotionMode::kIdle;
}

void MotionController::startPositionMove(const Command& command) {
  if (!core::closedLoopMotionReady(homed_, encoder_valid_) ||
      fault_ != FaultCode::kNone || mode_ != MotionMode::kIdle) {
    return;
  }
  setMotionProfile(command.speed_mm_s, command.acceleration_mm_s2);
  if (!enableDriver()) return;
  target_position_mm_ = command.position_mm;
  requested_velocity_mm_s_ = command.position_mm > encoderPositionMm()
                                 ? command.speed_mm_s
                                 : -command.speed_mm_s;
  setSynchronizationAnchor(encoderPositionMm());
  motion_monitor_.clear();
  mode_ = MotionMode::kMoving;
  motion_started_ms_ = millis();
  motion_started_step_ = stepper_->getCurrentPosition();
  motion_armed_ = false;
  diag_requires_clear_ = false;
  diag_arm_after_ms_ = millis() + 25;
  stepper_->moveTo(core::millimetresToMicrosteps(command.position_mm, active_microsteps_));
}

void MotionController::startVelocityMove(const Command& command) {
  Command finite_move = command;
  finite_move.type = CommandType::kMove;
  finite_move.position_mm = command.velocity_mm_s > 0
                                ? travel_mm_ - kSoftMarginMm
                                : kSoftMarginMm;
  finite_move.speed_mm_s = std::fabs(command.velocity_mm_s);
  startPositionMove(finite_move);
}

void MotionController::setMotionProfile(float speed_mm_s, float acceleration_mm_s2) {
  const uint32_t speed_steps = static_cast<uint32_t>(std::max(
      1L, static_cast<long>(std::lround(speed_mm_s * core::kFullStepsPerRevolution *
                                       active_microsteps_ / core::kMillimetresPerRevolution))));
  const int32_t acceleration_steps = static_cast<int32_t>(std::max(
      1L, static_cast<long>(std::lround(acceleration_mm_s2 * core::kFullStepsPerRevolution *
                                       active_microsteps_ / core::kMillimetresPerRevolution))));
  stepper_->setSpeedInHz(speed_steps);
  stepper_->setAcceleration(acceleration_steps);
}

bool MotionController::submitCommand(const Command& command, const char*& error_code) {
  error_code = nullptr;
  if (command.type == CommandType::kStop) {
    portENTER_CRITICAL(&data_mux_);
    urgent_stop_ = true;
    portEXIT_CRITICAL(&data_mux_);
    return true;
  }
  if (command.type == CommandType::kDisable) {
    portENTER_CRITICAL(&data_mux_);
    urgent_disable_ = true;
    portEXIT_CRITICAL(&data_mux_);
    return true;
  }

  portENTER_CRITICAL(&data_mux_);
  const StateSnapshot published = state_;
  if (pending_command_ || pending_config_) {
    portEXIT_CRITICAL(&data_mux_);
    error_code = "BUSY";
    return false;
  }

  if ((command.type == CommandType::kMove || command.type == CommandType::kVelocity) &&
      !published.encoder_valid) {
    portEXIT_CRITICAL(&data_mux_);
    error_code = "ENCODER_NOT_READY";
    return false;
  }

  if ((command.type == CommandType::kMove || command.type == CommandType::kVelocity) &&
      (!published.homed || published.mode != MotionMode::kIdle ||
       published.fault != FaultCode::kNone)) {
    portEXIT_CRITICAL(&data_mux_);
    error_code = !published.homed ? "NOT_HOMED" : "STATE_CONFLICT";
    return false;
  }
  if (command.type == CommandType::kMove &&
      (!std::isfinite(command.position_mm) ||
       command.position_mm < published.soft_min_mm ||
       command.position_mm > published.soft_max_mm)) {
    portEXIT_CRITICAL(&data_mux_);
    error_code = "OUTSIDE_SOFT_LIMITS";
    return false;
  }
  if (command.type == CommandType::kMove &&
      (!std::isfinite(command.speed_mm_s) ||
       !std::isfinite(command.acceleration_mm_s2) || command.speed_mm_s <= 0 ||
       command.speed_mm_s > kMaxNormalSpeedMmS ||
       command.acceleration_mm_s2 <= 0 ||
       command.acceleration_mm_s2 > kMaxNormalAccelerationMmS2)) {
    portEXIT_CRITICAL(&data_mux_);
    error_code = "INVALID_MOTION_PROFILE";
    return false;
  }
  if (command.type == CommandType::kVelocity &&
      (!std::isfinite(command.velocity_mm_s) ||
       !std::isfinite(command.acceleration_mm_s2) ||
       std::fabs(command.velocity_mm_s) < 0.001F ||
       std::fabs(command.velocity_mm_s) > kMaxNormalSpeedMmS ||
       command.acceleration_mm_s2 <= 0 ||
       command.acceleration_mm_s2 > kMaxNormalAccelerationMmS2)) {
    portEXIT_CRITICAL(&data_mux_);
    error_code = "INVALID_MOTION_PROFILE";
    return false;
  }
  if (command.type == CommandType::kVelocity &&
      ((command.velocity_mm_s < 0 && published.position_mm <= published.soft_min_mm) ||
       (command.velocity_mm_s > 0 && published.position_mm >= published.soft_max_mm))) {
    portEXIT_CRITICAL(&data_mux_);
    error_code = "OUTSIDE_SOFT_LIMITS";
    return false;
  }
  if (command.type == CommandType::kHome &&
      (published.mode == MotionMode::kMoving ||
       published.mode == MotionMode::kHomingMin || published.mode == MotionMode::kBackoffMin ||
       published.mode == MotionMode::kHomingMax || published.mode == MotionMode::kBackoffMax)) {
    portEXIT_CRITICAL(&data_mux_);
    error_code = "STATE_CONFLICT";
    return false;
  }
  if (command.type == CommandType::kHome && !canHomeFromFault(published.fault)) {
    portEXIT_CRITICAL(&data_mux_);
    error_code = "FAULT_RESET_REQUIRED";
    return false;
  }
  if (command.type == CommandType::kEnable && published.fault != FaultCode::kNone) {
    portEXIT_CRITICAL(&data_mux_);
    error_code = "FAULT_RESET_REQUIRED";
    return false;
  }
  if (command.type == CommandType::kEnable &&
      published.mode != MotionMode::kDisabled && published.mode != MotionMode::kIdle) {
    portEXIT_CRITICAL(&data_mux_);
    error_code = "STATE_CONFLICT";
    return false;
  }
  if (command.type == CommandType::kEnable &&
      !published.power_good) {
    portEXIT_CRITICAL(&data_mux_);
    error_code = "POWER_NOT_GOOD";
    return false;
  }
  if (command.type == CommandType::kResetFault &&
      published.mode != MotionMode::kFaulted) {
    portEXIT_CRITICAL(&data_mux_);
    error_code = "NO_ACTIVE_FAULT";
    return false;
  }
  command_ = command;
  pending_command_ = true;
  portEXIT_CRITICAL(&data_mux_);
  return true;
}

bool MotionController::submitConfig(const RuntimeConfig& value, const char*& error_code) {
  error_code = nullptr;
  if (!validateConfig(value)) {
    error_code = "INVALID_CONFIG";
    return false;
  }
  portENTER_CRITICAL(&data_mux_);
  const MotionMode published_mode = state_.mode;
  if (pending_command_ || pending_config_ ||
      (published_mode != MotionMode::kDisabled && published_mode != MotionMode::kIdle) ||
      state_.fault != FaultCode::kNone) {
    portEXIT_CRITICAL(&data_mux_);
    error_code = "STATE_CONFLICT";
    return false;
  }
  requested_config_ = value;
  pending_config_ = true;
  portEXIT_CRITICAL(&data_mux_);
  return true;
}

StateSnapshot MotionController::snapshot() const {
  portENTER_CRITICAL(&data_mux_);
  const StateSnapshot copy = state_;
  portEXIT_CRITICAL(&data_mux_);
  return copy;
}

RuntimeConfig MotionController::config() const {
  portENTER_CRITICAL(&data_mux_);
  const RuntimeConfig copy = published_config_;
  portEXIT_CRITICAL(&data_mux_);
  return copy;
}

void MotionController::updateSnapshot() {
  StateSnapshot next;
  next.mode = mode_;
  next.fault = fault_;
  next.fault_reason = fault_reason_;
  next.enabled = driver_enabled_;
  next.homed = homed_;
  next.position_mm = encoderPositionMm();
  next.commanded_position_mm = commandedPositionMm();
  next.target_position_mm = target_position_mm_;
  next.velocity_mm_s = requested_velocity_mm_s_;
  next.physical_min_mm = 0.0F;
  next.physical_max_mm = travel_mm_;
  next.soft_min_mm = homed_ ? kSoftMarginMm : 0.0F;
  next.soft_max_mm = homed_ ? travel_mm_ - kSoftMarginMm : 0.0F;
  next.encoder_valid = encoder_valid_;
  next.encoder_raw = encoder_raw_;
  next.encoder_unwrapped_counts = encoder_counts_;
  next.encoder_min_counts = encoder_min_counts_;
  next.encoder_max_counts = encoder_max_counts_;
  next.executed_microsteps = stepper_ == nullptr ? 0 : stepper_->getCurrentPosition();
  next.power_good = power_good_;
  next.vbus_voltage = vbus_voltage_;
  next.diag = diag_state_;
  next.stallguard_result = stallguard_result_;
  next.tmc_uart_ok = tmc_uart_ok_;
  next.tmc_status_raw = tmc_status_raw_;
  portENTER_CRITICAL(&data_mux_);
  state_ = next;
  portEXIT_CRITICAL(&data_mux_);
}

bool MotionController::isHoming() const {
  return mode_ == MotionMode::kHomingMin || mode_ == MotionMode::kBackoffMin ||
         mode_ == MotionMode::kHomingMax || mode_ == MotionMode::kBackoffMax;
}

bool MotionController::isNormalMotion() const {
  return mode_ == MotionMode::kMoving;
}

bool MotionController::isMoving() const {
  return isNormalMotion() || isHoming();
}

}  // namespace slider
