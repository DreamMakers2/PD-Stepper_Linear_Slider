#include "slider_core.h"

#include <cmath>

namespace slider::core {

float encoderCountsToMillimetres(int32_t counts) {
  return static_cast<float>(counts) * kMillimetresPerRevolution /
         static_cast<float>(kEncoderCountsPerRevolution);
}

int32_t millimetresToEncoderCounts(float millimetres) {
  return static_cast<int32_t>(std::lround(
      millimetres * static_cast<float>(kEncoderCountsPerRevolution) /
      kMillimetresPerRevolution));
}

float microstepsToMillimetres(int32_t microsteps, uint16_t microsteps_per_step) {
  if (microsteps_per_step == 0) return 0.0F;
  return static_cast<float>(microsteps) * kMillimetresPerRevolution /
         static_cast<float>(kFullStepsPerRevolution * microsteps_per_step);
}

int32_t millimetresToMicrosteps(float millimetres, uint16_t microsteps_per_step) {
  return static_cast<int32_t>(std::lround(
      millimetres * static_cast<float>(kFullStepsPerRevolution * microsteps_per_step) /
      kMillimetresPerRevolution));
}

int32_t microstepsToEncoderCounts(int32_t microsteps, uint16_t microsteps_per_step) {
  if (microsteps_per_step == 0) return 0;
  return static_cast<int32_t>(std::lround(
      static_cast<double>(microsteps) * kEncoderCountsPerRevolution /
      static_cast<double>(kFullStepsPerRevolution * microsteps_per_step)));
}

bool softLimitExceeded(int8_t direction, float encoder_position_mm,
                       float commanded_position_mm, float soft_min_mm,
                       float soft_max_mm) {
  return positionOutsideSoftLimits(encoder_position_mm, soft_min_mm, soft_max_mm) ||
         (direction < 0 && commanded_position_mm < soft_min_mm) ||
         (direction > 0 && commanded_position_mm > soft_max_mm);
}

bool positionOutsideSoftLimits(float position_mm, float soft_min_mm,
                               float soft_max_mm) {
  return position_mm < soft_min_mm || position_mm > soft_max_mm;
}

bool powerGoodInvariantViolated(bool driver_enabled, bool power_good) {
  return driver_enabled && !power_good;
}

bool encoderLossRequiresFault(bool homed, bool driver_enabled,
                              uint8_t consecutive_failures,
                              uint8_t failure_threshold) {
  return (homed || driver_enabled) && consecutive_failures >= failure_threshold;
}

bool closedLoopMotionReady(bool homed, bool encoder_valid) {
  return homed && encoder_valid;
}

void EncoderUnwrapper::reset(uint16_t raw_count) {
  initialized_ = true;
  previous_raw_ = raw_count & 0x0FFFU;
  unwrapped_counts_ = previous_raw_;
}

int32_t EncoderUnwrapper::update(uint16_t raw_count) {
  raw_count &= 0x0FFFU;
  if (!initialized_) {
    reset(raw_count);
    return unwrapped_counts_;
  }

  int32_t delta = static_cast<int32_t>(raw_count) - previous_raw_;
  if (delta > kEncoderCountsPerRevolution / 2) {
    delta -= kEncoderCountsPerRevolution;
  } else if (delta < -kEncoderCountsPerRevolution / 2) {
    delta += kEncoderCountsPerRevolution;
  }
  unwrapped_counts_ += delta;
  previous_raw_ = raw_count;
  return unwrapped_counts_;
}

void EncoderHardStopMonitor::clear() {
  initialized_ = false;
  direction_ = 0;
  commanded_origin_ = 0;
  actual_origin_ = 0;
  tracking_error_counts_ = 0;
  violation_active_ = false;
  violation_started_ms_ = 0;
}

bool EncoderHardStopMonitor::add(const MotionSample& sample) {
  if (sample.direction == 0) {
    clear();
    return false;
  }

  if (!initialized_ || direction_ != sample.direction) {
    initialized_ = true;
    direction_ = sample.direction;
    commanded_origin_ = sample.commanded_encoder_counts;
    actual_origin_ = sample.actual_encoder_counts;
    tracking_error_counts_ = 0;
    violation_active_ = false;
    return false;
  }

  const int32_t commanded =
      (sample.commanded_encoder_counts - commanded_origin_) * direction_;
  const int32_t actual = (sample.actual_encoder_counts - actual_origin_) * direction_;
  tracking_error_counts_ = commanded - actual;

  if (tracking_error_counts_ < kHardStopErrorCounts) {
    violation_active_ = false;
    return false;
  }
  if (!violation_active_) {
    violation_active_ = true;
    violation_started_ms_ = sample.time_ms;
    return false;
  }
  return sample.time_ms - violation_started_ms_ >= kPersistenceMs;
}

float SynchronizationAnchor::positionForEncoder(int32_t current_encoder_counts) const {
  return logical_position_mm +
         encoderCountsToMillimetres((current_encoder_counts - encoder_counts) * encoder_sign);
}

}  // namespace slider::core
