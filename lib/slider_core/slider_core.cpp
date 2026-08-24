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

void RollingMotionMonitor::clear() {
  head_ = 0;
  size_ = 0;
}

bool RollingMotionMonitor::add(const MotionSample& sample) {
  if (sample.direction == 0) {
    clear();
    return false;
  }

  if (size_ > 0) {
    const std::size_t newest = (head_ + size_ - 1) % kCapacity;
    if (samples_[newest].direction != sample.direction) clear();
  }

  if (size_ == kCapacity) {
    head_ = (head_ + 1) % kCapacity;
    --size_;
  }
  samples_[(head_ + size_) % kCapacity] = sample;
  ++size_;

  while (size_ > 1) {
    const MotionSample& oldest = samples_[head_];
    if (sample.time_ms - oldest.time_ms <= kWindowMs) break;
    const std::size_t next = (head_ + 1) % kCapacity;
    if (sample.time_ms - samples_[next].time_ms < kWindowMs) break;
    head_ = next;
    --size_;
  }

  if (size_ < 2) return false;
  const MotionSample& oldest = samples_[head_];
  const uint32_t elapsed = sample.time_ms - oldest.time_ms;
  if (elapsed < kWindowMs - 5 || elapsed > kWindowMs + 10) return false;

  const int32_t direction = sample.direction;
  const int32_t commanded =
      (sample.commanded_encoder_counts - oldest.commanded_encoder_counts) * direction;
  const int32_t actual =
      (sample.actual_encoder_counts - oldest.actual_encoder_counts) * direction;
  if (commanded < kHardStopErrorCounts) return false;

  return commanded - actual >= kHardStopErrorCounts;
}

float SynchronizationAnchor::positionForEncoder(int32_t current_encoder_counts) const {
  return logical_position_mm +
         encoderCountsToMillimetres((current_encoder_counts - encoder_counts) * encoder_sign);
}

}  // namespace slider::core
