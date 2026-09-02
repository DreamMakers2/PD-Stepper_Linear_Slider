#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace slider::core {

constexpr int32_t kEncoderCountsPerRevolution = 4096;
constexpr int32_t kFullStepsPerRevolution = 200;
constexpr float kMillimetresPerRevolution = 40.0F;
constexpr int32_t kHardStopErrorCounts = 164;

constexpr bool isSupportedPdVoltage(int voltage) {
  return voltage == 5 || voltage == 9 || voltage == 12;
}

float encoderCountsToMillimetres(int32_t counts);
int32_t millimetresToEncoderCounts(float millimetres);
float microstepsToMillimetres(int32_t microsteps, uint16_t microsteps_per_step);
int32_t millimetresToMicrosteps(float millimetres, uint16_t microsteps_per_step);
int32_t microstepsToEncoderCounts(int32_t microsteps, uint16_t microsteps_per_step);
bool softLimitExceeded(int8_t direction, float encoder_position_mm,
                       float commanded_position_mm, float soft_min_mm,
                       float soft_max_mm);
bool positionOutsideSoftLimits(float position_mm, float soft_min_mm,
                               float soft_max_mm);
bool powerGoodInvariantViolated(bool driver_enabled, bool power_good);
bool encoderLossRequiresFault(bool homed, bool driver_enabled,
                              uint8_t consecutive_failures,
                              uint8_t failure_threshold);
bool closedLoopMotionReady(bool homed, bool encoder_valid);

class EncoderUnwrapper {
 public:
  void reset(uint16_t raw_count);
  int32_t update(uint16_t raw_count);
  bool initialized() const { return initialized_; }
  int32_t counts() const { return unwrapped_counts_; }

 private:
  bool initialized_ = false;
  uint16_t previous_raw_ = 0;
  int32_t unwrapped_counts_ = 0;
};

struct MotionSample {
  uint32_t time_ms = 0;
  int32_t commanded_encoder_counts = 0;
  int32_t actual_encoder_counts = 0;
  int8_t direction = 0;
};

class EncoderHardStopMonitor {
 public:
  static constexpr uint32_t kWindowMs = 125;
  static constexpr std::size_t kCapacity = 32;

  void clear();
  bool add(const MotionSample& sample);
  std::size_t size() const { return size_; }

 private:
  std::array<MotionSample, kCapacity> samples_{};
  std::size_t head_ = 0;
  std::size_t size_ = 0;
};

struct SynchronizationAnchor {
  int32_t encoder_counts = 0;
  float logical_position_mm = 0.0F;
  int8_t encoder_sign = 1;

  float positionForEncoder(int32_t current_encoder_counts) const;
};

}  // namespace slider::core
