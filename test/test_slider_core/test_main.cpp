#include <unity.h>

#include "slider_core.h"

using slider::core::EncoderUnwrapper;
using slider::core::MotionSample;
using slider::core::RollingMotionMonitor;
using slider::core::SynchronizationAnchor;

void test_physical_unit_conversions() {
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 40.0F,
                           slider::core::microstepsToMillimetres(800, 4));
  TEST_ASSERT_EQUAL_INT32(800, slider::core::millimetresToMicrosteps(40.0F, 4));
  TEST_ASSERT_EQUAL_INT32(4096, slider::core::microstepsToEncoderCounts(800, 4));
  TEST_ASSERT_EQUAL_INT32(512, slider::core::millimetresToEncoderCounts(5.0F));
}

void test_encoder_unwraps_in_both_directions() {
  EncoderUnwrapper encoder;
  encoder.reset(4090);
  TEST_ASSERT_EQUAL_INT32(4101, encoder.update(5));

  encoder.reset(5);
  TEST_ASSERT_EQUAL_INT32(-6, encoder.update(4090));
}

void test_travel_is_derived_from_encoder_endpoint_delta() {
  const int32_t min_counts = -2500;
  const int32_t max_counts = 45628;
  const float travel = slider::core::encoderCountsToMillimetres(
      std::abs(max_counts - min_counts));
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 470.0F, travel);
}

void test_synchronization_anchor_reconstructs_physical_position() {
  SynchronizationAnchor anchor;
  anchor.encoder_counts = 1000;
  anchor.logical_position_mm = 100.0F;
  anchor.encoder_sign = 1;
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 105.0F, anchor.positionForEncoder(1512));

  anchor.encoder_sign = -1;
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 95.0F, anchor.positionForEncoder(1512));
}

void test_rolling_monitor_trips_on_a_blocked_motor() {
  RollingMotionMonitor monitor;
  bool tripped = false;
  for (uint32_t time = 0; time <= 125; time += 5) {
    tripped = monitor.add(MotionSample{time, static_cast<int32_t>(time * 4), 0, 1});
  }
  TEST_ASSERT_TRUE(tripped);
}

void test_rolling_monitor_does_not_trip_when_encoder_tracks() {
  RollingMotionMonitor monitor;
  bool tripped = false;
  for (uint32_t time = 0; time <= 125; time += 5) {
    const int32_t commanded = static_cast<int32_t>(time * 4);
    tripped = monitor.add(MotionSample{time, commanded, commanded - 20, 1});
  }
  TEST_ASSERT_FALSE(tripped);
}

void test_rolling_monitor_does_not_accumulate_across_windows() {
  RollingMotionMonitor monitor;
  bool tripped = false;
  for (uint32_t time = 0; time <= 1000; time += 5) {
    // Only 100 counts of disagreement can exist in any 125 ms window.
    const int32_t commanded = static_cast<int32_t>(time * 0.8F);
    tripped = monitor.add(MotionSample{time, commanded, 0, 1});
    TEST_ASSERT_FALSE(tripped);
  }
}

void test_rolling_monitor_resets_on_reversal() {
  RollingMotionMonitor monitor;
  for (uint32_t time = 0; time <= 100; time += 5) {
    monitor.add(MotionSample{time, static_cast<int32_t>(time * 4), 0, 1});
  }
  TEST_ASSERT_FALSE(monitor.add(MotionSample{105, 390, 10, -1}));
  TEST_ASSERT_EQUAL_UINT32(1, monitor.size());
}

void test_rolling_monitor_handles_negative_motion() {
  RollingMotionMonitor monitor;
  bool tripped = false;
  for (uint32_t time = 0; time <= 125; time += 5) {
    tripped = monitor.add(
        MotionSample{time, -static_cast<int32_t>(time * 4), 0, -1});
  }
  TEST_ASSERT_TRUE(tripped);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_physical_unit_conversions);
  RUN_TEST(test_encoder_unwraps_in_both_directions);
  RUN_TEST(test_travel_is_derived_from_encoder_endpoint_delta);
  RUN_TEST(test_synchronization_anchor_reconstructs_physical_position);
  RUN_TEST(test_rolling_monitor_trips_on_a_blocked_motor);
  RUN_TEST(test_rolling_monitor_does_not_trip_when_encoder_tracks);
  RUN_TEST(test_rolling_monitor_does_not_accumulate_across_windows);
  RUN_TEST(test_rolling_monitor_resets_on_reversal);
  RUN_TEST(test_rolling_monitor_handles_negative_motion);
  return UNITY_END();
}
