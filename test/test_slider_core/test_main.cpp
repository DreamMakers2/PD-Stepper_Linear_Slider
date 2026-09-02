#include <unity.h>

#include "board_pins.h"
#include "slider_core.h"
#include "slider_types.h"

using slider::core::ButtonEdge;
using slider::core::ButtonPanel;
using slider::core::EncoderUnwrapper;
using slider::core::EncoderHardStopMonitor;
using slider::core::MotionSample;
using slider::core::SynchronizationAnchor;

void test_button_pin_mapping() {
  TEST_ASSERT_EQUAL_UINT8(35, slider::pins::kButtonLeft);
  TEST_ASSERT_EQUAL_UINT8(36, slider::pins::kButtonCenter);
  TEST_ASSERT_EQUAL_UINT8(37, slider::pins::kButtonRight);
  TEST_ASSERT_TRUE(slider::pins::kButtonsActiveLow);
}

void test_button_debounce_filters_bounce_and_reports_edges() {
  ButtonPanel buttons;
  buttons.begin(false, false, false, 0);

  TEST_ASSERT_EQUAL(static_cast<int>(ButtonEdge::kNone),
                    static_cast<int>(buttons.update(true, false, false, 5).left));
  buttons.update(false, false, false, 8);
  buttons.update(true, false, false, 10);
  TEST_ASSERT_EQUAL(static_cast<int>(ButtonEdge::kNone),
                    static_cast<int>(buttons.update(true, false, false, 19).left));
  TEST_ASSERT_EQUAL(static_cast<int>(ButtonEdge::kPressed),
                    static_cast<int>(buttons.update(true, false, false, 20).left));
  TEST_ASSERT_EQUAL_INT8(-1, buttons.jogDirection());

  buttons.update(false, false, false, 30);
  TEST_ASSERT_EQUAL(static_cast<int>(ButtonEdge::kReleased),
                    static_cast<int>(buttons.update(false, false, false, 40).left));
  TEST_ASSERT_EQUAL_INT8(0, buttons.jogDirection());
}

void test_button_held_at_boot_arms_only_after_release() {
  ButtonPanel buttons;
  buttons.begin(true, false, false, 0);
  TEST_ASSERT_FALSE(buttons.update(true, false, false, 1000).left ==
                    ButtonEdge::kPressed);

  buttons.update(false, false, false, 1010);
  TEST_ASSERT_EQUAL(static_cast<int>(ButtonEdge::kNone),
                    static_cast<int>(buttons.update(false, false, false, 1020).left));
  buttons.update(true, false, false, 1030);
  TEST_ASSERT_EQUAL(static_cast<int>(ButtonEdge::kPressed),
                    static_cast<int>(buttons.update(true, false, false, 1040).left));
}

void test_center_held_at_boot_does_not_request_home_on_release() {
  ButtonPanel buttons;
  buttons.begin(false, true, false, 0);
  TEST_ASSERT_FALSE(buttons.update(false, true, false, 2000).home_requested);
  buttons.update(false, false, false, 2010);
  const auto released = buttons.update(false, false, false, 2020);
  TEST_ASSERT_EQUAL(static_cast<int>(ButtonEdge::kNone),
                    static_cast<int>(released.center));
  TEST_ASSERT_FALSE(released.home_requested);
}

void test_center_long_press_requests_home_on_release_only() {
  ButtonPanel buttons;
  buttons.begin(false, false, false, 0);
  buttons.update(false, true, false, 100);
  TEST_ASSERT_EQUAL(static_cast<int>(ButtonEdge::kPressed),
                    static_cast<int>(buttons.update(false, true, false, 110).center));
  TEST_ASSERT_FALSE(buttons.update(false, true, false, 1099).home_requested);
  buttons.update(false, false, false, 1100);
  const auto released = buttons.update(false, false, false, 1110);
  TEST_ASSERT_EQUAL(static_cast<int>(ButtonEdge::kReleased),
                    static_cast<int>(released.center));
  TEST_ASSERT_TRUE(released.home_requested);
}

void test_center_short_press_does_not_request_home() {
  ButtonPanel buttons;
  buttons.begin(false, false, false, 0);
  buttons.update(false, true, false, 100);
  buttons.update(false, true, false, 110);
  buttons.update(false, false, false, 1000);
  TEST_ASSERT_FALSE(buttons.update(false, false, false, 1010).home_requested);
}

void test_center_long_press_timing_is_wrap_safe() {
  ButtonPanel buttons;
  constexpr uint32_t start = UINT32_MAX - 30U;
  buttons.begin(false, false, false, start);
  buttons.update(false, true, false, start + 5U);
  buttons.update(false, true, false, start + 15U);
  buttons.update(false, false, false, start + 1015U);
  TEST_ASSERT_TRUE(
      buttons.update(false, false, false, start + 1025U).home_requested);
}

void test_simultaneous_side_buttons_have_neutral_direction() {
  ButtonPanel buttons;
  buttons.begin(false, false, false, 0);
  buttons.update(true, false, true, 10);
  const auto pressed = buttons.update(true, false, true, 20);
  TEST_ASSERT_EQUAL(static_cast<int>(ButtonEdge::kPressed),
                    static_cast<int>(pressed.left));
  TEST_ASSERT_EQUAL(static_cast<int>(ButtonEdge::kPressed),
                    static_cast<int>(pressed.right));
  TEST_ASSERT_TRUE(buttons.bothSideButtonsPressed());
  TEST_ASSERT_EQUAL_INT8(0, buttons.jogDirection());
}

void test_pd_voltage_is_capped_at_12_volts() {
  TEST_ASSERT_TRUE(slider::core::isSupportedPdVoltage(5));
  TEST_ASSERT_TRUE(slider::core::isSupportedPdVoltage(9));
  TEST_ASSERT_TRUE(slider::core::isSupportedPdVoltage(12));
  TEST_ASSERT_FALSE(slider::core::isSupportedPdVoltage(15));
  TEST_ASSERT_FALSE(slider::core::isSupportedPdVoltage(20));
}

void test_physical_unit_conversions() {
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 40.0F,
                           slider::core::microstepsToMillimetres(800, 4));
  TEST_ASSERT_EQUAL_INT32(800, slider::core::millimetresToMicrosteps(40.0F, 4));
  TEST_ASSERT_EQUAL_INT32(4096, slider::core::microstepsToEncoderCounts(800, 4));
  TEST_ASSERT_EQUAL_INT32(512, slider::core::millimetresToEncoderCounts(5.0F));
}

void test_all_microstep_scales_preserve_physical_coordinates() {
  constexpr uint16_t microsteps[] = {1, 2, 4, 8, 16, 32, 64, 128, 256};
  constexpr float position_mm = 123.4F;
  for (const uint16_t scale : microsteps) {
    const int32_t steps = slider::core::millimetresToMicrosteps(position_mm, scale);
    const float recovered = slider::core::microstepsToMillimetres(steps, scale);
    const float half_step_mm = 10.0F / static_cast<float>(scale * 100);
    TEST_ASSERT_FLOAT_WITHIN(half_step_mm + 0.0001F, position_mm, recovered);
  }

  TEST_ASSERT_EQUAL_INT32(24000,
                          slider::core::millimetresToMicrosteps(150.0F, 32));
  TEST_ASSERT_EQUAL_INT32(192000,
                          slider::core::millimetresToMicrosteps(150.0F, 256));
}

void test_public_full_step_maps_to_tmcstepper_zero() {
  TEST_ASSERT_EQUAL_UINT16(0, slider::core::tmcLibraryMicrosteps(1));
  TEST_ASSERT_EQUAL_UINT16(1, slider::core::publicMicrosteps(0));
  TEST_ASSERT_EQUAL_UINT16(32, slider::core::tmcLibraryMicrosteps(32));
  TEST_ASSERT_EQUAL_UINT16(32, slider::core::publicMicrosteps(32));
}

void test_runtime_defaults_and_limits() {
  const slider::RuntimeConfig config;
  TEST_ASSERT_EQUAL_UINT16(32, config.microsteps);
  TEST_ASSERT_EQUAL(static_cast<int>(slider::StandstillMode::kFreewheeling),
                    static_cast<int>(config.standstill_mode));
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 50.0F, config.default_speed_mm_s);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 75.0F, config.default_acceleration_mm_s2);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 150.0F, slider::runtime::kMaxSpeedMmS);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 300.0F,
                           slider::runtime::kMaxAccelerationMmS2);
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

void test_hard_stop_monitor_trips_on_a_blocked_motor() {
  EncoderHardStopMonitor monitor;
  bool tripped = false;
  for (uint32_t time = 0; time <= 125; time += 5) {
    tripped = monitor.add(MotionSample{time, static_cast<int32_t>(time * 4), 0, 1});
    if (time < 125) TEST_ASSERT_FALSE(tripped);
  }
  TEST_ASSERT_TRUE(tripped);
}

void test_hard_stop_monitor_does_not_trip_when_encoder_tracks() {
  EncoderHardStopMonitor monitor;
  bool tripped = false;
  for (uint32_t time = 0; time <= 1000; time += 5) {
    const int32_t commanded = static_cast<int32_t>(time * 4);
    tripped = monitor.add(MotionSample{time, commanded, commanded - 20, 1});
    TEST_ASSERT_FALSE(tripped);
  }
}

void test_hard_stop_monitor_tolerates_irregular_sampling() {
  EncoderHardStopMonitor monitor;
  bool tripped = false;
  for (uint32_t time = 0; time <= 140; time += 20) {
    tripped = monitor.add(MotionSample{time, static_cast<int32_t>(time * 4), 0, 1});
    if (time < 140) TEST_ASSERT_FALSE(tripped);
  }
  TEST_ASSERT_TRUE(tripped);
}

void test_hard_stop_monitor_does_not_accumulate_across_windows() {
  EncoderHardStopMonitor monitor;
  for (uint32_t time = 0; time <= 1000; time += 5) {
    // No 125 ms window contains the required 164 counts of disagreement.
    const int32_t commanded = static_cast<int32_t>(time * 0.8F);
    TEST_ASSERT_FALSE(monitor.add(MotionSample{time, commanded, 0, 1}));
  }
}

void test_hard_stop_monitor_uses_only_the_latest_window() {
  EncoderHardStopMonitor monitor;
  for (uint32_t time = 0; time <= 125; time += 5) {
    const int32_t commanded = static_cast<int32_t>(time * 4);
    TEST_ASSERT_FALSE(monitor.add(MotionSample{time, commanded, commanded, 1}));
  }
  for (uint32_t time = 130; time < 255; time += 5) {
    TEST_ASSERT_FALSE(monitor.add(MotionSample{time, 500, 500, 1}));
  }
  TEST_ASSERT_FALSE(monitor.add(MotionSample{255, 600, 500, 1}));
}

void test_hard_stop_monitor_resets_on_reversal() {
  EncoderHardStopMonitor monitor;
  TEST_ASSERT_FALSE(monitor.add(MotionSample{0, 0, 0, 1}));
  TEST_ASSERT_FALSE(monitor.add(MotionSample{100, 400, 0, 1}));
  TEST_ASSERT_FALSE(monitor.add(MotionSample{105, 390, 10, -1}));
  TEST_ASSERT_EQUAL_UINT32(1, monitor.size());
}

void test_hard_stop_monitor_handles_negative_motion() {
  EncoderHardStopMonitor monitor;
  bool tripped = false;
  for (uint32_t time = 0; time <= 125; time += 5) {
    tripped = monitor.add(
        MotionSample{time, -static_cast<int32_t>(time * 4), 0, -1});
  }
  TEST_ASSERT_TRUE(tripped);
}

void test_soft_limit_tolerance_allows_endpoint_rounding_but_overrun_faults() {
  TEST_ASSERT_FALSE(slider::core::positionOutsideSoftLimits(
      5.0F, 5.0F, 465.0F));
  TEST_ASSERT_FALSE(slider::core::positionOutsideSoftLimits(
      465.19F, 5.0F, 465.0F));
  TEST_ASSERT_FALSE(slider::core::positionOutsideSoftLimits(
      4.81F, 5.0F, 465.0F));
  TEST_ASSERT_TRUE(slider::core::positionOutsideSoftLimits(
      4.79F, 5.0F, 465.0F));
  TEST_ASSERT_TRUE(slider::core::positionOutsideSoftLimits(
      465.21F, 5.0F, 465.0F));
  TEST_ASSERT_FALSE(slider::core::softLimitExceeded(
      1, 464.9F, 465.0F, 5.0F, 465.0F));
  TEST_ASSERT_FALSE(slider::core::softLimitExceeded(
      -1, 5.0F, 5.1F, 5.0F, 465.0F));
  TEST_ASSERT_FALSE(slider::core::softLimitExceeded(
      1, 465.0F, 465.19F, 5.0F, 465.0F));
  TEST_ASSERT_FALSE(slider::core::softLimitExceeded(
      -1, 5.0F, 4.81F, 5.0F, 465.0F));
  TEST_ASSERT_TRUE(slider::core::softLimitExceeded(
      1, 465.21F, 465.0F, 5.0F, 465.0F));
  TEST_ASSERT_TRUE(slider::core::softLimitExceeded(
      -1, 5.0F, 4.79F, 5.0F, 465.0F));
}

void test_bad_power_good_faults_exactly_when_driver_is_enabled() {
  TEST_ASSERT_TRUE(slider::core::powerGoodInvariantViolated(true, false));
  TEST_ASSERT_FALSE(slider::core::powerGoodInvariantViolated(false, false));
  TEST_ASSERT_FALSE(slider::core::powerGoodInvariantViolated(true, true));
}

void test_sustained_encoder_loss_faults_when_homed_or_enabled() {
  constexpr uint8_t threshold = 5;
  TEST_ASSERT_FALSE(
      slider::core::encoderLossRequiresFault(true, false, threshold - 1, threshold));
  TEST_ASSERT_TRUE(
      slider::core::encoderLossRequiresFault(true, false, threshold, threshold));
  TEST_ASSERT_TRUE(
      slider::core::encoderLossRequiresFault(false, true, threshold, threshold));
  TEST_ASSERT_FALSE(
      slider::core::encoderLossRequiresFault(false, false, threshold, threshold));
}

void test_closed_loop_motion_requires_a_currently_valid_encoder() {
  TEST_ASSERT_TRUE(slider::core::closedLoopMotionReady(true, true));
  TEST_ASSERT_FALSE(slider::core::closedLoopMotionReady(true, false));
  TEST_ASSERT_FALSE(slider::core::closedLoopMotionReady(false, true));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_button_pin_mapping);
  RUN_TEST(test_button_debounce_filters_bounce_and_reports_edges);
  RUN_TEST(test_button_held_at_boot_arms_only_after_release);
  RUN_TEST(test_center_held_at_boot_does_not_request_home_on_release);
  RUN_TEST(test_center_long_press_requests_home_on_release_only);
  RUN_TEST(test_center_short_press_does_not_request_home);
  RUN_TEST(test_center_long_press_timing_is_wrap_safe);
  RUN_TEST(test_simultaneous_side_buttons_have_neutral_direction);
  RUN_TEST(test_pd_voltage_is_capped_at_12_volts);
  RUN_TEST(test_physical_unit_conversions);
  RUN_TEST(test_all_microstep_scales_preserve_physical_coordinates);
  RUN_TEST(test_public_full_step_maps_to_tmcstepper_zero);
  RUN_TEST(test_runtime_defaults_and_limits);
  RUN_TEST(test_encoder_unwraps_in_both_directions);
  RUN_TEST(test_travel_is_derived_from_encoder_endpoint_delta);
  RUN_TEST(test_synchronization_anchor_reconstructs_physical_position);
  RUN_TEST(test_hard_stop_monitor_trips_on_a_blocked_motor);
  RUN_TEST(test_hard_stop_monitor_does_not_trip_when_encoder_tracks);
  RUN_TEST(test_hard_stop_monitor_tolerates_irregular_sampling);
  RUN_TEST(test_hard_stop_monitor_does_not_accumulate_across_windows);
  RUN_TEST(test_hard_stop_monitor_uses_only_the_latest_window);
  RUN_TEST(test_hard_stop_monitor_resets_on_reversal);
  RUN_TEST(test_hard_stop_monitor_handles_negative_motion);
  RUN_TEST(test_soft_limit_tolerance_allows_endpoint_rounding_but_overrun_faults);
  RUN_TEST(test_bad_power_good_faults_exactly_when_driver_is_enabled);
  RUN_TEST(test_sustained_encoder_loss_faults_when_homed_or_enabled);
  RUN_TEST(test_closed_loop_motion_requires_a_currently_valid_encoder);
  return UNITY_END();
}
