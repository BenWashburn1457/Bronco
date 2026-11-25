#include "mini_test.h"
#include "../bronco_mcu/steering.h"

static Tuning tun() {
  Tuning t;
  t.max_steer_rad = 0.5f;
  t.adc_at_min = 1000; t.adc_at_max = 3000;
  t.adc_fault_low = 100; t.adc_fault_high = 4000;
  t.servo_us_min = 1000; t.servo_us_center = 1500; t.servo_us_max = 2000;
  t.steer_kp_us_per_rad = 0.f;
  return t;
}

TEST(adc_to_angle_linear_unclamped, "STR-1") {
  Tuning t = tun();
  CHECK_NEAR(steering::adcToAngle(1000, t), -0.5, 1e-6);
  CHECK_NEAR(steering::adcToAngle(3000, t),  0.5, 1e-6);
  CHECK_NEAR(steering::adcToAngle(2000, t),  0.0, 1e-6);
  CHECK_NEAR(steering::adcToAngle(2500, t),  0.25, 1e-6);
  CHECK_NEAR(steering::adcToAngle(4000, t),  1.0, 1e-6);   // not clamped
  CHECK_NEAR(steering::adcToAngle(0, t),    -1.0, 1e-6);
}

TEST(feedback_fault_detection, "STR-2") {
  Tuning t = tun();
  CHECK(steering::feedbackFault(100, t));
  CHECK(steering::feedbackFault(0, t));
  CHECK(steering::feedbackFault(4000, t));
  CHECK(steering::feedbackFault(4095, t));
  CHECK(!steering::feedbackFault(101, t));
  CHECK(!steering::feedbackFault(2000, t));
  CHECK(!steering::feedbackFault(3999, t));
}

TEST(target_angle_is_clamped, "STR-3") {
  Tuning t = tun();
  CHECK_NEAR(steering::clampAngle(0.3f, t), 0.3, 1e-6);
  CHECK_NEAR(steering::clampAngle(9.f, t), 0.5, 1e-6);
  CHECK_NEAR(steering::clampAngle(-9.f, t), -0.5, 1e-6);
}

TEST(angle_to_servo_pulse_linear_and_clamped, "STR-4") {
  Tuning t = tun();
  CHECK(steering::angleToPulseUs(0.f, t) == 1500);
  CHECK(steering::angleToPulseUs(0.5f, t) == 2000);
  CHECK(steering::angleToPulseUs(-0.5f, t) == 1000);
  CHECK(steering::angleToPulseUs(0.25f, t) == 1750);
  CHECK(steering::angleToPulseUs(5.f, t) == 2000);    // clamped
  CHECK(steering::angleToPulseUs(-5.f, t) == 1000);
}

TEST(feedback_correction_added_and_clamped_and_skipped_on_fault, "STR-5") {
  Tuning t = tun();
  // default kp = 0 -> identical to feed-forward
  CHECK(steering::servoPulseUs(0.25f, 0.0f, false, t) == 1750);
  t.steer_kp_us_per_rad = 200.f;
  // error = 0.25 - 0.15 = 0.10 rad -> +20 us
  CHECK(steering::servoPulseUs(0.25f, 0.15f, false, t) == 1770);
  // negative error
  CHECK(steering::servoPulseUs(0.25f, 0.35f, false, t) == 1730);
  // clamped to servo limits
  CHECK(steering::servoPulseUs(0.5f, -5.f, false, t) == 2000);
  // fault: correction skipped
  CHECK(steering::servoPulseUs(0.25f, 0.15f, true, t) == 1750);
}
