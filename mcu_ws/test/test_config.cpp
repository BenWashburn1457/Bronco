#include <algorithm>
#include <cstdio>
#include <vector>
#include "mini_test.h"
#include "../bronco_mcu/pins.h"
#include "../bronco_mcu/tuning.h"

TEST(pins_are_named_constants_and_listed, "CFG-1") {
  // Every pin must be in ALL_PINS so the uniqueness check below covers it.
  CHECK(pins::ALL_PINS_COUNT == 6);
  CHECK(pins::MOTOR_FRONT_PWM >= 0);
  CHECK(pins::MOTOR_FRONT_DIR >= 0);
  CHECK(pins::MOTOR_REAR_PWM >= 0);
  CHECK(pins::MOTOR_REAR_DIR >= 0);
  CHECK(pins::STEER_SERVO_PWM >= 0);
  CHECK(pins::STEER_FEEDBACK_ADC >= 0);
}

TEST(no_duplicate_pins, "CFG-2") {
  std::vector<int> v(pins::ALL_PINS, pins::ALL_PINS + pins::ALL_PINS_COUNT);
  std::sort(v.begin(), v.end());
  CHECK(std::adjacent_find(v.begin(), v.end()) == v.end());
}

TEST(tuning_has_defaults_for_everything, "CFG-3") {
  Tuning t;  // default-constructible, all values sane and self-consistent
  CHECK(t.wheelbase_m > 0);
  CHECK(t.max_speed_mps > 0);
  CHECK(t.max_steer_rad > 0);
  CHECK(t.motor_deadband >= 0 && t.motor_deadband < 1);
  CHECK(t.max_accel_per_s > 0);
  CHECK(t.cmd_timeout_ms > 0);
  CHECK(t.adc_at_min < t.adc_at_max);
  CHECK(t.adc_fault_low < t.adc_at_min);
  CHECK(t.adc_fault_high > t.adc_at_max);
  CHECK(t.servo_us_min < t.servo_us_center && t.servo_us_center < t.servo_us_max);
  CHECK(t.motor_pwm_freq_hz > 0 && t.motor_pwm_bits > 0);
  CHECK(t.servo_pwm_freq_hz == 50);
  CHECK(t.front_inverted == false && t.rear_inverted == false);
}
