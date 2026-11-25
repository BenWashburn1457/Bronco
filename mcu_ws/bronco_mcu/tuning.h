// Every physical / tuning value in one place (CFG-3). Defaults are guesses for
// a typical 1/10 scale RC car - calibrate on the real car.
#pragma once
#include <stdint.h>

struct Tuning {
  // Vehicle geometry / limits
  float wheelbase_m = 0.30f;
  float max_speed_mps = 2.0f;          // speed that maps to 100 % motor command
  float max_steer_rad = 0.45f;         // ~26 deg each side
  float min_speed_for_steer_mps = 0.05f;

  // Motors
  float motor_deadband = 0.10f;        // smallest duty that actually turns the motor
  float max_accel_per_s = 2.0f;        // command units/s (1.0 = full scale)
  bool front_inverted = false;
  bool rear_inverted = false;
  uint32_t motor_pwm_freq_hz = 20000;
  uint8_t motor_pwm_bits = 10;

  // Safety
  uint32_t cmd_timeout_ms = 500;

  // Steering feedback (ADC counts, 12-bit on ESP32). Calibrate lock-to-lock.
  int adc_at_min = 800;                // reading at full right  (-max_steer)
  int adc_at_max = 3300;               // reading at full left   (+max_steer)
  int adc_fault_low = 100;             // <= : wire open / shorted to GND
  int adc_fault_high = 4000;           // >= : shorted to 3V3

  // Steering servo command
  uint16_t servo_us_min = 1000;
  uint16_t servo_us_center = 1500;
  uint16_t servo_us_max = 2000;
  float steer_kp_us_per_rad = 0.0f;    // 0 = pure feed-forward
  uint32_t servo_pwm_freq_hz = 50;
};
