// Pure steering math (STR-1..5). No hardware access.
#pragma once
#include <math.h>
#include <stdint.h>
#include "tuning.h"

namespace steering {

// ADC counts -> angle (rad), linear, intentionally not clamped (STR-1).
inline float adcToAngle(int adc, const Tuning& t) {
  float span = float(t.adc_at_max - t.adc_at_min);
  return -t.max_steer_rad + float(adc - t.adc_at_min) * (2.f * t.max_steer_rad / span);
}

// Open wire / short detection (STR-2).
inline bool feedbackFault(int adc, const Tuning& t) {
  return adc <= t.adc_fault_low || adc >= t.adc_fault_high;
}

inline float clampAngle(float a, const Tuning& t) {  // STR-3
  return a > t.max_steer_rad ? t.max_steer_rad : (a < -t.max_steer_rad ? -t.max_steer_rad : a);
}

namespace detail {
inline float feedForwardUs(float angle, const Tuning& t) {
  float a = clampAngle(angle, t);
  float range = a >= 0.f ? float(t.servo_us_max - t.servo_us_center)
                         : float(t.servo_us_center - t.servo_us_min);
  return float(t.servo_us_center) + a / t.max_steer_rad * range;
}
inline uint16_t clampRound(float us, const Tuning& t) {
  if (us < float(t.servo_us_min)) us = float(t.servo_us_min);
  if (us > float(t.servo_us_max)) us = float(t.servo_us_max);
  return uint16_t(lroundf(us));
}
}  // namespace detail

// Feed-forward pulse for a target angle (STR-4).
inline uint16_t angleToPulseUs(float angle, const Tuning& t) {
  return detail::clampRound(detail::feedForwardUs(angle, t), t);
}

// Feed-forward + proportional feedback correction (STR-5).
inline uint16_t servoPulseUs(float target, float measured, bool fault, const Tuning& t) {
  float us = detail::feedForwardUs(target, t);
  if (!fault) us += t.steer_kp_us_per_rad * (clampAngle(target, t) - measured);
  return detail::clampRound(us, t);
}

}  // namespace steering
