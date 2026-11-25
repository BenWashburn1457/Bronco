// cmd_vel -> motors + steering, with watchdog and fault handling.
// Call begin() once, onCmdVel() whenever a command arrives, update() at ~50 Hz.
#pragma once
#include <math.h>
#include <stdint.h>
#include "hal.h"
#include "motor_driver.h"
#include "steering.h"
#include "tuning.h"

enum StateFlags : uint8_t { STEER_FAULT = 1, CMD_TIMEOUT = 2 };

struct State {
  float steer_measured = 0.f;  // rad
  float steer_target = 0.f;    // rad
  float front_cmd = 0.f;       // signed, applied (after ramp, before inversion)
  float rear_cmd = 0.f;
  uint8_t flags = 0;
};

class VehicleController {
 public:
  VehicleController(Hal& hal, const Tuning& t)
      : hal_(hal), t_(t), front_(hal, MOTOR_FRONT, t), rear_(hal, MOTOR_REAR, t) {}

  void begin() {  // SAF-1
    last_update_ms_ = hal_.millis();
    have_cmd_ = false;
    speed_cmd_ = 0.f;
    state_ = State();
    front_.stop();
    rear_.stop();
    state_.steer_target = 0.f;
    int adc = hal_.readSteeringAdc();
    state_.steer_measured = steering::adcToAngle(adc, t_);
    bool fault = steering::feedbackFault(adc, t_);
    hal_.writeServoUs(steering::servoPulseUs(0.f, state_.steer_measured, fault, t_));
    state_.flags = CMD_TIMEOUT | (fault ? STEER_FAULT : 0);
  }

  // v: m/s, w: rad/s (Twist linear.x / angular.z)
  void onCmdVel(float v, float w) {
    if (!isfinite(v) || !isfinite(w)) return;  // SAF-3
    last_cmd_ms_ = hal_.millis();
    have_cmd_ = true;

    float c = v / t_.max_speed_mps;  // DRV-1
    speed_cmd_ = c > 1.f ? 1.f : (c < -1.f ? -1.f : c);

    if (v == 0.f && w == 0.f) {  // DRV-4
      steer_target_ = 0.f;
    } else if (fabsf(v) >= t_.min_speed_for_steer_mps) {  // DRV-3
      steer_target_ = steering::clampAngle(atanf(t_.wheelbase_m * w / v), t_);  // DRV-2
    }
  }

  void update() {
    uint32_t now = hal_.millis();
    float dt = float(uint32_t(now - last_update_ms_)) / 1000.f;  // SAF-6 (unsigned math)
    last_update_ms_ = now;

    int adc = hal_.readSteeringAdc();
    float measured = steering::adcToAngle(adc, t_);
    bool fault = steering::feedbackFault(adc, t_);
    bool timeout = !have_cmd_ || uint32_t(now - last_cmd_ms_) >= t_.cmd_timeout_ms;

    if (fault || timeout) {  // SAF-2, SAF-4
      front_.stop();
      rear_.stop();
    } else {
      front_.setCommand(speed_cmd_);
      rear_.setCommand(speed_cmd_);
      front_.update(dt);
      rear_.update(dt);
    }

    hal_.writeServoUs(steering::servoPulseUs(steer_target_, measured, fault, t_));

    state_.steer_measured = measured;
    state_.steer_target = steer_target_;
    state_.front_cmd = front_.applied();
    state_.rear_cmd = rear_.applied();
    state_.flags = uint8_t((fault ? STEER_FAULT : 0) | (timeout ? CMD_TIMEOUT : 0));
  }

  const State& state() const { return state_; }

 private:
  Hal& hal_;
  const Tuning& t_;
  MotorDriver front_, rear_;
  State state_;
  float speed_cmd_ = 0.f;
  float steer_target_ = 0.f;
  bool have_cmd_ = false;
  uint32_t last_cmd_ms_ = 0;
  uint32_t last_update_ms_ = 0;
};
