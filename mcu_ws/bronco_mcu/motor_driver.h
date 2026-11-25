// One motor channel: clamp, slew-limit, deadband-map, invert (MOT-1..6).
#pragma once
#include <math.h>
#include "hal.h"
#include "tuning.h"

class MotorDriver {
 public:
  MotorDriver(Hal& hal, MotorId id, const Tuning& t) : hal_(hal), id_(id), t_(t) {}

  void setCommand(float c) {  // normalised [-1, 1]
    target_ = c > 1.f ? 1.f : (c < -1.f ? -1.f : c);
  }

  // Call periodically; dt in seconds. dt <= 0 does nothing (SAF-7).
  void update(float dt) {
    if (!(dt > 0.f)) return;
    float step = t_.max_accel_per_s * dt;
    float diff = target_ - applied_;
    if (diff > step) diff = step;
    if (diff < -step) diff = -step;
    applied_ += diff;
    write();
  }

  // Immediate stop, no ramp (MOT-6).
  void stop() {
    target_ = 0.f;
    applied_ = 0.f;
    write();
  }

  float applied() const { return applied_; }  // signed, before inversion

 private:
  void write() {
    float mag = fabsf(applied_);
    float duty = mag > 0.f ? t_.motor_deadband + (1.f - t_.motor_deadband) * mag : 0.f;
    if (duty > 1.f) duty = 1.f;
    bool inverted = (id_ == MOTOR_FRONT) ? t_.front_inverted : t_.rear_inverted;
    hal_.writeMotor(id_, duty, (applied_ >= 0.f) != inverted);
  }

  Hal& hal_;
  MotorId id_;
  const Tuning& t_;
  float target_ = 0.f;
  float applied_ = 0.f;
};
