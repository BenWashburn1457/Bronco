#pragma once
#include <cstdint>
#include "../bronco_mcu/hal.h"

// Records what the firmware does to the hardware; lets tests set sensors/time.
struct FakeHal : Hal {
  struct Out { float duty = -1; bool forward = true; int writes = 0; };
  Out motor[2];
  uint16_t servo_us = 0;
  int adc = 2000;
  uint32_t ms = 0;

  void writeMotor(MotorId id, float duty, bool forward) override {
    motor[id].duty = duty; motor[id].forward = forward; motor[id].writes++;
  }
  void writeServoUs(uint16_t us) override { servo_us = us; }
  int readSteeringAdc() override { return adc; }
  uint32_t millis() override { return ms; }
};
