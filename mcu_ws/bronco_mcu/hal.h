// Hardware abstraction: the control logic only talks to this interface, so it
// runs unchanged on the ESP32 (arduino_hal.h) and in host unit tests (FakeHal).
#pragma once
#include <stdint.h>

enum MotorId { MOTOR_FRONT = 0, MOTOR_REAR = 1 };

struct Hal {
  virtual ~Hal() {}
  virtual void writeMotor(MotorId id, float duty /*0..1*/, bool forward) = 0;
  virtual void writeServoUs(uint16_t pulse_us) = 0;
  virtual int readSteeringAdc() = 0;
  virtual uint32_t millis() = 0;
};
