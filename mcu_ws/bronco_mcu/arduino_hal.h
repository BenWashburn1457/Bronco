// Real ESP32 hardware layer (COM-5). Needs the ESP32 Arduino core 3.x
// (ledcAttach/ledcWrite take a pin, not a channel). Not compiled by host tests.
#pragma once
#include <Arduino.h>
#include "hal.h"
#include "pins.h"
#include "tuning.h"

class ArduinoHal : public Hal {
 public:
  explicit ArduinoHal(const Tuning& t) : t_(t) {}

  void begin() {
    pinMode(pins::MOTOR_FRONT_DIR, OUTPUT);
    pinMode(pins::MOTOR_REAR_DIR, OUTPUT);
    ledcAttach(pins::MOTOR_FRONT_PWM, t_.motor_pwm_freq_hz, t_.motor_pwm_bits);
    ledcAttach(pins::MOTOR_REAR_PWM, t_.motor_pwm_freq_hz, t_.motor_pwm_bits);
    ledcAttach(pins::STEER_SERVO_PWM, t_.servo_pwm_freq_hz, SERVO_BITS);
    analogReadResolution(12);
    pinMode(pins::STEER_FEEDBACK_ADC, INPUT);
  }

  void writeMotor(MotorId id, float duty, bool forward) override {
    int pwm = (id == MOTOR_FRONT) ? pins::MOTOR_FRONT_PWM : pins::MOTOR_REAR_PWM;
    int dir = (id == MOTOR_FRONT) ? pins::MOTOR_FRONT_DIR : pins::MOTOR_REAR_DIR;
    uint32_t maxDuty = (1u << t_.motor_pwm_bits) - 1;
    digitalWrite(dir, forward ? HIGH : LOW);
    ledcWrite(pwm, (uint32_t)(duty * maxDuty + 0.5f));
  }

  void writeServoUs(uint16_t us) override {
    uint32_t periodUs = 1000000UL / t_.servo_pwm_freq_hz;
    ledcWrite(pins::STEER_SERVO_PWM, (uint32_t)us * (1u << SERVO_BITS) / periodUs);
  }

  int readSteeringAdc() override { return analogRead(pins::STEER_FEEDBACK_ADC); }
  uint32_t millis() override { return ::millis(); }

 private:
  static constexpr uint8_t SERVO_BITS = 14;
  const Tuning& t_;
};
