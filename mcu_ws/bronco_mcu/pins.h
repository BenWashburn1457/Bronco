// ALL GPIO assignments live here (CFG-1). Placeholder values until the real
// schematic exists - change only this file. Keep numbers unique (CFG-2) and
// add every new pin to ALL_PINS so the unit test can check for duplicates.
//
// ESP32 notes: use an ADC1 pin (GPIO32-39) for the steering feedback; GPIO34-39
// are input-only. Avoid strapping pins (0, 2, 5, 12, 15) for outputs if you can.
#pragma once

namespace pins {

// Front motor driver
constexpr int MOTOR_FRONT_PWM = 25;
constexpr int MOTOR_FRONT_DIR = 26;
// Rear motor driver
constexpr int MOTOR_REAR_PWM = 27;
constexpr int MOTOR_REAR_DIR = 14;
// Steering servo (5-wire): PWM command out, analog position feedback in
constexpr int STEER_SERVO_PWM = 13;
constexpr int STEER_FEEDBACK_ADC = 34;

constexpr int ALL_PINS[] = {
    MOTOR_FRONT_PWM, MOTOR_FRONT_DIR, MOTOR_REAR_PWM,
    MOTOR_REAR_DIR,  STEER_SERVO_PWM, STEER_FEEDBACK_ADC,
};
constexpr int ALL_PINS_COUNT = sizeof(ALL_PINS) / sizeof(ALL_PINS[0]);

}  // namespace pins
