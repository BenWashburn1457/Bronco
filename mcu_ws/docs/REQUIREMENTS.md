# Bronco MCU firmware – Requirements

Target: ESP32, Arduino framework, micro-ROS over USB serial to the Raspberry Pi.

Every requirement has an ID. Every ID must be referenced by at least one test
(`test/`) or, where it can only be checked on the real board, by an entry in the
**Manual checklist** at the bottom. `make -C test coverage` lists any ID that
has no test.

## Assumptions (change these and tell me, and I'll update the requirements)

- A1. Two DC motors: one drives the front wheels, one drives the rear wheels.
  Each motor driver takes **one PWM pin (speed) and one DIR pin (direction)**.
- A2. Steering is a 5-wire servo: power, ground, PWM command, and a
  **position-feedback analog output** read by an ADC pin. The firmware turns the
  ADC reading into a steering angle itself (there is no helper chip).
- A3. Car is modelled as a bicycle/Ackermann vehicle with a single wheelbase.
- A4. No wheel encoders in version 1, so speed is open-loop (no odometry from
  the MCU yet).
- A5. Positive steering angle = turn left (ROS convention, CCW positive).
- A6. Real pins are unknown yet, so every pin is a named constant in one file.

## Configuration

| ID | Requirement |
|----|-------------|
| CFG-1 | Every GPIO used is defined as a named constant in `pins.h` and nowhere else. |
| CFG-2 | No two named pins in `pins.h` may share the same GPIO number. |
| CFG-3 | Every physical/tuning value (wheelbase, max speed, max steer angle, deadband, ramp rate, timeout, ADC calibration, servo pulse limits, PWM frequency/resolution, motor inversion) lives in one `Tuning` struct with defaults in `tuning.h`. |

## Motor driver (one instance per motor)

Input is a normalised command `c` in [-1, 1]. Output is `duty` in [0, 1] plus a
direction.

| ID | Requirement |
|----|-------------|
| MOT-1 | `c == 0` gives duty 0. `c > 0` gives forward, `c < 0` gives reverse. |
| MOT-2 | `c` outside [-1, 1] is clamped to [-1, 1]. |
| MOT-3 | Any non-zero command maps linearly onto duty `[deadband, 1]`: `duty = deadband + (1 - deadband) * abs(c)`, so the motor never gets a duty too small to move it. |
| MOT-4 | Output (signed) changes by at most `max_accel_per_s * dt` per update (slew limit), including through zero when reversing. |
| MOT-5 | If the motor is marked inverted in `Tuning`, forward/reverse are swapped at the output. The reported command (state) is not swapped. |
| MOT-6 | `stop()` sets duty 0 and the internal ramp state to 0 immediately, with no ramp. |

## Steering

| ID | Requirement |
|----|-------------|
| STR-1 | ADC reading is converted to angle linearly: `adc_at_min` gives `-max_steer`, `adc_at_max` gives `+max_steer`, not clamped (so out-of-range values stay visible). |
| STR-2 | Feedback fault is raised when the ADC reading is `<= adc_fault_low` or `>= adc_fault_high` (unplugged wire / short). Otherwise no fault. |
| STR-3 | A target angle is clamped to `[-max_steer, +max_steer]`. |
| STR-4 | Servo pulse (microseconds) for a target angle is linear: 0 rad gives `servo_us_center`, `+max_steer` gives `servo_us_max`, `-max_steer` gives `servo_us_min`. Always clamped to `[servo_us_min, servo_us_max]`. |
| STR-5 | Feedback correction: `pulse += steer_kp_us_per_rad * (target - measured)`, then clamped as in STR-4. Default `kp` is 0 (pure feed-forward). The correction is skipped while a feedback fault is active. |

## Vehicle controller (cmd_vel to actuators)

| ID | Requirement |
|----|-------------|
| DRV-1 | Linear velocity `v` (m/s) gives motor command `v / max_speed` clamped to [-1, 1]. Front and rear motors receive the same command. |
| DRV-2 | Steering target is `atan(wheelbase * w / v)` (bicycle model, `w` = angular velocity in rad/s), clamped per STR-3. Works for negative `v` too. |
| DRV-3 | If `abs(v) < min_speed_for_steer`, steering target is left unchanged (a car cannot rotate in place). |
| DRV-4 | `v == 0` and `w == 0` gives motors 0 and steering target 0 (centre). |

## Safety

| ID | Requirement |
|----|-------------|
| SAF-1 | At start-up (before any command): motors at 0, servo at centre, `CMD_TIMEOUT` flag set. |
| SAF-2 | If no valid command arrives for `cmd_timeout_ms`, motors are stopped immediately (MOT-6, no ramp), steering target is held, `CMD_TIMEOUT` flag set. |
| SAF-3 | A command containing NaN or infinity is rejected: it changes nothing and does **not** refresh the timeout. |
| SAF-4 | While a steering-feedback fault is active, motors are stopped immediately (MOT-6) and the `STEER_FAULT` flag is set. Motors may restart as soon as the fault clears and a valid command is present. |
| SAF-5 | A valid command after a timeout clears `CMD_TIMEOUT`; motors ramp up from 0 per MOT-4. |
| SAF-6 | Timing uses unsigned millisecond subtraction, so the 32-bit `millis()` rollover does not trigger or hide a timeout. |
| SAF-7 | Only a valid command updates `last_cmd_ms`; an update call with `dt == 0` must not divide by zero or move the ramp. |

## State report

| ID | Requirement |
|----|-------------|
| STA-1 | Controller exposes `State { steer_measured, steer_target, front_cmd, rear_cmd, flags }` where `front_cmd`/`rear_cmd` are the signed commands currently applied (after ramp, before inversion). |
| STA-2 | `flags` bit 0 = `STEER_FAULT`, bit 1 = `CMD_TIMEOUT`. |

## ROS interface (micro-ROS) – checked manually on the car

| ID | Requirement |
|----|-------------|
| COM-1 | Subscribes to `/cmd_vel` (`geometry_msgs/msg/Twist`); uses `linear.x` and `angular.z`. |
| COM-2 | Publishes `/mcu/state` (`std_msgs/msg/Float32MultiArray`) at 20 Hz with layout `[steer_measured_rad, steer_target_rad, front_cmd, rear_cmd, flags]`. |
| COM-3 | Control loop runs at 50 Hz regardless of whether the micro-ROS agent is connected. |
| COM-4 | If the agent disappears, motors stop within `cmd_timeout_ms` (SAF-2) and the firmware keeps trying to reconnect. |
| COM-5 | Hardware layer (`ArduinoHal`) maps the abstract actuators/sensors to the pins in `pins.h` and the PWM settings in `Tuning`. |

## Manual checklist (on the real car, wheels off the ground first)

- [ ] COM-1: `ros2 topic pub /cmd_vel ...` moves the motors / steering.
- [ ] COM-2: `ros2 topic hz /mcu/state` about 20 Hz.
- [ ] COM-3: with the agent not running, serial monitor shows the loop is still ticking and motors stay 0.
- [ ] COM-4: stop publishing while moving, motors stop within the timeout. Kill the agent, same result. Restart the agent, it reconnects without a reset.
- [ ] COM-5: each pin in `pins.h` drives/reads the right physical thing; fix `*_INVERTED` and ADC calibration values.
- [ ] STR-1/4: sweep steering lock to lock, check measured angle vs. real angle, and set `adc_at_min/max`, `servo_us_*`.
