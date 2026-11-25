#include <cmath>
#include <limits>
#include "mini_test.h"
#include "fake_hal.h"
#include "../bronco_mcu/vehicle_controller.h"

static Tuning tun() {
  Tuning t;
  t.wheelbase_m = 0.3f; t.max_speed_mps = 2.f; t.max_steer_rad = 0.5f;
  t.min_speed_for_steer_mps = 0.05f;
  t.motor_deadband = 0.f; t.max_accel_per_s = 1000.f;
  t.cmd_timeout_ms = 500;
  t.adc_at_min = 1000; t.adc_at_max = 3000; t.adc_fault_low = 100; t.adc_fault_high = 4000;
  t.servo_us_min = 1000; t.servo_us_center = 1500; t.servo_us_max = 2000;
  t.steer_kp_us_per_rad = 0.f;
  return t;
}
// Advance simulated time by `ms` and run one control update.
static void step(FakeHal& h, VehicleController& c, uint32_t ms = 20) { h.ms += ms; c.update(); }

TEST(speed_maps_to_both_motors, "DRV-1") {
  FakeHal h; Tuning t = tun(); VehicleController c(h, t); c.begin();
  c.onCmdVel(1.0f, 0.f); step(h, c);
  CHECK_NEAR(c.state().front_cmd, 0.5, 1e-5);
  CHECK_NEAR(c.state().rear_cmd, 0.5, 1e-5);
  CHECK_NEAR(h.motor[MOTOR_FRONT].duty, 0.5, 1e-5);
  CHECK_NEAR(h.motor[MOTOR_REAR].duty, 0.5, 1e-5);
  c.onCmdVel(-1.0f, 0.f); step(h, c);
  CHECK_NEAR(c.state().front_cmd, -0.5, 1e-5);
  CHECK(!h.motor[MOTOR_FRONT].forward && !h.motor[MOTOR_REAR].forward);
  c.onCmdVel(10.0f, 0.f); step(h, c);                  // above max speed: clamped
  CHECK_NEAR(c.state().front_cmd, 1.0, 1e-5);
}

TEST(steering_target_from_bicycle_model, "DRV-2") {
  FakeHal h; Tuning t = tun(); VehicleController c(h, t); c.begin();
  c.onCmdVel(1.0f, 0.5f); step(h, c);   // atan(0.3*0.5/1.0)
  CHECK_NEAR(c.state().steer_target, std::atan(0.15), 1e-5);
  c.onCmdVel(1.0f, -0.5f); step(h, c);
  CHECK_NEAR(c.state().steer_target, -std::atan(0.15), 1e-5);
  c.onCmdVel(-1.0f, 0.5f); step(h, c);  // reversing: sign flips
  CHECK_NEAR(c.state().steer_target, -std::atan(0.15), 1e-5);
  c.onCmdVel(1.0f, 100.f); step(h, c);  // huge turn: clamped to max steer
  CHECK_NEAR(c.state().steer_target, 0.5, 1e-6);
  // servo is actually driven toward target (STR-4 linkage)
  CHECK(h.servo_us == 2000);
}

TEST(steering_held_when_too_slow_to_steer, "DRV-3") {
  FakeHal h; Tuning t = tun(); VehicleController c(h, t); c.begin();
  c.onCmdVel(1.0f, 0.5f); step(h, c);
  float before = c.state().steer_target;
  c.onCmdVel(0.01f, 3.0f); step(h, c);   // |v| < 0.05: target unchanged
  CHECK_NEAR(c.state().steer_target, before, 1e-6);
}

TEST(zero_command_centres_and_stops, "DRV-4") {
  FakeHal h; Tuning t = tun(); VehicleController c(h, t); c.begin();
  c.onCmdVel(1.0f, 0.5f); step(h, c);
  c.onCmdVel(0.f, 0.f); step(h, c);
  CHECK(c.state().front_cmd == 0.f && c.state().rear_cmd == 0.f);
  CHECK_NEAR(c.state().steer_target, 0.0, 1e-6);
  CHECK(h.servo_us == 1500);
}

TEST(startup_state_is_safe, "SAF-1,STA-2") {
  FakeHal h; Tuning t = tun(); VehicleController c(h, t); c.begin();
  CHECK(h.motor[MOTOR_FRONT].duty == 0.f && h.motor[MOTOR_REAR].duty == 0.f);
  CHECK(h.servo_us == 1500);
  CHECK((c.state().flags & CMD_TIMEOUT) != 0);
  step(h, c);   // still no command: still stopped
  CHECK(h.motor[MOTOR_FRONT].duty == 0.f);
  CHECK((c.state().flags & CMD_TIMEOUT) != 0);
}

TEST(watchdog_stops_motors_and_holds_steering, "SAF-2") {
  FakeHal h; Tuning t = tun(); VehicleController c(h, t); c.begin();
  c.onCmdVel(1.0f, 0.5f); step(h, c);
  float steer = c.state().steer_target;
  CHECK((c.state().flags & CMD_TIMEOUT) == 0);
  step(h, c, 400);                                     // 420 ms since command
  CHECK((c.state().flags & CMD_TIMEOUT) == 0);
  CHECK(h.motor[MOTOR_FRONT].duty > 0.f);
  step(h, c, 80);                                      // exactly 500 ms
  CHECK((c.state().flags & CMD_TIMEOUT) != 0);
  CHECK(h.motor[MOTOR_FRONT].duty == 0.f && h.motor[MOTOR_REAR].duty == 0.f);
  CHECK(c.state().front_cmd == 0.f);
  CHECK_NEAR(c.state().steer_target, steer, 1e-6);     // held
}

TEST(invalid_commands_rejected_and_dont_feed_watchdog, "SAF-3,SAF-7") {
  FakeHal h; Tuning t = tun(); VehicleController c(h, t); c.begin();
  c.onCmdVel(1.0f, 0.f); step(h, c);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  c.onCmdVel(nan, 0.f); c.onCmdVel(0.f, nan); c.onCmdVel(inf, 0.f); c.onCmdVel(0.f, -inf);
  step(h, c, 100);
  CHECK_NEAR(c.state().front_cmd, 0.5, 1e-5);          // unchanged
  step(h, c, 200); c.onCmdVel(nan, nan);
  step(h, c, 200);                                      // 520 ms since last VALID cmd
  CHECK((c.state().flags & CMD_TIMEOUT) != 0);
  CHECK(h.motor[MOTOR_FRONT].duty == 0.f);
}

TEST(steering_feedback_fault_stops_motors, "SAF-4,STA-2") {
  FakeHal h; Tuning t = tun(); VehicleController c(h, t); c.begin();
  c.onCmdVel(1.0f, 0.f); step(h, c);
  CHECK(h.motor[MOTOR_FRONT].duty > 0.f);
  h.adc = 4095;                                         // wire fault
  c.onCmdVel(1.0f, 0.f); step(h, c);
  CHECK((c.state().flags & STEER_FAULT) != 0);
  CHECK(h.motor[MOTOR_FRONT].duty == 0.f && h.motor[MOTOR_REAR].duty == 0.f);
  c.onCmdVel(1.0f, 0.f); step(h, c);                    // still faulted: stays stopped
  CHECK(h.motor[MOTOR_FRONT].duty == 0.f);
  h.adc = 2000;                                         // recovers
  c.onCmdVel(1.0f, 0.f); step(h, c);
  CHECK((c.state().flags & STEER_FAULT) == 0);
  CHECK(h.motor[MOTOR_FRONT].duty > 0.f);
}

TEST(recovery_after_timeout_ramps_from_zero, "SAF-5") {
  FakeHal h; Tuning t = tun(); t.max_accel_per_s = 2.f;  // 0.04 per 20 ms
  VehicleController c(h, t); c.begin();
  c.onCmdVel(2.0f, 0.f);
  for (int i = 0; i < 100; i++) { c.onCmdVel(2.0f, 0.f); step(h, c); }   // full speed
  CHECK_NEAR(c.state().front_cmd, 1.0, 1e-5);
  step(h, c, 600);                                       // timeout
  CHECK((c.state().flags & CMD_TIMEOUT) != 0);
  CHECK(c.state().front_cmd == 0.f);
  c.onCmdVel(2.0f, 0.f); step(h, c);
  CHECK((c.state().flags & CMD_TIMEOUT) == 0);
  CHECK_NEAR(c.state().front_cmd, 0.04, 1e-5);           // ramping from 0, not 1
}

TEST(millis_rollover_is_handled, "SAF-6") {
  FakeHal h; Tuning t = tun(); h.ms = 0xFFFFFF00u;       // 256 ms before rollover
  VehicleController c(h, t); c.begin();
  c.onCmdVel(1.0f, 0.f); step(h, c, 20);
  step(h, c, 200);                                       // 220 ms: still before rollover
  CHECK((c.state().flags & CMD_TIMEOUT) == 0);
  step(h, c, 100);                                       // crosses 0: 320 ms since cmd
  CHECK(h.ms < 1000u);
  CHECK((c.state().flags & CMD_TIMEOUT) == 0);
  CHECK(h.motor[MOTOR_FRONT].duty > 0.f);
  step(h, c, 200);                                       // 520 ms since cmd
  CHECK((c.state().flags & CMD_TIMEOUT) != 0);
}

TEST(zero_dt_update_is_safe, "SAF-7") {
  FakeHal h; Tuning t = tun(); t.max_accel_per_s = 2.f; VehicleController c(h, t); c.begin();
  c.onCmdVel(1.0f, 0.f); step(h, c);
  float before = c.state().front_cmd;
  c.update();                                            // same millis -> dt == 0
  CHECK(std::isfinite(c.state().front_cmd));
  CHECK_NEAR(c.state().front_cmd, before, 1e-9);
}

TEST(state_reports_measured_steering_and_commands, "STA-1") {
  FakeHal h; Tuning t = tun(); VehicleController c(h, t); c.begin();
  h.adc = 2500;                                          // +0.25 rad
  c.onCmdVel(1.0f, 0.f); step(h, c);
  CHECK_NEAR(c.state().steer_measured, 0.25, 1e-5);
  CHECK_NEAR(c.state().steer_target, 0.0, 1e-6);
  CHECK_NEAR(c.state().front_cmd, 0.5, 1e-5);
  CHECK_NEAR(c.state().rear_cmd, 0.5, 1e-5);
  CHECK(c.state().flags == 0);
}
