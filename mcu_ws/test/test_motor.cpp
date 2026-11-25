#include "mini_test.h"
#include "fake_hal.h"
#include "../bronco_mcu/motor_driver.h"

static Tuning fast() { Tuning t; t.max_accel_per_s = 1000.f; t.motor_deadband = 0.f; return t; }

TEST(zero_is_stopped_forward_is_forward_reverse_is_reverse, "MOT-1") {
  FakeHal h; Tuning t = fast(); MotorDriver m(h, MOTOR_FRONT, t);
  m.setCommand(0.f);  m.update(0.02f);
  CHECK(h.motor[MOTOR_FRONT].duty == 0.f);
  m.setCommand(0.5f); m.update(0.02f);
  CHECK(h.motor[MOTOR_FRONT].forward);
  CHECK_NEAR(h.motor[MOTOR_FRONT].duty, 0.5, 1e-6);
  m.setCommand(-0.5f); m.update(0.02f);
  CHECK(!h.motor[MOTOR_FRONT].forward);
  CHECK_NEAR(h.motor[MOTOR_FRONT].duty, 0.5, 1e-6);
}

TEST(command_is_clamped, "MOT-2") {
  FakeHal h; Tuning t = fast(); MotorDriver m(h, MOTOR_REAR, t);
  m.setCommand(7.f); m.update(0.02f);
  CHECK_NEAR(h.motor[MOTOR_REAR].duty, 1.0, 1e-6);
  CHECK_NEAR(m.applied(), 1.0, 1e-6);
  m.setCommand(-7.f); m.update(0.02f);
  CHECK_NEAR(h.motor[MOTOR_REAR].duty, 1.0, 1e-6);
  CHECK_NEAR(m.applied(), -1.0, 1e-6);
}

TEST(deadband_mapping, "MOT-3") {
  FakeHal h; Tuning t = fast(); t.motor_deadband = 0.2f; MotorDriver m(h, MOTOR_FRONT, t);
  m.setCommand(0.001f); m.update(0.02f);
  CHECK_NEAR(h.motor[MOTOR_FRONT].duty, 0.2 + 0.8 * 0.001, 1e-5);
  m.setCommand(0.5f); m.update(0.02f);
  CHECK_NEAR(h.motor[MOTOR_FRONT].duty, 0.2 + 0.8 * 0.5, 1e-5);
  m.setCommand(1.0f); m.update(0.02f);
  CHECK_NEAR(h.motor[MOTOR_FRONT].duty, 1.0, 1e-6);
  m.setCommand(0.0f); m.update(0.02f);
  CHECK(h.motor[MOTOR_FRONT].duty == 0.f);
}

TEST(slew_rate_limit_including_through_zero, "MOT-4") {
  FakeHal h; Tuning t; t.motor_deadband = 0.f; t.max_accel_per_s = 2.f;  // 0.04 per 20 ms
  MotorDriver m(h, MOTOR_FRONT, t);
  m.setCommand(1.f);
  m.update(0.02f); CHECK_NEAR(m.applied(), 0.04, 1e-6);
  m.update(0.02f); CHECK_NEAR(m.applied(), 0.08, 1e-6);
  for (int i = 0; i < 100; i++) m.update(0.02f);
  CHECK_NEAR(m.applied(), 1.0, 1e-6);
  m.setCommand(-1.f);                      // reverse: must ramp down through 0
  m.update(0.02f); CHECK_NEAR(m.applied(), 0.96, 1e-6);
  for (int i = 0; i < 24; i++) m.update(0.02f);   // 25 steps total = 1.0 -> 0.0
  CHECK_NEAR(m.applied(), 0.0, 1e-5);
  m.update(0.02f); CHECK_NEAR(m.applied(), -0.04, 1e-5);
}

TEST(inversion_swaps_output_direction_only, "MOT-5") {
  FakeHal h; Tuning t = fast(); t.front_inverted = true;
  MotorDriver f(h, MOTOR_FRONT, t), r(h, MOTOR_REAR, t);
  f.setCommand(0.5f); r.setCommand(0.5f);
  f.update(0.02f); r.update(0.02f);
  CHECK(!h.motor[MOTOR_FRONT].forward);   // inverted
  CHECK(h.motor[MOTOR_REAR].forward);     // not inverted
  CHECK_NEAR(f.applied(), 0.5, 1e-6);     // reported command is not swapped
}

TEST(stop_is_immediate_and_resets_ramp, "MOT-6") {
  FakeHal h; Tuning t; t.motor_deadband = 0.f; t.max_accel_per_s = 2.f;
  MotorDriver m(h, MOTOR_FRONT, t);
  m.setCommand(1.f);
  for (int i = 0; i < 100; i++) m.update(0.02f);
  CHECK_NEAR(m.applied(), 1.0, 1e-6);
  m.stop();
  CHECK(h.motor[MOTOR_FRONT].duty == 0.f);
  CHECK(m.applied() == 0.f);
  m.update(0.02f);                       // target was cleared too: stays stopped
  CHECK(h.motor[MOTOR_FRONT].duty == 0.f);
}
