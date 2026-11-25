// Bronco MCU firmware: micro-ROS over USB serial (COM-1..4).
// Board: any ESP32 with the Espressif "esp32" Arduino core 3.x
// Library: micro_ros_arduino (pick the release matching your ROS distro)
// All the control logic is in vehicle_controller.h and is unit-tested on the PC
// (see ../test). This file is only glue.
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "arduino_hal.h"
#include "vehicle_controller.h"

static const uint32_t CONTROL_PERIOD_MS = 20;  // 50 Hz  (COM-3)
static const uint32_t PUBLISH_PERIOD_MS = 50;  // 20 Hz  (COM-2)

static Tuning tuning;
static ArduinoHal hal(tuning);
static VehicleController controller(hal, tuning);

static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_subscription_t cmd_sub;
static rcl_publisher_t state_pub;
static geometry_msgs__msg__Twist cmd_msg;
static std_msgs__msg__Float32MultiArray state_msg;
static float state_data[5];

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static AgentState agent_state = WAITING_AGENT;

static void cmdVelCallback(const void* msg) {
  const geometry_msgs__msg__Twist* m = (const geometry_msgs__msg__Twist*)msg;
  controller.onCmdVel((float)m->linear.x, (float)m->angular.z);  // COM-1
}

static bool createEntities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "bronco_mcu", "", &support) != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&cmd_sub, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&state_pub, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "mcu/state") != RCL_RET_OK) return false;
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmdVelCallback, ON_NEW_DATA) != RCL_RET_OK) return false;
  return true;
}

static void destroyEntities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rcl_publisher_fini(&state_pub, &node);
  rcl_subscription_fini(&cmd_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

static void publishState() {  // COM-2: [steer_meas, steer_target, front, rear, flags]
  const State& s = controller.state();
  state_data[0] = s.steer_measured;
  state_data[1] = s.steer_target;
  state_data[2] = s.front_cmd;
  state_data[3] = s.rear_cmd;
  state_data[4] = (float)s.flags;
  (void)rcl_publish(&state_pub, &state_msg, NULL);
}

void setup() {
  hal.begin();
  controller.begin();  // motors 0, servo centred before anything else (SAF-1)

  set_microros_transports();  // USB serial

  state_msg.data.data = state_data;
  state_msg.data.size = 5;
  state_msg.data.capacity = 5;
  state_msg.layout.dim.data = NULL;
  state_msg.layout.dim.size = 0;
  state_msg.layout.dim.capacity = 0;
}

void loop() {
  static uint32_t last_control = 0, last_publish = 0, last_ping = 0;
  uint32_t now = millis();

  // Control loop never depends on the agent (COM-3, COM-4).
  if (now - last_control >= CONTROL_PERIOD_MS) {
    last_control = now;
    controller.update();
  }

  // Agent connection state machine (COM-4: auto-reconnect).
  switch (agent_state) {
    case WAITING_AGENT:
      if (now - last_ping >= 500) {
        last_ping = now;
        if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK) agent_state = AGENT_AVAILABLE;
      }
      break;
    case AGENT_AVAILABLE:
      if (createEntities()) {
        agent_state = AGENT_CONNECTED;
      } else {
        destroyEntities();
        agent_state = WAITING_AGENT;
      }
      break;
    case AGENT_CONNECTED:
      if (now - last_ping >= 500) {
        last_ping = now;
        if (rmw_uros_ping_agent(50, 3) != RMW_RET_OK) { agent_state = AGENT_DISCONNECTED; break; }
      }
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
      if (now - last_publish >= PUBLISH_PERIOD_MS) {
        last_publish = now;
        publishState();
      }
      break;
    case AGENT_DISCONNECTED:
      destroyEntities();
      agent_state = WAITING_AGENT;
      break;
  }
}
