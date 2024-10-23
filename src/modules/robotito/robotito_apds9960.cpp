#include "../build_modules.h"
#ifdef BUILD_ROBOTITO_APDS9960

#include <Arduino.h>

#define COLOR_RAW_ENABLE true

static const uint8_t D1LED_PIN = 32;
static const int threshold = 10;


#include "micro_rosso.h"

#include "robotito_apds9960.h"

#include <Wire.h>
#include <Arduino_APDS9960.h>

#include <std_msgs/msg/bool.h>
static std_msgs__msg__Bool msg_proximity;
static publisher_descriptor pdescriptor_proximity;

#include <geometry_msgs/msg/point32.h>
static geometry_msgs__msg__Point32 msg_color_raw;
static publisher_descriptor pdescriptor_color_raw;

static bool color_raw_enabled = false;

static int last_close = -1;

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { return false; } \
  }
#define RCNOCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    (void)temp_rc; \
  }

Apds9960::Apds9960() {
}

static void ros_state_cb(ros_states state) {
  switch (state) {
    case WAITING_AGENT:
      break;
    case AGENT_AVAILABLE:
      break;
    case AGENT_CONNECTED:
      last_close = -1;
      break;
    case AGENT_DISCONNECTED:
      break;
    default:
      break;
  }
}

static void control_cb(int64_t last_call_time) {
}

static void check_proximity() {
  int proximity = APDS.readProximity();
  
  /*
  char cmd_str[10] {0};
  sprintf(cmd_str, "> %d %d", last_proximity, proximity);
  micro_rosso::logger.log(cmd_str);
  */
  
  if ( last_close!=0 && proximity>threshold ) {
    digitalWrite(D1LED_PIN, LOW);
    color_raw_enabled = false;
    msg_proximity.data = false;
    last_close = 0;
    RCNOCHECK(rcl_publish(
      &pdescriptor_proximity.publisher,
      &msg_proximity,
      NULL));
  } else if ( last_close!=1 && proximity<threshold ) {
    digitalWrite(D1LED_PIN, HIGH);
    color_raw_enabled = COLOR_RAW_ENABLE;
    msg_proximity.data = true;
    last_close = 1;
    RCNOCHECK(rcl_publish(
      &pdescriptor_proximity.publisher,
      &msg_proximity,
      NULL));
  }
}

static void check_color_raw() {
  int r = 0, g = 0, b = 0;
  APDS.readColor(r, g, b);
    
  msg_color_raw.x = r;
  msg_color_raw.y = g;
  msg_color_raw.z = b;
  RCNOCHECK(rcl_publish(
    &pdescriptor_color_raw.publisher,
    &msg_color_raw,
    NULL));
}

static void report_cb(int64_t last_call_time) {
  if (APDS.proximityAvailable()) {
    check_proximity();
  }
  if (color_raw_enabled && APDS.colorAvailable()) {
    check_color_raw();
  }
}

bool Apds9960::setup() {
  D_println("setup: robotito_apds");
  
  if (!APDS.begin()) {
    return false;
  }
  
  pinMode(D1LED_PIN, OUTPUT);
  digitalWrite(D1LED_PIN, LOW);

  pdescriptor_proximity.qos = QOS_DEFAULT;
  pdescriptor_proximity.type_support = (rosidl_message_type_support_t*)
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);
  pdescriptor_proximity.topic_name = APDS9960_TOPIC_PROXIMITY;
  micro_rosso::publishers.push_back(&pdescriptor_proximity);

  pdescriptor_color_raw.qos = QOS_BEST_EFFORT;
  pdescriptor_color_raw.type_support = (rosidl_message_type_support_t*)
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32);
  pdescriptor_color_raw.topic_name = APDS9960_TOPIC_COLOR_RAW;
  micro_rosso::publishers.push_back(&pdescriptor_color_raw);

  micro_rosso::timer_report.callbacks.push_back(&report_cb);
  micro_rosso::timer_control.callbacks.push_back(&control_cb);
  micro_rosso::ros_state_listeners.push_back(ros_state_cb);

  return true;
}

#endif // BUILD_ROBOTITO_APDS9960

