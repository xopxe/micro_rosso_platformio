#include "../build_modules.h"
#ifdef BUILD_ROBOTITO_VL53RING

#include <Arduino.h>

#include "micro_rosso.h"

#include "robotito_vl53ring.h"

#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <micro_ros_utilities/string_utilities.h>

#define N_SENSORS 6
#define SENSOR_RADIUS 0.07  // distance to sensor from roboti's center
#define ADDRESS_DEFAULT 0b0101001

typedef struct {
    uint8_t xshut_pin;
    uint8_t address;
} sensor_t;

//TODO verify: "angles are measured around the positive Z axis (counterclockwise, if Z is up) 
// with zero angle being forward along the x axis"
static const sensor_t sensor_defs[N_SENSORS] = {
 {.xshut_pin=13, .address=ADDRESS_DEFAULT+1},
 {.xshut_pin=16, .address=ADDRESS_DEFAULT+2},
 {.xshut_pin=17, .address=ADDRESS_DEFAULT+3},
 {.xshut_pin=2, .address=ADDRESS_DEFAULT+4},
 {.xshut_pin=14, .address=ADDRESS_DEFAULT+5},
 {.xshut_pin=12, .address=ADDRESS_DEFAULT+6},
};
#define CONINUOUS_PERIOD_MS (uint16_t)100 // sampling period (ie timer period) / 2

static Adafruit_VL53L0X sensors[N_SENSORS];

#include <sensor_msgs/msg/laser_scan.h>
static sensor_msgs__msg__LaserScan msg_laser_scan;
static publisher_descriptor pdescriptor_laser_scan;
static float ranges[N_SENSORS];
static float intensities[N_SENSORS];


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

Vl53Ring::Vl53Ring() {
  msg_laser_scan.header.frame_id = micro_ros_string_utilities_set(
    msg_laser_scan.header.frame_id, "laser_link");  //FIXME

  msg_laser_scan.angle_min = 0.0;
  msg_laser_scan.angle_max = 2*M_PI;
  msg_laser_scan.angle_increment = 2*M_PI/N_SENSORS;
  msg_laser_scan.time_increment = 0.0; //FIXME
  msg_laser_scan.scan_time = CONINUOUS_PERIOD_MS/1000.0;
  msg_laser_scan.range_min = SENSOR_RADIUS+0.003;
  msg_laser_scan.range_max = SENSOR_RADIUS+1.2; //1.2 in default, 2 in long range
  
  msg_laser_scan.ranges.capacity = N_SENSORS;
  msg_laser_scan.ranges.data = (float*)&ranges;
  msg_laser_scan.ranges.size = N_SENSORS;
  msg_laser_scan.intensities.capacity = N_SENSORS;
  msg_laser_scan.intensities.data = (float*)&intensities;
  msg_laser_scan.intensities.size = 0;
}

static bool initialize_ring() {
  //put all sensors in reset
  for (int i=0; i<N_SENSORS; i++) {
    uint8_t pin = sensor_defs[i].xshut_pin;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  delay(10);
  
  //init sensors and renumber
  for (int i=0; i<N_SENSORS; i++) {
    uint8_t pin = sensor_defs[i].xshut_pin;
    uint8_t address = sensor_defs[i].address;
    Adafruit_VL53L0X *s = &sensors[i];
    digitalWrite(pin, HIGH);
    delay(10);
    if (!s->begin(address, false, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT)) {
      return false;
    }
  }
  
  for (int i=0; i<N_SENSORS; i++) {
    sensors[i].startRangeContinuous(CONINUOUS_PERIOD_MS);
  }
  
  return true; 
}

static void ros_state_cb(ros_states state) {
  switch (state) {
    case WAITING_AGENT:
      break;
    case AGENT_AVAILABLE:
      break;
    case AGENT_CONNECTED:
/*      for (int i=0; i<N_SENSORS; i++) {
        digitalWrite(sensors[i].xshut_pin, HIGH);
        delay(10);
        sensors[i].sensor.startRangeContinuous(CONINUOUS_PERIOD_MS);
        micro_rosso::logger.log("+");
      }*/
      //delay(10);
      break;
    case AGENT_DISCONNECTED:
      break;
    default:
      break;
  }
}

/*
static void control_cb() {
}
*/

static void report_cb(int64_t last_call_time) {
  bool new_reading = false;
  for (int i=0; i<N_SENSORS; i++) {
    Adafruit_VL53L0X *s = &sensors[i];
    if (s->isRangeComplete()) {
      new_reading = true;
      ranges[i] = SENSOR_RADIUS + s->readRange() / 1000.0; // mm -> m
      /*
      if (i==0 || i==N_SENSORS-1) {
        char cmd_str[20] {0};
        sprintf(cmd_str, "> %d %f", i, ranges[i]);
        micro_rosso::logger.log(cmd_str);
      }
      */
      
    }
  }
  if (new_reading) {
    micro_rosso::set_timestamp(msg_laser_scan.header.stamp);
    RCNOCHECK(rcl_publish(
      &pdescriptor_laser_scan.publisher,
      &msg_laser_scan,
      NULL));  
  }
}

bool Vl53Ring::setup() {
  D_println("setup: robotito_vl53ring");
  
  if (!initialize_ring()) {
    return false;
  }
  
  pdescriptor_laser_scan.qos = QOS_DEFAULT;
  pdescriptor_laser_scan.type_support = (rosidl_message_type_support_t*)
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan);
  pdescriptor_laser_scan.topic_name = VL53RING_TOPIC_LASER_SCAN;
  micro_rosso::publishers.push_back(&pdescriptor_laser_scan);
  
  micro_rosso::timer_report.callbacks.push_back(&report_cb);
  //micro_rosso::timer_control.callbacks.push_back(&control_cb);
  micro_rosso::ros_state_listeners.push_back(ros_state_cb);
  
  return true;
}

#endif // BUILD_ROBOTITO_VL53RING

