#include "build_modules.h"
#ifdef BUILD_IMU_NPU6050

#define MPU6050_RANGE_G MPU6050_RANGE_4_G        // 2, 4, 8, 16 G
#define MPU6050_RANGE_DEG MPU6050_RANGE_250_DEG  // 250, 500, 1000, 2000 deg/s
#define MPU6050_BAND MPU6050_BAND_10_HZ          // 5 10 21 44 94 184 260

#define IMU_TEMPERATURE_INTERVAL_MS 1000

#include "micro_rosso.h"

#include "imu_mpu6050.h"

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <micro_ros_utilities/string_utilities.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/temperature.h>

static sensor_msgs__msg__Imu msg_imu;
static sensor_msgs__msg__Temperature msg_temperature;

static publisher_descriptor pdescriptor_imu;
static publisher_descriptor pdescriptor_temperature;

static Adafruit_MPU6050 mpu;
static unsigned long temperature_read_time_ms;
static unsigned long temperature_topic_time_ms;

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

ImuMPU6050::ImuMPU6050() {

  msg_imu.header.frame_id = micro_ros_string_utilities_set(msg_imu.header.frame_id, "imu_link");

  msg_imu.angular_velocity_covariance[0] = msg_imu.angular_velocity_covariance[4] = msg_imu.angular_velocity_covariance[8] = 0.05;
  msg_imu.linear_acceleration_covariance[0] = msg_imu.linear_acceleration_covariance[4] = msg_imu.linear_acceleration_covariance[8] = 0.05;
  msg_imu.orientation_covariance[0] = -1;  // orientation not available from mpu6050


  msg_temperature.temperature = -273.15;
};

static bool retrieve_raw() {
  sensors_event_t a, g, temp;
  if (!mpu.getEvent(&a, &g, &temp)) {
    return false;
  }

  unsigned long now = millis();

  msg_imu.linear_acceleration.x = a.acceleration.x;
  msg_imu.linear_acceleration.y = a.acceleration.y;
  msg_imu.linear_acceleration.z = a.acceleration.z;
  msg_imu.angular_velocity.x = g.gyro.x;
  msg_imu.angular_velocity.y = g.gyro.y;
  msg_imu.angular_velocity.z = g.gyro.z;
  micro_rosso::set_timestamp(msg_imu.header.stamp);

  msg_temperature.temperature = temp.temperature;
  //set_header_timestamp(msg_temperature.header);
  temperature_read_time_ms = now;

  return true;
}

static bool retrieve_temperature() {
  unsigned long now = millis();
  if (now - temperature_topic_time_ms > IMU_TEMPERATURE_INTERVAL_MS) {
    // if last reading too old, refresh
    if (now - temperature_read_time_ms > IMU_TEMPERATURE_INTERVAL_MS) {
      sensors_event_t a, g, temp;
      if (!mpu.getEvent(&a, &g, &temp)) {
        return false;
      }
      if (msg_temperature.temperature != temp.temperature) {
        return false;
      }
      msg_temperature.temperature = temp.temperature;
      temperature_read_time_ms = now;
    }
    micro_rosso::set_timestamp(msg_temperature.header.stamp);
    temperature_topic_time_ms = now;
    return true;
  } else {
    return false;
  }
}

static void report_cb(int64_t last_call_time) {
  if (retrieve_raw()) {
    micro_rosso::set_timestamp(msg_imu.header.stamp);
    RCNOCHECK(rcl_publish(
      &pdescriptor_imu.publisher,
      &msg_imu,
      NULL));
  }
  if (retrieve_temperature()) {
    micro_rosso::set_timestamp(msg_temperature.header.stamp);
    RCNOCHECK(rcl_publish(
      &pdescriptor_temperature.publisher,
      &msg_temperature,
      NULL));
  }
}

bool ImuMPU6050::setup() {
  D_println("setup imu_mpu6050");
  if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire, 0)) {
    return false;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_G);
  mpu.setGyroRange(MPU6050_RANGE_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND);

  pdescriptor_imu.qos = QOS_DEFAULT;
  pdescriptor_imu.type_support =
    (rosidl_message_type_support_t*)ROSIDL_GET_MSG_TYPE_SUPPORT(
      sensor_msgs, msg, Imu);
  pdescriptor_imu.topic_name = MPU6050_TOPIC_IMU;
  micro_rosso::publishers.push_back(&pdescriptor_imu);

  pdescriptor_temperature.qos = QOS_DEFAULT;
  pdescriptor_temperature.type_support = (rosidl_message_type_support_t*)
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature);
  pdescriptor_temperature.topic_name = MPU6050_TOPIC_TEMPERATURE;
  micro_rosso::publishers.push_back(&pdescriptor_temperature);

  micro_rosso::timer_report.callbacks.push_back(&report_cb);

  return true;
}

#endif // BUILD_IMU_NPU6050

