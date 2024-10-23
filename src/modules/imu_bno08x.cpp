#include "build_modules.h"
#ifdef BUILD_IMU_BNO08X

// For the most reliable interaction with the SHTP bus, we need
// to use hardware reset control, and to monitor the H_INT pin.
// The H_INT pin will go low when its okay to talk on the SHTP bus.
// Note, these can be other GPIO if you like.
// Define as -1 to disable these features.
//#define BNO08X_INT  A4
#define BNO08X_INT -1
//#define BNO08X_RST  A5
#define BNO08X_RST -1

#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B
//#define BNO08X_ADDR 0x4A // Alternate address if ADR jumper is closed

#include "micro_rosso.h"

#include "imu_bno08x.h"

#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <micro_ros_utilities/string_utilities.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

static sensor_msgs__msg__Imu msg_imu;
static sensor_msgs__msg__MagneticField msg_magnetic_field;

static publisher_descriptor pdescriptor_imu;
static publisher_descriptor pdescriptor_magnetic_field;


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

static BNO08x bno;

static bool updated_imu = false;
static bool updated_magnetic_field = false;

ImuBNO08x::ImuBNO08x() {

  msg_imu.header.frame_id =
    micro_ros_string_utilities_set(msg_imu.header.frame_id, "imu_link");

  msg_imu.orientation_covariance[0] = msg_imu.orientation_covariance[4] = msg_imu.orientation_covariance[8] = 0.05;
  msg_imu.angular_velocity_covariance[0] = msg_imu.angular_velocity_covariance[4] = msg_imu.angular_velocity_covariance[8] = 0.05;
  msg_imu.linear_acceleration_covariance[0] = msg_imu.linear_acceleration_covariance[4] = msg_imu.linear_acceleration_covariance[8] = 0.05;
  msg_magnetic_field.magnetic_field_covariance[0] = msg_magnetic_field.magnetic_field_covariance[4] = msg_magnetic_field.magnetic_field_covariance[8] = 0.05;
};

// Here is where you define the sensor outputs you want to receive
static bool setReports(void) {
  if (!bno.enableAccelerometer()) { return false; }
  if (!bno.enableGyro()) { return false; }
  if (!bno.enableRotationVector()) { return false; }
  if (!bno.enableMagnetometer()) { return false; }

  D_println(F("BNO Reports enabled."));
  return true;
}

static void printResetReasonName(byte resetReasonNumber) {
  if (resetReasonNumber == 1) {D_print("POR"); }
  else if (resetReasonNumber == 2) { D_print("Internal reset"); }
  else if (resetReasonNumber == 3) { D_print("Watchdog"); }
  else if (resetReasonNumber == 4) { D_print("External reset"); }
  else if (resetReasonNumber == 5) { D_print("Other"); }
}

static void control_cb(int64_t last_call_time) {
  if (bno.wasReset()) {
    D_print(F("BNO reset: "));
    printResetReasonName(bno.getResetReason());
    D_println();

    if (!setReports()) { return; }
  }

  if (!bno.getSensorEvent()) {
    return;
  }

  uint8_t reportID = bno.getSensorEventID();

  switch (reportID) {
    case SENSOR_REPORTID_ACCELEROMETER:
      updated_imu = true;
      msg_imu.linear_acceleration.x = bno.getAccelX();
      msg_imu.linear_acceleration.y = bno.getAccelY();
      msg_imu.linear_acceleration.z = bno.getAccelZ();
      break;
    case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
      updated_imu = true;
      msg_imu.angular_velocity.x = bno.getGyroX();
      msg_imu.angular_velocity.y = bno.getGyroY();
      msg_imu.angular_velocity.z = bno.getGyroZ();
      break;
    case SENSOR_REPORTID_ROTATION_VECTOR:
      updated_imu = true;
      msg_imu.orientation.x = bno.getQuatI();
      msg_imu.orientation.y = bno.getQuatJ();
      msg_imu.orientation.z = bno.getQuatK();
      msg_imu.orientation.w = bno.getQuatReal();
      //float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();
      break;
    case SENSOR_REPORTID_MAGNETIC_FIELD:
      updated_magnetic_field = true;
      msg_magnetic_field.magnetic_field.x = bno.getMagX();
      msg_magnetic_field.magnetic_field.y = bno.getMagY();
      msg_magnetic_field.magnetic_field.z = bno.getMagZ();
      //msg_magnetic_field.magnetic_field_covariance[0] = bno.getMagAccuracy();  //FIXME
      break;
    default:
      break;
  }

  return;
}

static void report_cb(int64_t last_call_time) {
  if (updated_imu) {
    updated_imu = false;
    micro_rosso::set_timestamp(msg_imu.header.stamp);
    RCNOCHECK(rcl_publish(
      &pdescriptor_imu.publisher,
      &msg_imu,
      NULL));
  }
  if (updated_magnetic_field) {
    updated_magnetic_field = false;
    micro_rosso::set_timestamp(msg_magnetic_field.header.stamp);
    RCNOCHECK(rcl_publish(
      &pdescriptor_magnetic_field.publisher,
      &msg_magnetic_field, NULL));
  }
}

bool ImuBNO08x::setup() {
  D_println("setup: imu_bno08x");
  delay(10);
  if (!bno.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST)) { return false; }
  if (!setReports()) { return false; }

  pdescriptor_imu.qos = QOS_DEFAULT;
  pdescriptor_imu.type_support =
    (rosidl_message_type_support_t*)ROSIDL_GET_MSG_TYPE_SUPPORT(
      sensor_msgs, msg, Imu);
  pdescriptor_imu.topic_name = BNO08X_TOPIC_IMU;
  micro_rosso::publishers.push_back(&pdescriptor_imu);

  pdescriptor_magnetic_field.qos = QOS_DEFAULT;
  pdescriptor_magnetic_field.type_support =
    (rosidl_message_type_support_t*)ROSIDL_GET_MSG_TYPE_SUPPORT(
      sensor_msgs, msg, MagneticField);
  pdescriptor_magnetic_field.topic_name = BNO08X_TOPIC_MAG;
  micro_rosso::publishers.push_back(&pdescriptor_magnetic_field);

  micro_rosso::timer_report.callbacks.push_back(&report_cb);
  micro_rosso::timer_control.callbacks.push_back(&control_cb);

  return true;
}

#endif // BUILD_IMU_BNO08X

