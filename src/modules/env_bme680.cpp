#include "build_modules.h"
#ifdef BUILD_ENV_BME680

#define READS_INTERVAL_MS 2000

#include "micro_rosso.h"

#include "env_bme680.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

#include <sensor_msgs/msg/temperature.h>
#include <sensor_msgs/msg/relative_humidity.h>
#include <sensor_msgs/msg/fluid_pressure.h>
#include <std_msgs/msg/float32.h>
static sensor_msgs__msg__Temperature msg_temperature;
static sensor_msgs__msg__RelativeHumidity msg_humidity;
static sensor_msgs__msg__FluidPressure msg_pressure;
static std_msgs__msg__Float32 msg_gas_resistance;

static publisher_descriptor pdescriptor_temperature;
static publisher_descriptor pdescriptor_humidity;
static publisher_descriptor pdescriptor_pressure;
static publisher_descriptor pdescriptor_gas_resistance;

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

Adafruit_BME680 *bme;

static unsigned long last_read_ms = 0;

EnvBME680::EnvBME680() {
  msg_temperature.temperature = -273.15;  //0K
  msg_humidity.relative_humidity = 0;
  msg_pressure.fluid_pressure = 0;
  msg_gas_resistance.data = -1;
}

void report_cb(int64_t last_call_time) {
  int ramaining_millis = bme->remainingReadingMillis();
  if (ramaining_millis == bme->reading_not_started) {
    bme->beginReading();
    return;
  }
  if (ramaining_millis > bme->reading_complete) {
    return;
  }
  if (!bme->endReading()) {
    return;
  }

  unsigned long now = millis();
  if (now - last_read_ms < READS_INTERVAL_MS) {
    return;
  }
  last_read_ms = now;

  float t = bme->temperature;
  float h = bme->humidity;
  float p = bme->pressure / 100.0;
  float r = bme->gas_resistance / 1000.0;  // KOhm

  if (msg_temperature.temperature != t) {
    msg_temperature.temperature = t;
    micro_rosso::set_timestamp(msg_temperature.header.stamp);
    RCNOCHECK(rcl_publish(
      &pdescriptor_temperature.publisher,
      &msg_temperature,
      NULL));
  }
  if (msg_humidity.relative_humidity != h) {
    msg_humidity.relative_humidity = h;
    micro_rosso::set_timestamp(msg_humidity.header.stamp);
    RCNOCHECK(rcl_publish(
      &pdescriptor_humidity.publisher,
      &msg_humidity,
      NULL));
  }
  if (msg_pressure.fluid_pressure != p) {
    msg_pressure.fluid_pressure = p;
    micro_rosso::set_timestamp(msg_pressure.header.stamp);
    RCNOCHECK(rcl_publish(
      &pdescriptor_pressure.publisher,
      &msg_pressure,
      NULL));
  }
  if (msg_gas_resistance.data != r) {
    msg_gas_resistance.data = r;
    RCNOCHECK(rcl_publish(
      &pdescriptor_gas_resistance.publisher,
      &msg_gas_resistance,
      NULL));
  }
}

bool EnvBME680::setup( TwoWire &wire ) {
  D_println("setup: env_bme680");
  bme = new Adafruit_BME680(&wire);
  if (!bme->begin()) {
    return false;
  }

  // Set up oversampling and filter initialization
  bme->setTemperatureOversampling(BME680_OS_8X);
  bme->setHumidityOversampling(BME680_OS_2X);
  bme->setPressureOversampling(BME680_OS_4X);
  bme->setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme->setGasHeater(320, 150);  // 320*C for 150 ms

  pdescriptor_temperature.qos = QOS_DEFAULT;
  pdescriptor_temperature.type_support = (rosidl_message_type_support_t*)
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature);
  pdescriptor_temperature.topic_name = BME680_TOPIC_TEMPERATURE;
  micro_rosso::publishers.push_back(&pdescriptor_temperature);

  pdescriptor_humidity.qos = QOS_DEFAULT;
  pdescriptor_humidity.type_support = (rosidl_message_type_support_t*)
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, RelativeHumidity);
  pdescriptor_humidity.topic_name = BME680_TOPIC_HUMIDITY;
  micro_rosso::publishers.push_back(&pdescriptor_humidity);

  pdescriptor_pressure.qos = QOS_DEFAULT;
  pdescriptor_pressure.type_support = (rosidl_message_type_support_t*)
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, FluidPressure);
  pdescriptor_pressure.topic_name = BME680_TOPIC_PRESSURE;
  micro_rosso::publishers.push_back(&pdescriptor_pressure);

  pdescriptor_gas_resistance.qos = QOS_DEFAULT;
  pdescriptor_gas_resistance.type_support = (rosidl_message_type_support_t*)
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);
  pdescriptor_gas_resistance.topic_name = BME680_TOPIC_GAS_RESISTANCE;
  micro_rosso::publishers.push_back(&pdescriptor_gas_resistance);

  micro_rosso::timer_report.callbacks.push_back(&report_cb);

  return true;
}

#endif // BUILD_ENV_BME680

