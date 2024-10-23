#include "build_modules.h"
#ifdef BUILD_ENV_DHT22

#define NOINTERRUPTS
#define COOLDOWN_TIME_MS 2000
#define DHT_SENSOR_PIN 25  // support pull-up: 2,4,5,12,13,14,15,25,26,27

#include "micro_rosso.h"

#include "env_dht22.h"

#include <sensor_msgs/msg/temperature.h>
#include <sensor_msgs/msg/relative_humidity.h>

static sensor_msgs__msg__Temperature msg_temperature;
static sensor_msgs__msg__RelativeHumidity msg_humidity;

static publisher_descriptor pdescriptor_temperature;
static publisher_descriptor pdescriptor_humidity;

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

static unsigned long lastreadtime;
static uint32_t maxCyclesShort;
static uint32_t maxCyclesLong;

class DisableInterruptsGuard {
public:
  DisableInterruptsGuard() {
    noInterrupts();
  }

  ~DisableInterruptsGuard() {
    interrupts();
  }
};

EnvDHT22::EnvDHT22() {
  //lastreadtime = millis();  // to start immediately, substract COOLDOWN_TIME_MS

  //maxCycles = microsecondsToClockCycles(1000);
  maxCyclesShort = microsecondsToClockCycles(50 + 50);
  maxCyclesLong = microsecondsToClockCycles(100 + 50);
  pinMode(DHT_SENSOR_PIN, INPUT);
  digitalWrite(DHT_SENSOR_PIN, HIGH);

  msg_temperature.temperature = -273.15;  //0K
  msg_humidity.relative_humidity = 0;
}


float getTemperature(byte data[5]) {
  float f = ((uint16_t)(data[2] & 0x7F)) << 8 | data[3];
  f *= 0.1;
  if (data[2] & 0x80) {
    f *= -1;
  }
  return f;
}

float getHumidity(byte data[5]) {
  float f = ((uint16_t)data[0]) << 8 | data[1];
  f *= 0.1;
  return f;
}

uint32_t expectPulse(bool level, uint32_t cycles) {
  uint32_t count = 0;
  while (digitalRead(DHT_SENSOR_PIN) == level) {
    if (count++ >= cycles) {
      return 0;  // Exceeded timeout, fail.
    }
  }
  return count;
}

bool readData(uint8_t data[5]) {
  unsigned long currenttime = millis();
  if ((currenttime - lastreadtime) < COOLDOWN_TIME_MS) {
    return false;  // cooldown
  }
  lastreadtime = currenttime;

  // Send start signal.
  pinMode(DHT_SENSOR_PIN, OUTPUT);
  digitalWrite(DHT_SENSOR_PIN, LOW);
  delayMicroseconds(1100);  // "at least 1ms"

  uint32_t cycles[80];

  {
    // End the start signal by setting data line high for 40 microseconds.
    digitalWrite(DHT_SENSOR_PIN, HIGH);
    delayMicroseconds(30);  // "20-40us"

    // Now start reading the data line to get the value from the DHT sensor.
    pinMode(DHT_SENSOR_PIN, INPUT);
    // Delay a bit to let sensor pull data line low.
    delayMicroseconds(10);

#ifdef NOINTERRUPTS
    volatile DisableInterruptsGuard interrupt;
#endif

    // First expect a low signal for ~80 microseconds followed by a high signal
    // for ~80 microseconds again.
    if (expectPulse(LOW, maxCyclesLong) == 0) {  // 80us
      //Serial2.print("!1");
      return false;
    }
    if (expectPulse(HIGH, maxCyclesLong) == 0) {  // 80us
      //Serial2.print("!2");
      return false;
    }

    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
    // microsecond low pulse followed by a variable length high pulse.  If the
    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
    // and use that to compare to the cycle count of the high pulse to determine
    // if the bit is a 0 (high state cycle count < low state cycle count), or a
    // 1 (high state cycle count > low state cycle count). Note that for speed all
    // the pulses are read into a array and then examined in a later step.
    for (int i = 0; i < 80; i += 2) {
      uint32_t low_cycles = expectPulse(LOW, maxCyclesShort);
      if (low_cycles == 0) {
        //Serial2.print("!3");
        return false;
      }
      cycles[i] = low_cycles;

      uint32_t high_cycles = expectPulse(HIGH, maxCyclesLong);
      if (high_cycles == 0) {
        //Serial2.print("!4");
        return false;
      }
      cycles[i + 1] = high_cycles;
    }

    /* Timing critical code is now complete. */
  }

  pinMode(DHT_SENSOR_PIN, INPUT);
  digitalWrite(DHT_SENSOR_PIN, HIGH);

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;

  for (int i = 0; i < 40; ++i) {
    uint32_t low_cycles = cycles[2 * i];
    uint32_t high_cycles = cycles[2 * i + 1];
    if ((low_cycles == 0) || (high_cycles == 0)) {
      return false;
    }
    data[i / 8] <<= 1;
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (high_cycles > low_cycles) {
      // High cycles are greater than 50us low cycle count, must be a 1.
      data[i / 8] |= 1;
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }

  // Check we read 40 bits and that the checksum matches.
  if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    return true;
  } else {
    return false;
  }
}

static void report_cb(int64_t last_call_time) {
  byte data[5];
  if (!readData(data)) {
    return;
  }
  float t = getTemperature(data);
  float h = getHumidity(data);

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
}

bool EnvDHT22::setup() {
  D_println("setup: env_dht22");

  pdescriptor_temperature.qos = QOS_DEFAULT;
  pdescriptor_temperature.type_support = (rosidl_message_type_support_t*)
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature);
  pdescriptor_temperature.topic_name = DHT22_TOPIC_TEMPERATURE;
  micro_rosso::publishers.push_back(&pdescriptor_temperature);

  pdescriptor_humidity.qos = QOS_DEFAULT;
  pdescriptor_humidity.type_support = (rosidl_message_type_support_t*)
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, RelativeHumidity);
  pdescriptor_humidity.topic_name = DHT22_TOPIC_HUMIDITY;
  micro_rosso::publishers.push_back(&pdescriptor_humidity);

  micro_rosso::timer_report.callbacks.push_back(&report_cb);

  return true;
}

#endif // BUILD_ENV_DHT22

