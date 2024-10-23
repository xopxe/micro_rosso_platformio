#include <Arduino.h>
#include <micro_ros_platformio.h>

#include "micro_rosso.h"

#include "modules/robotito/robotito_omni.h"
RobotitoOmni mobility;

#include "modules/robotito/robotito_apds9960.h"
Apds9960 apds9960;

#include "modules/robotito/robotito_vl53ring.h"
Vl53Ring laser;

#include "modules/ticker.h"
Ticker ticker;

#include "modules/sync_time.h"
SyncTime sync_time;

void setup() {

  delay(2000);

  D_SerialBegin(DEBUG_CONSOLE_BAUD, SERIAL_8N1, DEBUG_CONSOLE_PIN_RX, DEBUG_CONSOLE_PIN_TX);

  D_println("Booting...");

  if (!micro_rosso::setup()) {
    D_println("FAIL micro_rosso.setup()");
  }
/*
  if (!imu.setup()) {
    D_println("FAIL imu.setup()");
  };

  if (!env_sensor.setup()) {
    D_println("FAIL env_sensor.setup()");
  };
*/

  if (!mobility.setup()) {
    D_println("FAIL mobility.setup()");
  };
  if (!apds9960.setup()) {
    D_println("FAIL apds9960.setup()");
  };
  if (!laser.setup()) {
    D_println("FAIL laser.setup()");
  };

  if (!ticker.setup()) {
    D_println("FAIL ticker.setup()");
  };
  
  if (!sync_time.setup()) {
    D_println("FAIL sync_time.setup()");
  };

  D_println("Boot completed.");
}

void loop() {
  micro_rosso::loop();
}
