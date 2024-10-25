#include <Arduino.h>
#include <micro_ros_platformio.h>

#include "micro_rosso.h"

#include "modules/ticker.h"
Ticker ticker;

#include "modules/sync_time.h"
SyncTime sync_time;

#include "modules/ros_status.h"
RosStatus ros_status;

void setup() {
  delay(2000);

  D_SerialBegin(DEBUG_CONSOLE_BAUD, SERIAL_8N1, DEBUG_CONSOLE_PIN_RX, DEBUG_CONSOLE_PIN_TX);

  D_println("Booting...");

  if (!micro_rosso::setup()) {
    D_println("FAIL micro_rosso.setup()");
  }

  if (!ticker.setup()) {
    D_println("FAIL ticker.setup()");
  };
  
  if (!sync_time.setup()) {
    D_println("FAIL sync_time.setup()");
  };

  if (!ros_status.setup()) {
    D_println("FAIL ros_status.setup()");
  };

  D_println("Boot completed.");
}

void loop() {
  micro_rosso::loop();
}
