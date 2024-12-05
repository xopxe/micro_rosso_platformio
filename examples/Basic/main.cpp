#include <Arduino.h>

#include "micro_rosso.h"

#include "ticker.h"
Ticker ticker;

#include "sync_time.h"
SyncTime sync_time;

#include "ros_status.h"
RosStatus ros_status;

void setup()
{
  D_println("Booting...");

  D_print("Setting up transport... ");
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  if (!micro_rosso::setup("my_node_name"))
  {
    D_println("FAIL micro_rosso.setup()");
  }

  if (!ticker.setup())
  {
    D_println("FAIL ticker.setup()");
  };

  if (!sync_time.setup())
  {
    D_println("FAIL sync_time.setup()");
  };

  if (!ros_status.setup())
  {
    D_println("FAIL ros_status.setup()");
  };

  D_println("Boot completed.");
}

void loop()
{
  micro_rosso::loop();
}
