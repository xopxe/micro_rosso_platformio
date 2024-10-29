#include "micro_rosso.h"

#include "ros_status.h"

#if defined(LED_ROS2_PIN)
#define LED_ROS2_CONFIGURE() pinMode(LED_ROS2_PIN, OUTPUT);
#define LED_ROS2(on) 
#else
#define LED_ROS2_CONFIGURE(...)
#define LED_ROS2(...) 
#endif


RosStatus::RosStatus() {
};

static void ros_state_cb(ros_states state) {
  switch (state) {
    case WAITING_AGENT:
      LED_ROS2(0);
      break;
    case AGENT_AVAILABLE:
      LED_ROS2(0);
      break;
    case AGENT_CONNECTED:
      LED_ROS2(1);
      break;
    case AGENT_DISCONNECTED:
      LED_ROS2(0);   
      break;
    default:
      break;
  }
}

bool RosStatus::setup() {
  D_println("setup: ros_status");
  
  LED_ROS2_CONFIGURE();

  micro_rosso::ros_state_listeners.push_back(ros_state_cb);

  return true;
}


