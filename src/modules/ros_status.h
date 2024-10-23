#ifndef __ros_status_h
#define __ros_status_h

//#define LED_ROS2_PIN 5       // gets set to 1 when ROS2 is connected

class RosStatus {
public:
  RosStatus();
  static bool setup();
};

#endif  // __ros_status_h
