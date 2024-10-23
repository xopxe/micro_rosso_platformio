#ifndef __mobility_tracked_h
#define __mobility_tracked_h

#define TRACKED_TOPIC_CMD_VEL "/cmd_vel"
#define TRACKED_TOPIC_JOY "/joy"

class MobilityTracked {
public:
  MobilityTracked();

  static bool setup();

  static void set_motor_enable(bool enable);  // TODO
  static void stop();
};

#endif  // __mobility_tracked_h
