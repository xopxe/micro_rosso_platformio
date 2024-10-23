#ifndef __robotito_omni_h
#define __robotito_omni_h

#include "micro_rosso.h"

#define ROBOTITO_TOPIC_CMD_VEL "/cmd_vel"
#define ROBOTITO_TOPIC_JOY "/joy"

class RobotitoOmni {
public:
  RobotitoOmni();

  static bool setup();

};

#endif  // __robotito_omni_h
