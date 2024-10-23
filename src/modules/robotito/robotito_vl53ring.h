#ifndef __robotito_vl53ring
#define __robotito_vl53ring

#define VL53RING_TOPIC_LASER_SCAN "/laser_scan"

class Vl53Ring {
public:
  Vl53Ring();
  static bool setup();
};

#endif  // __robotito_vl53ring
