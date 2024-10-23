#ifndef __odom_helper_h
#define __odom_helper_h

#define ODOM_TOPIC_ODOM "/odom"

#include "micro_rosso.h"
#include <nav_msgs/msg/odometry.h>

typedef struct {
  float x;
  float y;
  float phi;
} odom_t;

class OdomHelper {
public:
  OdomHelper();
  static bool setup();
  
  static void update_pos( float vx, float vy, float vphi, float dt);
  static void set( float x, float y, float phi, float vx, float vy, float vphi );
  
  static void reset();
     
  static odom_t pos;
  static odom_t vel;
};



#endif  // __odom_helper_h
