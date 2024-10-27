#ifndef __imu_bno08x_h
#define __imu_bno08x_h

#include <Wire.h>

#define BNO08X_TOPIC_IMU "/imu/raw"
#define BNO08X_TOPIC_MAG "/mag"

class ImuBNO08x {
public:
  ImuBNO08x();
  static bool setup( TwoWire &wire = Wire );
};

#endif  // __imu_bno08x_h
