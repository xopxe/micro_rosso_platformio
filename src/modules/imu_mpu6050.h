#ifndef __imu_mpu6050_h
#define __imu_mpu6050_h

#define MPU6050_TOPIC_IMU "/imu/raw"
#define MPU6050_TOPIC_TEMPERATURE "/imu/temperature"

class ImuMPU6050 {
public:
  ImuMPU6050();
  static bool setup();
};

#endif  // __imu_mpu6050_h
