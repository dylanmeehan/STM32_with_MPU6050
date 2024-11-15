#pragma once

#include <stdint.h>

enum class GyroScale {
  GYRO_250DPS = 0,
  GYRO_500DPS = 1,
  GYRO_1000DPS = 2,
  GYRO_2000DPS = 3
};

enum class AccelScale {
  ACCEL_2G = 0,
  ACCEL_4G = 1,
  ACCEL_8G = 2,
  ACCEL_16G = 3
};

class Mpu6050{
 public:
  Mpu6050();
  void Init(GyroScale gyro_scale, AccelScale accel_scale);
  void Read();

 private:

  void ReadAccel();
  void ReadGyro();

  GyroScale gyro_scale_ = GyroScale::GYRO_250DPS;
  AccelScale accel_scale_ = AccelScale::ACCEL_2G;

  static const uint8_t address = (0x68 << 1);
  static const int16_t range_Gs = 2;

};
