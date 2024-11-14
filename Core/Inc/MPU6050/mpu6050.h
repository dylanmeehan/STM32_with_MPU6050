#pragma once

#include <stdint.h>

class Mpu6050{
 public:
  Mpu6050();
  void Init();
  void Read();

 private:
  static const uint8_t address = (0x68 << 1);

};
