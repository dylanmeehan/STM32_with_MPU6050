#include "MPU6050/mpu6050.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <string.h>


extern UART_HandleTypeDef huart2;  

Mpu6050::Mpu6050(){
 
}

void Mpu6050::Init(){
 
}

void Mpu6050::Read(){

  float x_accel, y_accel, z_accel;
 
  z_accel = 1;

  char buffer[50];
  int len = sprintf(buffer, "z_accel: %d\r\n", z_accel);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len,  HAL_MAX_DELAY);

  z_accel = 0;
   
}
