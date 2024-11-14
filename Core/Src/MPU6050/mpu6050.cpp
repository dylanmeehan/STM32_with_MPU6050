#include "MPU6050/mpu6050.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include <string.h>


extern UART_HandleTypeDef huart2; 
extern I2C_HandleTypeDef hi2c1;

Mpu6050::Mpu6050(){

}

void Mpu6050::Init(){

  uint8_t pwr_mgmt_1_register = 0x6B;
  uint8_t pwr_mgmt_1_data = 0x01; // 1 to use gryoscope mems and clock
  uint8_t data[2] = {pwr_mgmt_1_register, pwr_mgmt_1_data};
  HAL_StatusTypeDef transmit_status = HAL_I2C_Master_Transmit(&hi2c1, Mpu6050::address,
                                         data, 2, HAL_MAX_DELAY);
}

void Mpu6050::Read(){

  // float x_accel, y_accel, z_accel;
  uint8_t accel_data_register = 0x3B;
  HAL_StatusTypeDef transmit_status = HAL_I2C_Master_Transmit(&hi2c1, Mpu6050::address,
                                         &accel_data_register, 1, HAL_MAX_DELAY);
    
  uint8_t raw_accel_data[6];
  HAL_StatusTypeDef read_status = HAL_I2C_Master_Receive(&hi2c1, Mpu6050::address,
                                              raw_accel_data, 6, HAL_MAX_DELAY);

  int16_t raw_x_accel = raw_accel_data[0] << 8 | raw_accel_data[1];
  int16_t raw_y_accel = raw_accel_data[2] << 8 | raw_accel_data[3];
  int16_t raw_z_accel = raw_accel_data[4] << 8 | raw_accel_data[5];

  char buffer[50];
  int len = sprintf(buffer, "x_accel %d,  y_accel %d, z_accel %d\r\n", raw_x_accel, raw_y_accel, raw_z_accel);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len,  HAL_MAX_DELAY);

   
}
