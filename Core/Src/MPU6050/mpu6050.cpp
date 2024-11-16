#include "MPU6050/mpu6050.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include <string.h>


extern UART_HandleTypeDef huart2; 
extern I2C_HandleTypeDef hi2c1;

Mpu6050::Mpu6050(){

}

void Mpu6050::Init(GyroScale gyro_scale, AccelScale accel_scale){
  gyro_scale_ = gyro_scale;
  accel_scale_ = accel_scale;


  HAL_StatusTypeDef transmit_status;


  // turn on device
  uint8_t pwr_mgmt_1_register = 0x6B;
  uint8_t pwr_mgmt_1_data = 0x01; // 1 to use gryoscope mems and clock
  uint8_t data[2] = {pwr_mgmt_1_register, pwr_mgmt_1_data};
  transmit_status = HAL_I2C_Master_Transmit(&hi2c1, Mpu6050::address,
                                         data, 2, HAL_MAX_DELAY);

  // sample rate config
  uint8_t sample_rate_register = 0x19;
  uint8_t sample_rate_data =  800; //31; // 8000 Hz / (1 + 31) = 250 Hz
  uint8_t sample_rate_data_array[2] = {sample_rate_register, sample_rate_data};
  transmit_status = HAL_I2C_Master_Transmit(&hi2c1, Mpu6050::address,
                                         sample_rate_data_array, 2, HAL_MAX_DELAY);

  // set general config
  uint8_t config_register = 0x1A;
  uint8_t config_data = 0x00;
  config_data |= 0x00; // 250 Hz
  uint8_t config_data_array[2] = {config_register, config_data};
  transmit_status = HAL_I2C_Master_Transmit(&hi2c1, Mpu6050::address,
                                         config_data_array, 2, HAL_MAX_DELAY);

  // set accel config
  uint8_t accel_config_register = 0x1C;
  uint8_t accel_config = 0x00;
	accel_config |= (static_cast<uint8_t>(accel_scale_) << 3);
  uint8_t accel_data[2] = {accel_config_register, accel_config};
  transmit_status = HAL_I2C_Master_Transmit(&hi2c1, Mpu6050::address,
                                         accel_data, 2, HAL_MAX_DELAY);

  // set gyro config
  uint8_t gyro_config_register = 0x1B;
  uint8_t gyro_config = 0x00;
  gyro_config |= (static_cast<uint8_t>(gyro_scale_) << 3);
  uint8_t gyro_data[2] = {gyro_config_register, gyro_config};
  transmit_status = HAL_I2C_Master_Transmit(&hi2c1, Mpu6050::address,
                                         gyro_data, 2, HAL_MAX_DELAY);
     
  // set interrupt config
  uint8_t int_pin_config_register = 0x37;
  uint8_t int_pin_config = 0x00;
  int_pin_config |= 0x1 << 7; // active low
  int_pin_config |= 0x0 << 6; // push pull
  int_pin_config |= 0x0 << 5; // don't latch
  int_pin_config |= 0x1 << 4; // clear on any read
  int_pin_config |= 0x0 << 3; // doesn't matter (FSYNC)
  int_pin_config |= 0x0 << 2; // don't use FSYNC as int
  int_pin_config |= 0x0 << 1; // don't access axillary I2C bus
  uint8_t int_pin_data[2] = {int_pin_config_register, int_pin_config};
  transmit_status = HAL_I2C_Master_Transmit(&hi2c1, Mpu6050::address,
                                         int_pin_data, 2, HAL_MAX_DELAY);

  // int enable
  uint8_t int_enable_register = 0x38;
  uint8_t int_enable = 0x01; // YES data ready int
  uint8_t int_enable_data[2] = {int_enable_register, int_enable};
  transmit_status = HAL_I2C_Master_Transmit(&hi2c1, Mpu6050::address,
                                         int_enable_data, 2, HAL_MAX_DELAY);
}

void Mpu6050::DataReadyCallback(){
  is_data_ready_ = true;
}

void Mpu6050::ReadAccel(){
  // read accels
  uint8_t accel_data_register = 0x3B;
  HAL_StatusTypeDef transmit_status = HAL_I2C_Master_Transmit(&hi2c1, Mpu6050::address,
                                         &accel_data_register, 1, HAL_MAX_DELAY);

  uint8_t raw_accel_data[6];
  HAL_StatusTypeDef read_status = HAL_I2C_Master_Receive(&hi2c1, Mpu6050::address,
                                              raw_accel_data, 6, HAL_MAX_DELAY);

  int16_t raw_x_accel = raw_accel_data[0] << 8 | raw_accel_data[1];
  int16_t raw_y_accel = raw_accel_data[2] << 8 | raw_accel_data[3];
  int16_t raw_z_accel = raw_accel_data[4] << 8 | raw_accel_data[5];

  float max_reading = 2 << static_cast<int>(accel_scale_);
  float scale_factor = max_reading / 0x7FFF;
  float x_accel = raw_x_accel * scale_factor;
  float y_accel = raw_y_accel * scale_factor;
  float z_accel = raw_z_accel * scale_factor;

  char buffer[50];
  int len = sprintf(buffer, "x_accel %d,  y_accel %d, z_accel %d\r\n", x_accel, y_accel, z_accel);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len,  HAL_MAX_DELAY);
}

bool Mpu6050::DataReadyInterrupt(){
  uint8_t int_status_register = 0x3A;
  uint8_t int_status_data;
  HAL_StatusTypeDef transmit_status = HAL_I2C_Master_Transmit(&hi2c1, Mpu6050::address,
                                         &int_status_register, 1, HAL_MAX_DELAY);
  HAL_StatusTypeDef read_status = HAL_I2C_Master_Receive(&hi2c1, Mpu6050::address,
                                              &int_status_data, 1, HAL_MAX_DELAY);
  return int_status_data & 0x01; // 1 if data ready, 0 if data has been read
}

void Mpu6050::ReadGyro(){

  uint8_t gyro_data[6];

  uint8_t gyro_data_register = 0x43;

  HAL_StatusTypeDef transmit_status = HAL_I2C_Master_Transmit(&hi2c1, Mpu6050::address,
                                         &gyro_data_register, 1, HAL_MAX_DELAY);
  HAL_StatusTypeDef read_status = HAL_I2C_Master_Receive(&hi2c1, Mpu6050::address,
                                              gyro_data, 6, HAL_MAX_DELAY);

  int16_t gyro_x_raw = gyro_data[0] << 8 | gyro_data[1];
  int16_t gyro_y_raw = gyro_data[2] << 8 | gyro_data[3];
  int16_t gyro_z_raw = gyro_data[4] << 8 | gyro_data[5];

  float max_reading = 250.0 * (1 << static_cast<int>(gyro_scale_));
  float scale_factor = max_reading /  0x7FFF ;
  float gyro_x_dps = gyro_x_raw * scale_factor;
  float gyro_y_dps = gyro_y_raw * scale_factor;
  float gyro_z_dps = gyro_z_raw * scale_factor;



}

void Mpu6050::ReadIfReady(){
  if (is_data_ready_){
    ReadAccel();
    ReadGyro(); 
    is_data_ready_ = false;
  }  else{

  }
}
