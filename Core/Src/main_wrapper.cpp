#include "main_wrapper.h"
#include "MPU6050/mpu6050.h"

extern "C" {

static Mpu6050 mpu6050;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  switch(GPIO_Pin){
    case MPU_INT_Pin:
      mpu6050.DataReadyCallback();
      break;
    default:
      break;
  }

}

void Main_Wrapper_Init(){
  mpu6050.Init(GyroScale::GYRO_2000DPS, AccelScale::ACCEL_16G);
}

void Main_Wrapper_Loop(){
  
  mpu6050.ReadIfReady();

  // LED blink
  HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_RESET);
  HAL_Delay(300);


}


} // extern "C"
