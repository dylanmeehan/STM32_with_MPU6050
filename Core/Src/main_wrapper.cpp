#include "main_wrapper.h"
#include "MPU6050/mpu6050.h"

extern "C" {

static Mpu6050 mpu6050;

extern TIM_HandleTypeDef htim1;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == TIM1){
    // when timer rolls over (arr), set green led on
    HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_SET);
  }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == TIM1){
    // when timer1 hits OC value, set green led off
    HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_RESET);
  }
}

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
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);  

  mpu6050.Init(GyroScale::GYRO_250DPS, AccelScale::ACCEL_2G);
}

void Main_Wrapper_Loop(){
  
  mpu6050.ReadIfReady();

}


} // extern "C"
