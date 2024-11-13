#include "main_wrapper.h"

extern "C" {

void Main_Wrapper_Init(){

}

void Main_Wrapper_Loop(){
  HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_RESET);
  HAL_Delay(300);
}


} // extern "C"