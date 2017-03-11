#include "stm32f3xx_hal.h"


void my_main_loop()
{

	HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7);
	HAL_Delay(500);
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
}
