/*
 * my_interupts.c
 *
 *  Created on: 8 mar 2017
 *      Author: Micha³
 */

#include "stm32f3xx_hal.h"
#define MAX_SPEED 10000; // Max speed of motor in pulses per second
#define STEP_ANGLE 1.8; // Motor step angle in degrees
#define N 1000;
#define STEP_DIVIDER 8; // full step mode=1, half stepping =2 ect
#define MOTOR_DRIVER_FREQ 10000;

float  my_regulator_ict(float acceleration)
{


	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
	return 0;

}

void my_motor_driver_ict()
{
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7);
}
