/*
 * steppers.h
 *
 *  Created on: 29 mar 2017
 *      Author: Micha³
 */
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })


#ifndef APPLICATION_USER_STEPPERS_H_
#define APPLICATION_USER_STEPPERS_H_

#include "stm32f3xx_hal.h"
#include <math.h>
#include <limits.h>

#define MCU_FREQ 72000000
#define MAX_MOTOR_SPEED 10000
#define MIN_MOTOR_SPEED 10
#define MAX_MOTOR_ACCELERATION 5

#define STEPPER_DRIVER_FREQUENCY 10000.0
#define MOTOR_STEP_DIVIDER 8.0
#define MOTOR_STEPS_PER_RESOLUION 200.0
#define MOTOR_STEP_ANGLE 360.0/(MOTOR_STEPS_PER_RESOLUION*MOTOR_STEP_DIVIDER)

#define MIN_MOTOR_DELAY (MCU_FREQ*(1/MAX_MOTOR_SPEED))/STEPPER_DRIVER_FREQUENCY
#define MAX_MOTOR_DELAY (MCU_FREQ*(1/MIN_MOTOR_SPEED))/STEPPER_DRIVER_FREQUENCY

#define MOTOR_A_DIRECTION_FORWARD	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET)
#define MOTOR_A_DIRECTION_BACKWARD	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET)

#define MOTOR_B_DIRECTION_FORWARD	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET)
#define MOTOR_B_DIRECTION_BACKWARD	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)

#define MOTOR_A_STEP_PIN_PORT GPIOC
#define MOTOR_A_STEP_PIN_PIN  GPIO_PIN_2

#define MOTOR_B_STEP_PIN_PORT GPIOC
#define MOTOR_B_STEP_PIN_PIN  GPIO_PIN_3

uint32_t calculate_next_step(float *, int8_t* ,float *);
void steppers_init(int step_divider);
void make_step(int8_t direction);
void delay_200ns();


#endif /* APPLICATION_USER_STEPPERS_H_ */
