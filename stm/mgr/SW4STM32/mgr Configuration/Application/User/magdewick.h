/*
 * magdewick.h
 *
 *  Created on: 20 maj 2017
 *      Author: Micha³
 */

#ifndef APPLICATION_USER_MAGDEWICK_H_
#define APPLICATION_USER_MAGDEWICK_H_
#include "tm_stm32f4_l3gd20.h"
#include <math.h>
// System constants
//#define deltat 0.005f // sampling period in seconds (shown as 1 ms)
//#define gyroMeasError 3.14159265358979f * (1.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
//#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta

typedef struct {
	float SEq_1 ;
	float SEq_2 ;
	float SEq_3;
	float SEq_4 ;
} Quaternion;

#endif /* APPLICATION_USER_MAGDEWICK_H_ */
