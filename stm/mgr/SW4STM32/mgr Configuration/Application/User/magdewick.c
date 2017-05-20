#include "magdewick.h"
/*
void filterUpdate(TM_L3GD20_t *gyro_data, Acceleration_G_data *accel_data, Quaternion *Magdewick_res)
{
// Local system variables
	float w_x=(float)(gyro_data->X)*L3GD20_SENSITIVITY_250 * 0.001;
	float w_y=(float)(gyro_data->Y)*L3GD20_SENSITIVITY_250 * 0.001;
	float w_z=(float)(gyro_data->Z)*L3GD20_SENSITIVITY_250 * 0.001;
	float a_x=accel_data->X,  a_y=accel_data->Y,  a_z=accel_data->Z;
	float norm; // vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
	float f_1, f_2, f_3; // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
	// Axulirary variables to avoid reapeated calcualtions
	float halfSEq_1 = 0.5f * Magdewick_res->SEq_1;
	float halfSEq_2 = 0.5f * Magdewick_res->SEq_2;
	float halfSEq_3 = 0.5f * Magdewick_res->SEq_3;
	float halfSEq_4 = 0.5f * Magdewick_res->SEq_4;
	float twoSEq_1 = 2.0f * Magdewick_res->SEq_1;
	float twoSEq_2 = 2.0f * Magdewick_res->SEq_2;
	float twoSEq_3 = 2.0f * Magdewick_res->SEq_3;

	// Normalise the accelerometer measurement
	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;
	// Compute the objective function and Jacobian
	f_1 = twoSEq_2 * Magdewick_res->SEq_4 - twoSEq_1 * Magdewick_res->SEq_3 - a_x;
	f_2 = twoSEq_1 * Magdewick_res->SEq_2 + twoSEq_3 * Magdewick_res->SEq_2 - a_y;
	f_3 = 1.0f - twoSEq_2 * Magdewick_res->SEq_2 - twoSEq_3 * Magdewick_res->SEq_3 - a_z;
	J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
	J_12or23 = 2.0f * Magdewick_res->SEq_4;
	J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21; // negated in matrix multiplication
	J_33 = 2.0f * J_11or24; // negated in matrix multiplication
	// Compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
	// Normalise the gradient
	norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 /= norm;
	SEqHatDot_2 /= norm;
	SEqHatDot_3 /= norm;
	SEqHatDot_4 /= norm;
	// Compute the quaternion derrivative measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
	// Compute then integrate the estimated quaternion derrivative
	Magdewick_res->SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
	Magdewick_res->SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
	Magdewick_res->SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
	Magdewick_res->SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
	// Normalise quaternion
	norm = sqrt(Magdewick_res->SEq_1 * Magdewick_res->SEq_1 + Magdewick_res->SEq_2 * Magdewick_res->SEq_2 + Magdewick_res->SEq_3 * Magdewick_res->SEq_3 + Magdewick_res->SEq_4 * Magdewick_res->SEq_4);
	Magdewick_res->SEq_1 /= norm;
	Magdewick_res->SEq_2 /= norm;
	Magdewick_res->SEq_3 /= norm;
	Magdewick_res->SEq_4 /= norm;
}*/
