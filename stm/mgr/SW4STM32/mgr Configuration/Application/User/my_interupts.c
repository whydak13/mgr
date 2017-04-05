/*
 * my_interupts.c
 *
 *  Created on: 8 mar 2017
 *      Author: Micha³
 */
#include "my_interupts.h"
// Zmienne
uint8_t Data[6]; // Zmienna do bezposredniego odczytu danych z akcelerometru
int16_t Xaxis = 0; // Zawiera przeksztalcona forme odczytanych danych z osi X
int16_t Yaxis = 0; // Zawiera przeksztalcona forme odczytanych danych z osi Y
int16_t Zaxis = 0; // Zawiera przeksztalcona forme odczytanych danych z osi Z

float Xaxis_g = 0; // Zawiera przyspieszenie w osi X przekstalcone na jednostke fizyczna [g]
float Yaxis_g = 0; // Zawiera przyspieszenie w osi Y przekstalcone na jednostke fizyczna [g]
float Zaxis_g = 0; // Zawiera przyspieszenie w osi Z przekstalcone na jednostke fizyczna [g]

void  my_regulator_ict(float *acceleration)
{

	  // Pobranie 6 bajt7ow danych zawierajacych przyspieszenia w 3 osiach
	   HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_X_L_A_MULTI_READ, 1, Data, 6, 100);

	   // Konwersja odebranych bajtow danych na typ int16_t
	   Xaxis = ((Data[1] << 8) | Data[0]);
	   Yaxis = ((Data[3] << 8) | Data[2]);
	   Zaxis = ((Data[5] << 8) | Data[4]);

	   // obliczenie przyspieszen w kazdej z osi w jednostce SI [g]
	   Xaxis_g = ((float) Xaxis * LSM303_ACC_RESOLUTION) / (float) INT16_MAX;
	   Yaxis_g = ((float) Yaxis * LSM303_ACC_RESOLUTION) / (float) INT16_MAX;
	   Zaxis_g = ((float) Zaxis * LSM303_ACC_RESOLUTION) / (float) INT16_MAX;
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
	HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_11);//led
	*acceleration= 0;

}

void my_motor_driver_ict()
{
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7);
}
