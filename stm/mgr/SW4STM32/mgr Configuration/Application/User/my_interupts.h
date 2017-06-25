/*
 * my_interupts.h
 *
 *  Created on: 11 mar 2017
 *      Author: Micha³
 */

#ifndef APPLICATION_USER_MY_INTERUPTS_H_
#define APPLICATION_USER_MY_INTERUPTS_H_

#include "stm32f3xx_hal.h"
#include <limits.h>

////////////////////////////AKCELEROMETR
extern I2C_HandleTypeDef hi2c1;
#define LSM303_ACC_ADDRESS (0x19 << 1) // adres akcelerometru: 0011001x
#define LSM303_ACC_CTRL_REG1_A 0x20 // rejestr ustawien 1
#define LSM303_ACC_Z_H_A 0x2D // wyzszy bajt danych osi Z
#define LSM303_ACC_Z_L_A 0x2C // nizszy bajt danych osi Z

// mlodszy bajt danych osi Z z najstarszym bitem ustawionym na 1 w celu
// wymuszenia autoinkrementacji adresow rejestru w urzadzeniu docelowym
#define LSM303_ACC_Z_L_A_MULTI_READ (LSM303_ACC_Z_L_A | 0x80)

// Maski bitowe
// CTRL_REG1_A = [ODR3][ODR2][ODR1][ODR0][LPEN][ZEN][YEN][XEN]
#define LSM303_ACC_Z_ENABLE 0x07 // 0000 0100
//#define LSM303_ACC_100HZ 0x50 //0101 0000
#define LSM303_ACC_100HZ 0x60 //0110 0000 200HZ
#define LSM303_ACC_RESOLUTION 2.0 // Maksymalna wartosc przyspieszenia [g]

#define LSM303_ACC_X_L_A 0x28 // mlodszy bajt danych osi X

// mlodszy bajt danych osi X z najstarszym bitem ustawionym na 1 w celu
// wymuszenia autoinkrementacji adresow rejestru w urzadzeniu docelowym
// (zeby moc odczytac wiecej danych na raz)
#define LSM303_ACC_X_L_A_MULTI_READ (LSM303_ACC_X_L_A | 0x80)

#define LSM303_ACC_XYZ_ENABLE 0x07 // 0000 0111
////////////////////////////AKCELEROMETR KONIEC

typedef struct {
	float X; /*!< X axis rotation */
	float Y; /*!< Y axis rotation */
	float Z; /*!< Z axis rotation */
} Acceleration_G_data;

void  getAcceleration(Acceleration_G_data *);

#endif /* APPLICATION_USER_MY_INTERUPTS_H_ */
