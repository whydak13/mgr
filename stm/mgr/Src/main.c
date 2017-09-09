/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
typedef int bool;
#define true 1
#define false 0

#define WHEEL_INTERTIA 2.2456e-05
bool motors_on=false;
bool battery_ok=true;
uint16_t Analog[2];
float Voltage=0;
float Current=0;
float BalancePoint= 88.57;
float time_s=0;
float angle=0;
float angle_correction;
float comp_gain=0.005;
float set_point=0;
float speed_set_point=0;
float speed=0;
float speed_integral=0;
float P=68;//40
float I=36;
float D=1.8;//1.6 z kablem 1.8 spoko bez
float P_2=3;
float I_2=0.22;
float error;
float integral;
float derivative;
#include <math.h>
#include <limits.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "LIB_Config.h"
#include "G:\Studia\Mgr\mgr\stm\mgr\SW4STM32\mgr Configuration\Application\User\steppers.h"
#include "G:\Studia\Mgr\mgr\stm\mgr\SW4STM32\mgr Configuration\Application\User\magdewick.h"
#include "G:\Studia\Mgr\mgr\stm\mgr\SW4STM32\mgr Configuration\Application\User\MadgwickAHRS.h"
#include "my_interupts.h"
#include "tm_stm32f4_l3gd20.h"




/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint32_t steppers_cnt=0;
uint32_t time_to_next_step=2;
int8_t stepper_direction=1;
float acceleration=0.1;
TM_L3GD20_t L3GD20_Data;
Acceleration_G_data LSM303_Data;
// Global system variables
//Quaternion Magdewick_res={1,0,0,0};

// estimated orientation quaternion elements with initial conditions
float gyro_X_angle =0;
float gyro_Y_angle =0;
float gyro_Z_angle =0;
float gyro_X_speed =0;
float gyro_Y_speed =0;
float gyro_Z_speed =0;
float accel_angle;
float pitch=0;
float roll=0;
/// MPC VARIABLES/////////////////////
float Np = 15;
const float r[2]={0, 0};
const float F[180]={
		0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,
		-0.019154,1.002771,-0.035982,2.011563,-0.045803,3.028297,-0.047792,4.054551,-0.041997,5.091798,-0.028647,6.141457,-0.007993,7.204910,0.019725,8.283512,0.054284,9.378592,0.095486,10.491456,0.143148,11.623393,0.197107,12.775676,0.257217,13.949564,0.323350,15.146308,0.395391,16.367150,
		0.023148,0.001353,0.040760,0.005149,0.056536,0.011564,0.071371,0.020569,0.085515,0.032095,0.099059,0.046070,0.112062,0.062424,0.124569,0.081097,0.136622,0.102032,0.148260,0.125179,0.159519,0.150495,0.170433,0.177942,0.181036,0.207486,0.191358,0.239100,0.201428,0.272760,
		0.012898,0.003998,0.028192,0.011083,0.043501,0.020974,0.058316,0.033541,0.072557,0.048693,0.086243,0.066356,0.099414,0.086465,0.112111,0.108961,0.124373,0.133792,0.136240,0.160914,0.147748,0.190286,0.158930,0.221875,0.169819,0.255651,0.180445,0.291590,0.190838,0.329673,
		1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,
		0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000,0.000000,1.000000};
float X_act[6] ={
		  0.000000, //phi
		  0.000000, //etha
		  0.000000, //d phi
		  0.000000, //d etha
		  0.000000, // wheel speed out
		  0.000000 }; //etha out
float X_prev[6];
float X_diff[6];
const float Phi[90]={
		-0.259245,0.020149,-0.309054,0.058948,-0.310690,0.104093,-0.301390,0.152728,-0.289734,0.204182,-0.277682,0.258295,-0.265665,0.315027,-0.253760,0.374371,-0.241966,0.436329,-0.230263,0.500912,-0.218628,0.568132,-0.207037,0.638008,-0.195470,0.710562,-0.183904,0.785819,-0.172319,0.863809,
		0.000000,0.000000,-0.259245,0.020149,-0.309054,0.058948,-0.310690,0.104093,-0.301390,0.152728,-0.289734,0.204182,-0.277682,0.258295,-0.265665,0.315027,-0.253760,0.374371,-0.241966,0.436329,-0.230263,0.500912,-0.218628,0.568132,-0.207037,0.638008,-0.195470,0.710562,-0.183904,0.785819,
		0.000000,0.000000,0.000000,0.000000,-0.259245,0.020149,-0.309054,0.058948,-0.310690,0.104093,-0.301390,0.152728,-0.289734,0.204182,-0.277682,0.258295,-0.265665,0.315027,-0.253760,0.374371,-0.241966,0.436329,-0.230263,0.500912,-0.218628,0.568132,-0.207037,0.638008,-0.195470,0.710562};
float f[3];
const float H[9]={
		  4.375610,3.925508,3.476743,
		  3.925508,3.599751,3.215021,
		  3.476743,3.215021,2.948419,
		  };
const float A_cons[36]={
		1.000000,1.000000,1.000000,-1.000000,-1.000000,-1.000000,1.000000,0.000000,0.000000,-1.000000,-0.000000,-0.000000,
		0.000000,1.000000,1.000000,-0.000000,-1.000000,-1.000000,0.000000,1.000000,0.000000,-0.000000,-1.000000,-0.000000,
		0.000000,0.000000,1.000000,-0.000000,-0.000000,-1.000000,0.000000,0.000000,1.000000,-0.000000,-0.000000,-1.000000};
const float b[12]={
		  0.200000,
		  0.200000,
		  0.200000,
		  0.200000,
		  0.200000,
		  0.200000,
		  0.500000,
		  0.500000,
		  0.500000,
		  0.500000,
		  0.500000,
		  0.500000,
		  };
float eta[3];
/////////////////////MPc VARIABLES END ////////////////////////////////////////////
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if ((motors_on==true)&&(battery_ok==true))
	{	motors_on=false;
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);}
	else
		motors_on=true;

	//acceleration=-acceleration;
	speed_integral=0;
	integral=0;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
 if(htim->Instance == TIM6){ // Je¿eli przerwanie pochodzi od timera 6 100Hz
	 float const dt=0.01;
	 time_s+=dt;

	 //HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
	  // Pobranie 6 bajt7ow danych zawierajacych przyspieszenia w 3 osiach
	  	TM_L3GD20_Read(&L3GD20_Data);
	 	getAcceleration(&LSM303_Data);
	 	gyro_X_speed =((float)(L3GD20_Data.X)*L3GD20_SENSITIVITY_250 * 0.001)*0.0174532925;
	 	gyro_Y_speed =((float)(L3GD20_Data.Y)*L3GD20_SENSITIVITY_250 * 0.001)*0.0174532925;
	 	gyro_Z_speed =((float)(L3GD20_Data.Z)*L3GD20_SENSITIVITY_250 * 0.001)*0.0174532925;

		if(time_to_next_step<5)
			time_to_next_step=5;

		speed=(float)((GET_WHEEL_SPEED_FACTOR*stepper_direction)/time_to_next_step);
		speed_integral+= speed*dt;

	 	gyro_X_angle+=gyro_X_speed *dt;
		accel_angle=atan2f(LSM303_Data.X,LSM303_Data.Z)*-57.2957795;

		angle=comp_gain*accel_angle+(1-comp_gain)*(angle+((float)(L3GD20_Data.X)*L3GD20_SENSITIVITY_250 * 0.001)*dt);//-BalancePoint;

		angle_correction=angle-BalancePoint;
		derivative=((set_point-angle_correction)-error)/dt ;
		error = set_point-angle_correction;
		integral += error*dt;

		X_act[0]=angle_correction;
		X_act[1]=speed_integral;
		X_act[2]=gyro_X_speed;
		X_act[3]=speed;
		X_act[4]=speed* R_WHEEL;
		X_act[5]=angle_correction;

		float X_prev[6];

		for(int i=0;i<4;i++)
		{
			X_diff[i]=X_act[i]-X_prev[i];
		}
		X_diff[4]=speed* R_WHEEL;
		X_diff[5]=angle_correction;
		/* get_f(Np,  r,  F, X_diff,Phi, f);
		 QPhild2(H, f, A_cons,b, eta);
		// acceleration = (P*error + I*integral + D*derivative);//*stepper_direction ;
		 for(int i=0;i<4;i++)
		 {
		 	X_prev[i]=X_act[i];
		 }

		 if (motors_on)
		 {
			 if (fabs(error-set_point)>30)
			 { HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
				speed_integral=0;
				integral=0;}
			 else
				 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
		 } else HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
		 acceleration+= eta[0]/WHEEL_INTERTIA;*/
	 	if (acceleration< -300)
	 		acceleration=-300;
	 	if (acceleration >300)
	 		acceleration=300;

	 	Voltage=3*(((float)Analog[0])/409);
	 	Current=3*(((float)Analog[1])/409);

 }
 if(htim->Instance == TIM7){ // Je¿eli przerwanie pochodzi od timera 7 100 kHz
	 steppers_cnt++;
	 if(steppers_cnt>time_to_next_step)
	 {
		 make_step(stepper_direction);
		 time_to_next_step=calculate_next_step(&acceleration,&stepper_direction);
		 steppers_cnt=0;

	 }
 }
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* L3GD20 Struct */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();

  /* USER CODE BEGIN 2 */
  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_3);
 //HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_3); //////////TUUUUUUUU
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_ADC_Start_DMA(&hadc3, Analog, 2);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
//GYRO INIT
  TM_L3GD20_Init(TM_L3GD20_Scale_250);


  //Akcelerometr
  // wypelnieine zmiennej konfiguracyjnej odpowiednimi opcjami
   uint8_t Settings = LSM303_ACC_XYZ_ENABLE | LSM303_ACC_100HZ;
   // Wpisanie konfiguracji do rejestru akcelerometru
    HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG1_A, 1, &Settings, 1, 100);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */




  while (1)
  {
		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);
		HAL_Delay(500);
		if (Voltage <10.5)
		{	battery_ok=false;
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_10);
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_11);
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_12);
		}
	//	TM_L3GD20_INT_ReadSPI(L3GD20_REG_WHO_AM_I) ;
		 //TM_L3GD20_Read(&L3GD20_Data);
	 // float x =my_regulator_ict(0,&hi2c1);
	 //my_main_loop();
	 /* steppers_cnt++;
	  	 if(steppers_cnt>time_to_next_step)
	  	 {
	  		 make_step(stepper_direction);
	  		 time_to_next_step=calculate_next_step(acceleration,stepper_direction);
	  		 steppers_cnt=0;
	  	 }*/
	//TM_L3GD20_Read(&L3GD20_Data);

	 //float x =my_regulator_ict(0,&hi2c1);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC3 init function */
void MX_ADC3_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION12b;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc3);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

    /**Configure Analogue filter 
    */
  HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLED;
  HAL_SPI_Init(&hspi1);

}

/* TIM6 init function */
void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 359;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 499;
  HAL_TIM_Base_Init(&htim6);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

}

/* TIM7 init function */
void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 71;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9;
  HAL_TIM_Base_Init(&htim7);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA2_CLK_ENABLE();
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA11   ------> USB_DM
     PA12   ------> USB_DP
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOE_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin 
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin 
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin 
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin 
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 
                           PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DM_Pin DP_Pin */
  GPIO_InitStruct.Pin = DM_Pin|DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF14_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
