#include "steppers.h"

uint32_t calculate_next_step(float* acceleration,int8_t* stepper_direction)
{
	static float c=0;
	static float n=1;
	static float accleration_prev=1;
	uint32_t tmp=0;
	if ((*acceleration)==0)
	{
		(*acceleration)=accleration_prev;
	}

	if ((c==0)||(fabs(n)<3)) // Init
	{
		c=(float)STEPPER_DRIVER_FREQUENCY*sqrtf((2*MOTOR_STEP_ANGLE)/fabs(*acceleration));
		if ((*acceleration) > 0)
			(*stepper_direction)=1;
		else
			(*stepper_direction)=-1;
		if(c>5010)
			c=5005;
		accleration_prev=(*acceleration);
		n=3;
		//n++;
		return (uint32_t)c;
	}

	if ((*acceleration)!=accleration_prev)
    {
        n=(n*accleration_prev)/(*acceleration);
    }
    else
    {
    	n++;
    }
	tmp=(uint32_t)(c - (2*c)/(4*n+1));
	if (tmp>50)
	{
		c=tmp;
	}
	else
	{
		n--;
	}

	if(c>1010)
			c=1005;

    accleration_prev=(*acceleration);
    return (uint32_t)c;

}

void limit_motor_speed(uint32_t* time_to_next_step, int8_t* direction, float* acceleration)
{
	*time_to_next_step=max(*time_to_next_step,MIN_MOTOR_DELAY);
	if(*time_to_next_step>MAX_MOTOR_DELAY)
	{
		*time_to_next_step=MAX_MOTOR_DELAY;
		/*if((*acceleration)<0)// point of 0 speed reached so its time to start accelerating in opossite direction
		{
			*direction=-(*direction);
			*acceleration=-(*acceleration);
		}*/
	}
}

void steppers_init(int step_divider){
	// Set pins for motor mode, full step, half step ect..
	delay_200ns();
}
void make_step(int8_t direction){
	if(direction>0)
	{
		MOTOR_A_DIRECTION_FORWARD;
		MOTOR_B_DIRECTION_FORWARD;
	}else
	{
		MOTOR_A_DIRECTION_BACKWARD;
		MOTOR_B_DIRECTION_BACKWARD;
	}
	HAL_GPIO_TogglePin(MOTOR_A_STEP_PIN_PORT,MOTOR_A_STEP_PIN_PIN);
	HAL_GPIO_TogglePin(MOTOR_B_STEP_PIN_PORT,MOTOR_B_STEP_PIN_PIN);
	delay_200ns();
	HAL_GPIO_TogglePin(MOTOR_A_STEP_PIN_PORT,MOTOR_A_STEP_PIN_PIN);
	HAL_GPIO_TogglePin(MOTOR_B_STEP_PIN_PORT,MOTOR_B_STEP_PIN_PIN);
}

void delay_200ns(){
	__asm__ __volatile__ (
			"nop" "\n\t"
	        "nop" "\n\t"
			"nop" "\n\t"
	        "nop" "\n\t"
			"nop" "\n\t"
			"nop" "\n\t"
	        "nop" "\n\t"
			"nop" "\n\t"
	        "nop" "\n\t"
			"nop" "\n\t"
			"nop" "\n\t"
	        "nop" "\n\t"
			"nop" "\n\t"
	        "nop" "\n\t"
			"nop" "\n\t"
	        "nop" "\n\t"
	        "nop" "\n\t"
			"nop");

}


