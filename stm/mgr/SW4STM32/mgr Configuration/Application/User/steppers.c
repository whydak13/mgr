# include "steppers.h"

uint32_t calculate_next_step(float acceleration,int8_t stepper_direction)
{
	static uint32_t c=0;
	static uint32_t c_prev=0;
	static float n=1;
	static float n_prev=1;
	static float accleration_prev=0;


	if (c==0) // Init
	{
		c=STEPPER_DRIVER_FREQUENCY*sqrtf((2*MOTOR_STEP_ANGLE)/(acceleration));
		//c=STEPPER_DRIVER_FREQUENCY*powf((2*MOTOR_STEP_ANGLE)/(acceleration),0.5);
		c_prev=c;
		accleration_prev=acceleration;
		return c;
	}
    if (acceleration!=accleration_prev)
    {
        n=(n_prev*accleration_prev)/acceleration;
    }
    else
        n++;
    c=(int32_t)(c_prev - (2*c_prev)/(4*n+1));
    limit_motor_speed(&c, &stepper_direction, &acceleration);
    c_prev=c;
    n_prev=n;
    accleration_prev=acceleration;
    return c;

}

void limit_motor_speed(uint32_t* time_to_next_step, int8_t* direction, float* acceleration)
{
	*time_to_next_step=max(*time_to_next_step,MIN_MOTOR_DELAY);
	if(*time_to_next_step>MAX_MOTOR_DELAY)
	{
		*time_to_next_step=MAX_MOTOR_DELAY;
		if(*acceleration<0)// point of 0 speed reached so its time to start accelerating in opossite direction
		{
			*direction=-(*direction);
			*acceleration=-(*acceleration);
		}
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
