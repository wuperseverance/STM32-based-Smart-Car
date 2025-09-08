#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

uint16_t arr_sg90;
uint8_t crr_sg90;

void sg90_init()
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
}


void set_sg90_degree(uint8_t degree_sg90)
{
	degree_sg90+=90;
	degree_sg90%=180;
	
	arr_sg90=__HAL_TIM_GET_AUTORELOAD(&htim1);
	crr_sg90=200*degree_sg90/180+50;
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,crr_sg90);
	
}