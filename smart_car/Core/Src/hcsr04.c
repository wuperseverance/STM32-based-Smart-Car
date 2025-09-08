#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

int success_hcsr04=0;

void hcsr04_start()
{
	__HAL_TIM_SET_COUNTER(&htim3,0);
	
	__HAL_TIM_CLEAR_FLAG(&htim3,TIM_FLAG_CC1);
	__HAL_TIM_CLEAR_FLAG(&htim3,TIM_FLAG_CC2);
	
	HAL_TIM_IC_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim3,TIM_CHANNEL_2);
}


void hcsr04_stop()
{
	HAL_TIM_IC_Stop(&htim3,TIM_CHANNEL_1);
	HAL_TIM_IC_Stop(&htim3,TIM_CHANNEL_2);
}

float hcsr04_measure()
{
	HAL_GPIO_WritePin(trig_GPIO_Port,trig_Pin,GPIO_PIN_SET);
	for(int i=90;i>0;i--);
	HAL_GPIO_WritePin(trig_GPIO_Port,trig_Pin,GPIO_PIN_RESET);
	
	uint32_t max_time=HAL_GetTick()+50;
	while(HAL_GetTick()<max_time)
	{
		uint8_t cc1_flag=__HAL_TIM_GET_FLAG(&htim3,TIM_FLAG_CC1);
		uint8_t cc2_flag=__HAL_TIM_GET_FLAG(&htim3,TIM_FLAG_CC2);
		if(cc1_flag && cc2_flag)
		{
			success_hcsr04=1;
			break;
		}
	}
	hcsr04_stop();
	if(success_hcsr04==1)
	{
		success_hcsr04=0;
		
		double distance_=170.0f*(__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_2)-__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_1))*1e-6f;
		return distance_;
		
	}
	/*else
		return -1;
	*/
	
}