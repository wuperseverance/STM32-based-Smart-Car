/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include <math.h>
#include <stdio.h>
#include "oled.h"
#include "cube.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//#define M_PI 3.1415

float angle_x=0.0f;
float angle_y=0.0f;
float angle_z=0.0f;

//ADC数据采集暂存
uint16_t Xvalue_measure=0;
uint16_t Yvalue_measure=0;

MPU6050_t mpu6050_value;

//mpu6050,摇杆控制与否？
uint8_t mpu6050_state=0;
uint8_t xykey_state=1;

//mpu6050遥控角度阈值
int16_t angle_threhold=20;

//摇杆遥控角度阈值
int16_t xyposition_threhold_up=3000;//
int16_t xyposition_threhold_down=1000;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void process_Gyro_data(float gx, float gy, float gz) {
   
    float dt = 0.005f; // 采样时间，根据你的循环频率调整（例如200Hz）
    angle_x = (angle_x + gx * dt) ;
    angle_y = (angle_y - gy * dt);
    angle_z += gz * dt; // 偏航角仅积分，会漂移

}

void mpu6050_control(){
	MPU6050_Read_All(&hi2c1,&mpu6050_value);
		
		/*自己解析数据，已经注释掉
		G_X=mpu6050_value.Gyro_X_RAW;
		G_Y=mpu6050_value.Gyro_Y_RAW;
		G_Z=mpu6050_value.Gyro_Z_RAW;
		process_Gyro_data(G_X,G_Y,G_X);
		*/
	
		angle_x=mpu6050_value.KalmanAngleX;
		angle_y=mpu6050_value.KalmanAngleY;
		angle_z=mpu6050_value.Gz;
		
		HAL_Delay(5);
	
}

void OLED_output()
{

    OLED_NewFrame();
		//控制方式显示
		if(mpu6050_state)
		{
			OLED_PrintString(10,5,"mpu6050",&font16x16,OLED_COLOR_NORMAL);
			
			//mpu6050数据
			OLED_PrintString(70,5,"x:",&font16x16,OLED_COLOR_NORMAL);
			OLED_PrintString(70,28,"y:",&font16x16,OLED_COLOR_NORMAL);
			OLED_PrintString(70,50,"z:",&font16x16,OLED_COLOR_NORMAL);

			char x_angle_value[1]={0};
			char y_angle_value[1]={0};
			char z_angle_value[1]={0};
			sprintf(x_angle_value, "%d", (int)angle_x);
			OLED_PrintString(90,5,x_angle_value,&font16x16,OLED_COLOR_NORMAL);
			
			sprintf(y_angle_value, "%d", (int)angle_y);
			OLED_PrintString(90,28,y_angle_value,&font16x16,OLED_COLOR_NORMAL);

			sprintf(z_angle_value, "%d", (int)angle_z);
			OLED_PrintString(90,50,z_angle_value,&font16x16,OLED_COLOR_NORMAL);
			
			//////////////////////////////////////////////////////
			CubeProjection cube = GetCubeLinesDefault(angle_y, -angle_x, angle_z, 28.0f);
        
        
      for (int i = 0; i < 12; i++) 
			{
				OLED_DrawLine(cube.lines[i].start.x-30, cube.lines[i].start.y+10,
                         cube.lines[i].end.x-30, cube.lines[i].end.y+10,
                         OLED_COLOR_NORMAL);
        }
			
		}
		else if(xykey_state)
		{
			OLED_PrintString(10,10,"hw-key",&font16x16,OLED_COLOR_NORMAL);
			
			//摇杆模块数据
			OLED_PrintString(10,30,"X:",&font16x16,OLED_COLOR_NORMAL);
			OLED_PrintString(10,50,"Y:",&font16x16,OLED_COLOR_NORMAL);
			char x_value[1]={0};
			char y_value[1]={0};
			sprintf(x_value, "%d", Xvalue_measure);
			OLED_PrintString(30,30,x_value,&font16x16,OLED_COLOR_NORMAL);
			sprintf(y_value, "%d", Yvalue_measure);
			OLED_PrintString(30,50,y_value,&font16x16,OLED_COLOR_NORMAL);
			OLED_DrawRectangle(70,10,50,50,OLED_COLOR_NORMAL);
			OLED_DrawFilledCircle(120-(25./2047)*Xvalue_measure,60-(25./2047)*Yvalue_measure,2,OLED_COLOR_NORMAL);

		}	

		OLED_ShowFrame();
		
	
}

/* USER CODE END Header_adc_read_value */
void adc_read_value()
{
  
		HAL_ADC_PollForConversion(&hadc1, 100);
		Xvalue_measure=HAL_ADC_GetValue(&hadc1);

		HAL_ADC_PollForConversion(&hadc2, 100);
		Yvalue_measure=HAL_ADC_GetValue(&hadc2);
}

void send_control_data()
{
	int8_t front_message[8]={0xff,0xaa,0x02,0x01,0x01,(0x02&0x01&0x01),0xfe,0xef};
	int8_t left_message[8]={0xff,0xaa,0x02,0x01,0x04,(0x02&0x01&0x04),0xfe,0xef};
	int8_t stop_message[8]={0xff,0xaa,0x02,0x01,0x03,(0x02&0x01&0x03),0xfe,0xef};
	int8_t right_message[8]={0xff,0xaa,0x02,0x01,0x02,(0x02&0x01&0x02),0xfe,0xef};
	int8_t back_message[8]={0xff,0xaa,0x02,0x01,0x05,(0x02&0x01&0x05),0xfe,0xef};
	/*
	int8_t left_front_message[8]={0xff,0xaa,0x02,0x01,0x01,(0x02&0x01&0x01),0xfe,0xef};
	int8_t right_front_message[8]={0xff,0xaa,0x02,0x01,0x04,(0x02&0x01&0x04),0xfe,0xef};
	int8_t left_back_message[8]={0xff,0xaa,0x02,0x01,0x02,(0x02&0x01&0x02),0xfe,0xef};
	int8_t right_back_message[8]={0xff,0xaa,0x02,0x01,0x05,(0x02&0x01&0x05),0xfe,0xef};
	*/
	
	//使用mpu6050控制
	if(mpu6050_state)
	{
		//usart内部写的usart1，如果更改请同步改动usart.c最底部,启用重定向记得勾选microlib
		//printf("%f,%f,%f\n",angle_x,angle_y,angle_z);

		if(fabs(angle_y)>=angle_threhold && angle_y>0 && fabs(angle_x)<=angle_threhold)
			HAL_UART_Transmit(&huart1,(uint8_t *)front_message, sizeof(front_message), HAL_MAX_DELAY); 

		else if(fabs(angle_x)>=angle_threhold && angle_x<0 && fabs(angle_y)<=angle_threhold)
			HAL_UART_Transmit(&huart1,(uint8_t *)left_message, sizeof(front_message), HAL_MAX_DELAY); 
		
		else if(fabs(angle_x)>=angle_threhold && angle_x>0 && fabs(angle_y)<=angle_threhold)
			HAL_UART_Transmit(&huart1,(uint8_t *)right_message, sizeof(front_message), HAL_MAX_DELAY); 
		
		else if(fabs(angle_y)>=angle_threhold && angle_y<0 && fabs(angle_x)<=angle_threhold)
			HAL_UART_Transmit(&huart1,(uint8_t *)back_message, sizeof(front_message), HAL_MAX_DELAY); 
		/*
		//左上
		else if(fabs(angle_y)>=angle_threhold && angle_y>0 && fabs(angle_x)>=angle_threhold && angle_x<0)
			HAL_UART_Transmit(&huart1,(uint8_t *)left_front_message, sizeof(front_message), HAL_MAX_DELAY); 

		//右上
		else if(fabs(angle_y)>=angle_threhold && angle_y>0 && fabs(angle_x)>=angle_threhold && angle_x>0)
			HAL_UART_Transmit(&huart1,(uint8_t *)right_front_message, sizeof(front_message), HAL_MAX_DELAY); 
		
		//左下
		else if(fabs(angle_x)>=angle_threhold && angle_x<0 && fabs(angle_y)>=angle_threhold && angle_y<0)
			HAL_UART_Transmit(&huart1,(uint8_t *)left_back_message, sizeof(front_message), HAL_MAX_DELAY); 
		
		//右下
		else if(fabs(angle_x)>=angle_threhold && angle_x>0 && fabs(angle_y)>=angle_threhold && angle_y<0)
			HAL_UART_Transmit(&huart1,(uint8_t *)right_back_message, sizeof(front_message), HAL_MAX_DELAY);
		*/
		else
			HAL_UART_Transmit(&huart1,(uint8_t *)stop_message, sizeof(front_message), HAL_MAX_DELAY); 

		HAL_GPIO_WritePin(led1_GPIO_Port,led1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(led2_GPIO_Port,led2_Pin,GPIO_PIN_RESET);
		HAL_Delay(10);
	}
	//使用摇杆模块控制
	else if(xykey_state)
	{
			//printf("%d,%d\n",Xvalue_measure,Yvalue_measure);
			if(Yvalue_measure>xyposition_threhold_up && Xvalue_measure>xyposition_threhold_down && Xvalue_measure<xyposition_threhold_up)
				HAL_UART_Transmit(&huart1,(uint8_t *)front_message, sizeof(front_message), HAL_MAX_DELAY); 

			else if(Xvalue_measure>xyposition_threhold_up && Yvalue_measure>xyposition_threhold_down && Yvalue_measure<xyposition_threhold_up)
				HAL_UART_Transmit(&huart1,(uint8_t *)left_message, sizeof(front_message), HAL_MAX_DELAY); 

			else if(Xvalue_measure<xyposition_threhold_down && Yvalue_measure>xyposition_threhold_down && Yvalue_measure<xyposition_threhold_up)
				HAL_UART_Transmit(&huart1,(uint8_t *)right_message, sizeof(front_message), HAL_MAX_DELAY); 

			else if(Yvalue_measure<xyposition_threhold_down && Xvalue_measure>xyposition_threhold_down && Xvalue_measure<xyposition_threhold_up)
				HAL_UART_Transmit(&huart1,(uint8_t *)back_message, sizeof(front_message), HAL_MAX_DELAY); 

			/*
			//左上
			else if(Yvalue_measure>xyposition_threhold_up && Xvalue_measure>xyposition_threhold_up)
				HAL_UART_Transmit(&huart1,(uint8_t *)left_front_message, sizeof(front_message), HAL_MAX_DELAY); 

			//右上
			else if(Yvalue_measure>xyposition_threhold_up && Xvalue_measure<xyposition_threhold_down)
				HAL_UART_Transmit(&huart1,(uint8_t *)right_front_message, sizeof(front_message), HAL_MAX_DELAY); 

			//左下
			else if(Xvalue_measure>xyposition_threhold_up && Yvalue_measure<xyposition_threhold_down)
				HAL_UART_Transmit(&huart1,(uint8_t *)left_back_message, sizeof(front_message), HAL_MAX_DELAY); 

			//右下
			else if(Yvalue_measure<xyposition_threhold_down && Xvalue_measure<xyposition_threhold_d
				HAL_UART_Transmit(&huart1,(uint8_t *)right_back_message, sizeof(front_message), HAL_MAX_DELAY);

			*/
			else
				HAL_UART_Transmit(&huart1,(uint8_t *)stop_message, sizeof(front_message), HAL_MAX_DELAY); 
			
			HAL_GPIO_WritePin(led1_GPIO_Port,led1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(led2_GPIO_Port,led2_Pin,GPIO_PIN_SET);
			HAL_Delay(10);
	}
		
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	

	MPU6050_Init(&hi2c1);
	
	float G_X=0,G_Y=0,G_Z=0;
	
	HAL_Delay(20);
	OLED_Init();
	
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);
	
	//否则会卡死
	mpu6050_control();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	


  while (1)
  {
		//按下按键即可使用 mpu6050/摇杆模块  遥控小车
		mpu6050_control();
		adc_read_value();

		if(HAL_GPIO_ReadPin(mpu6050_start_GPIO_Port,mpu6050_start_Pin)==GPIO_PIN_RESET)
		{
			HAL_Delay(10);
			while(HAL_GPIO_ReadPin(mpu6050_start_GPIO_Port,mpu6050_start_Pin)==GPIO_PIN_RESET);
			mpu6050_state=~mpu6050_state;
			xykey_state=~mpu6050_state;
		}
		send_control_data();

				
		//oled显示
		OLED_output();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
