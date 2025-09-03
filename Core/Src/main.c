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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sg90.h"
#include "hcsr04.h"
#include "oled.h"
#include "font.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    FREE,       
    TURN_ON,    
    TURN_OFF,
    FRONT,
    BACK,
    RIGHT,
    LEFT,
    STOP,
    AUTO_WALK,
    SEEK_LINE,
		Update_speed,
		Update_distance,
		Update_angle
} CarState;

typedef enum{
	Front_No_barrier,
	Front_Have_barrier,
	Left_No_barrier,
	Left_Have_barrier,
	Right_No_barrier,
	Right_Have_barrier,
	all_Have_barrier
}Barrier_state;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t receivedata[8];
uint8_t auto_state=0;
uint8_t turn_state=0;//按下左右转弯,后退就置1，防止测距小于阈值，循环检测左右转，后退都失效
CarState now_state=FREE;
Barrier_state now_barrier_state=Front_No_barrier;

uint8_t speed=70;
uint8_t distance=20;//cm
uint8_t angle=30;
float one_round_time=2.8;//转一圈时间，现在是speed=70
double measure_distance=1;//cm

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
CarState check_state(uint8_t receivedata[8]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Front(int speed){
		HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1, speed);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2, speed);
}

void Left(int speed){
	//turn_state=1;
	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1, speed);
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2, speed);
}

void Right(int speed){
	//turn_state=1;
	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_SET);
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1, speed);
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2, speed);
}

void Stop(){
	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2, 0);
}

void Back(int speed){
	//turn_state=1;
	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_SET);
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,speed);
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2, speed);
	
}

void Turn_On(){
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_GPIO_WritePin(led_GPIO_Port,led_Pin,GPIO_PIN_SET);
}

void Turn_Off(){
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);
	HAL_GPIO_WritePin(led_GPIO_Port,led_Pin,GPIO_PIN_RESET);
}


//自主行走
void Auto_Walk(){
		//待实现
}

//巡线功能
void Seek_Line(){
		//待实现
}


void OLED_display_update(){
	OLED_NewFrame();
	OLED_PrintString(10,10,"distance:",&font16x16,OLED_COLOR_NORMAL);
	// 同样先转换为字符串
	char numStr[5];
	sprintf(numStr, "%d", (int)measure_distance);
	OLED_PrintString(90,10,numStr,&font16x16,OLED_COLOR_NORMAL);
	
	
	switch(now_barrier_state){
		case Front_Have_barrier:{
			OLED_PrintString(10,25,"F_Have",&font16x16,OLED_COLOR_NORMAL);
			break;
		}
		case Front_No_barrier:{
			OLED_PrintString(10,25,"F_No",&font16x16,OLED_COLOR_NORMAL);
			break;
		}
		case Left_Have_barrier:{
			OLED_PrintString(10,25,"L_Have",&font16x16,OLED_COLOR_NORMAL);
			break;
		}
		case Left_No_barrier:{
			OLED_PrintString(10,25,"L_No",&font16x16,OLED_COLOR_NORMAL);
			break;
		}
		case Right_Have_barrier:{
			OLED_PrintString(10,25,"R_Have",&font16x16,OLED_COLOR_NORMAL);
			break;
		}
		case Right_No_barrier:{
			OLED_PrintString(10,25,"R_No",&font16x16,OLED_COLOR_NORMAL);
			break;
		}
		case all_Have_barrier:{
			OLED_PrintString(10,25,"A_Have",&font16x16,OLED_COLOR_NORMAL);
			break;
		}
	}
	
	char key_state[1];
	sprintf(key_state, "%d", (int)auto_state);
	OLED_PrintString(10,45,"key:",&font16x16,OLED_COLOR_NORMAL);
	OLED_PrintString(50,45,key_state,&font16x16,OLED_COLOR_NORMAL);
	
	
	OLED_ShowFrame();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    static uint8_t buffer[8];  // 临时缓冲区
    static uint8_t index = 0;  // 当前写入位置

    // 将新数据存入缓冲区
    buffer[index] = receivedata[0];
    index = (index + 1) % 8;  // 循环写入，防止溢出

    // 检查当前缓冲区是否对齐帧头（0xFF 0xAA）
    for (int i = 0; i < 8; i++) {
        if (buffer[(index + i) % 8] == 0xFF && 
            buffer[(index + i + 1) % 8] == 0xAA) {
            // 找到帧头，复制对齐后的数据到 receivedata
            for (int j = 0; j < 8; j++) {
                receivedata[j] = buffer[(index + i + j) % 8];
            }
            
            // 处理数据并回传
            now_state = check_state(receivedata);
            HAL_UART_Transmit_IT(&huart1, receivedata, 8);
            break;
        }
    }

    // 继续接收下一字节
    HAL_UART_Receive_IT(&huart1, &receivedata[0], 1);
}
CarState check_state(uint8_t receivedata[8])
{
    // 检查数据头尾是否符合协议规范
    if((receivedata[0] != 0xff) || (receivedata[1] != 0xaa) || 
       (receivedata[6] != 0xfe) || (receivedata[7] != 0xef))
    {
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2, 0);
        return FREE;  // 协议不匹配，返回空闲状态
    }

    uint8_t command_type = receivedata[2];  // 命令类型
    uint8_t data_length = receivedata[3];   // 数据长度
    
    // 检查数据长度是否有效
    if(data_length > 4 || (4 + data_length) > 8)
    {
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2, 0);
        return FREE;  // 数据长度无效
    }

    switch(command_type)
    {
        case 0x01:  // 电源控制命令
            for(int i = 0; i < data_length; i++)
            {
                switch(receivedata[4 + i])
                {
                    case 0x01: {
											Turn_On();
											return TURN_ON;
										}
                    case 0x02: {
											Turn_Off();
											return TURN_OFF;
										}
                    default: break;
                }
            }
            break;
            
        case 0x02:  // 运动控制命令
            for(int i = 0; i < data_length; i++)
            {
                switch(receivedata[4 + i])
                {
                    case 0x01: {//直走
											Front(speed);
											return FRONT;
										}
                    case 0x02: {//
											Right(speed);
											return RIGHT;
										}
                    case 0x03: {
											Stop();
											return STOP;
										}
                    case 0x04: {
											Left(speed);
											return LEFT;
										}
                    case 0x05: {
											Back(speed);
											return BACK;
										}
                    default: break;
                }
            }
            break;
            
        case 0x03:  // 模式控制命令
            for(int i = 0; i < data_length; i++)
            {
                switch(receivedata[4 + i])
                {
                    case 0x01:{ 
											Auto_Walk();
											return AUTO_WALK;
										}
                    case 0x02:{ 
											Seek_Line();
											return SEEK_LINE;
										}
                    default: break;
                }
            }
            break;
            
        case 0x04:  // 速度更新命令
            if(data_length >= 1 )
            {
							speed=receivedata[4];
                return Update_speed;
            }
            break;
            
        case 0x05:  // 距离更新命令
            if(data_length >= 1 )
            {
							distance=receivedata[4];
                return Update_distance;
            }
            break;
            
        case 0x06:  // 角度更新命令
            if(data_length >= 1)
            {
							angle=receivedata[4];
                return Update_angle;
            }
            break;
            
        default:
            break;
    }
    
    return FREE;  // 默认返回空闲状态
}


//前方障碍物状态，未完全实现
/*
Barrier_state barrier_state(){
	switch(now_barrier_state){
		case Front_Have_barrier:{
			Left(70);
			HAL_Delay((int)one_round_time*1000/360*angle);//2.8是转一圈的时间，按照speed=70测算，如若变化还需重测
			Turn_Off();
			if(measure_distance<distance)
				now_barrier_state=Left_Have_barrier;
			else
				now_barrier_state=Left_No_barrier;
		}
			return now_barrier_state;
		
		case Front_No_barrier:{
			Front(speed);
			return now_barrier_state;
		}
		
		case Left_Have_barrier:{
			Right(70);
			HAL_Delay((int)2*one_round_time*1000/360*angle);//已经是左转过的，到右转状态需要先恢复，再转，所以乘2
			Turn_Off();
			if(measure_distance<distance)
				now_barrier_state=Right_Have_barrier;
			else
				now_barrier_state=Right_No_barrier;
		}
			return now_barrier_state;
		
		case Left_No_barrier:{
			now_barrier_state=Front_No_barrier;
			return now_barrier_state;
		}
		
		case Right_Have_barrier:{
			now_barrier_state=all_Have_barrier;
			return now_barrier_state;
		}
		
		case Right_No_barrier:{
			now_barrier_state=Front_No_barrier;
			return now_barrier_state;
		}
		
		case all_Have_barrier:{
			angle+=15;//多转15°
			now_barrier_state=Front_Have_barrier;
			return now_barrier_state;
		}
		
	}
}

*/

//小车雷达测距状态更新
/*
void car_visual(){
	hcsr04_start();
	measure_distance=hcsr04_measure();
	
	measure_distance*=100;//转化cm单位
	
	if(measure_distance==0){//无效
		measure_distance=999.0;
	}
	
	OLED_display_update();
	
	//开启自动行驶才会左右转更新，否则只是测距
	if(auto_state){
		now_barrier_state=barrier_state();
	}
}
*/

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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_Delay(20);
	OLED_Init();
	
	/*装上舵机考虑取消注释
	sg90_init();
	set_sg90_degree(0);
	*/
	
	HAL_UART_Receive_IT(&huart1,receivedata,8);	
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//自动行驶
		if(HAL_GPIO_ReadPin(key_GPIO_Port,key_Pin)==GPIO_PIN_RESET)
		{
			HAL_Delay(10);
			while(HAL_GPIO_ReadPin(key_GPIO_Port,key_Pin)==GPIO_PIN_RESET);
			HAL_Delay(10);
			auto_state=~auto_state;
			if(auto_state)
				Turn_On();
			else
				Turn_Off();
		}
		
		//car_visual();
		
		
		//距离小于阈值或者测量失败停车
		/*
		if((measure_distance<distance)&&(turn_state==0)){
			HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);
	}
		*/
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
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
