/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "motor.h"
#include "zigbee_edc24.h"
#include "algorithm.h"
#include "jy62.h"
#include "pid.h"
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM1)
	{
		int32_t cnt_x  = __HAL_TIM_GET_COUNTER(&htim3);
		cnt_x = (int16_t)cnt_x;
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		motor_speed_x = (float)cnt_x * 1000 / circle * 2 * PI * radius;

		int32_t cnt_y  = __HAL_TIM_GET_COUNTER(&htim2);
		cnt_y = (int16_t)cnt_y;
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		motor_speed_y = (float)cnt_y * 1000 / circle * 2 * PI * radius;

		if(receive_flag)
		{
			reqGameInfo();
			zigbeeMessageRecord();

		}

	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if(huart==&huart3)
	{

	  jy62MessageRecord();
//	  u1_printf("ROW: %f, PITCH:%f, YAW:%f	\n", GetRoll(), GetPitch(), GetYaw());

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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_GPIO_WritePin(reset_GPIO_Port, reset_Pin, GPIO_PIN_SET);
  zigbee_Init(&huart2);
  jy62_Init(&huart3);
  u1_printf("hello\n");
  PID_Init_S(&pid_x, p_ex_set, p_set, i_set, d_set, straight_x);
  PID_Init_S(&pid_y, p_ex_set, p_set, i_set, d_set, straight_y);
  PID_Init_S(&pid_rotate, rp_ex_set, rp_set, ri_set, rd_set, rotate);
  MOTOR_Standby();
  SetBaud(115200);
  SetHorizontal();
  InitAngle();
  Calibrate();
  SleepOrAwake();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	u1_printf("YAW:%f	", GetYaw());
	if(getGameStage()==Prematch){
		orderInit();
		MOTOR_Standby();
	}

	if (HAL_GPIO_ReadPin(reset_GPIO_Port, reset_Pin) == GPIO_PIN_RESET){
		orderInit();
		InitAngle();
		MOTOR_Standby();
	}
	else{
		if(getGameStatus() == GameStandby)
		{
			MOTOR_Standby();
		}
		else if (GetYaw() > 5 && GetYaw() < 180){
			MOTOR_Rotate(negative);
		}
		else if (GetYaw() < 355 && GetYaw() > 180){
			MOTOR_Rotate(positive);
		}
		else
		{
			if(getGameStage()==FirstHalf&&!pile[1])
				set_pile();
			if(pile[1])
				setChargingPile();
			store_order();
//			u1_printf("\t%d", (int)getOwnChargingPileNum());
//			printf("ROW: %f, PITCH:%f, YAW:%f	", GetRoll(), GetPitch(), GetYaw());
			if(next_point.x == 0 && next_point.y == 0)
				next_point=get_nearest_point();
			u1_printf("\tnext_point:(%d,%d)",next_point.x,next_point.y);
			if(next_point.x!=0||next_point.y!=0)
				get_path(next_point);
//			for(int trans = 1; trans <= cnt; trans++)
//			{
//				u1_printf("(%d,%d)--", path[trans].x, path[trans].y);
//			}
			if (getGameStage() == SecondHalf){
				Position_edc24 tmp=check_power();
				if(tmp.x!= 0||tmp.y!= 0)
				{
					next_point=tmp;
					get_path(next_point);
					charge=true;
				}
			}

			now=getVehiclePos();
			int trans;
			for(trans=1;trans<=cnt;++trans)
				if((now.x-path[trans].x)*(now.x-path[trans].x)+(now.y-path[trans].y)*(now.y-path[trans].y)>64)
					break;
			MOTOR_Move(path[trans]);
			u1_printf("path:(%d, %d)  ", path[trans].x, path[trans].y);
//			u1_printf("No.%d:(%d,%d)\n", cnt_run + 1,  next_point.x, next_point.y);
//			u1_printf("cnt_run:%d cnt:%d", cnt_run, cnt);
			u1_printf("\tnow:(%d,%d)\n", getVehiclePos().x, getVehiclePos().y);
			now=getVehiclePos();
			if((now.x-next_point.x)*(now.x-next_point.x)+(now.y-next_point.y)*(now.y-next_point.y)<=64)
			{
				next_point=pos_pair(0,0);
				if(!charge)
				{
					if(order_status[order_id]==waiting)
						order_status[order_id]=loading;
					else if(order_status[order_id]==loading)
						order_status[order_id]=finishing;
				}
				else
					charge=false;
			}
		}
	}
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
