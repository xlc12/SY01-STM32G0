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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "OLED.h"
#include "Timer.h"
#include "My_USART.h"
#include "QMC5883P.h"
#include "qmc_5883p_data.h"
#include "step_motor.h"
#include "my_adc.h"
#include "Control.h"    //核心控制逻辑

#include "app_user.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//宏定义


extern uint8_t Timer_Count; //定时器测试变量
extern uint8_t RxBuff[LENGTH];		//接收缓冲区

extern int16_t qmcdata[3]; //原始磁力计数据
extern int16_t xd, yd, zd; //转换XYZ
extern float MagX, MagY, MagZ, MagH; // 滤波后的数据
extern float Azimuth;         //方位角（单位：度，0°=磁北，顺时针递增）
extern float Target_Azimuth; //校准后的最终角度
extern float Azimuth_off;         //校准角度差值
extern uint8_t dir_code;          // 磁力计当前方向编码（0x01-0x08）

extern uint8_t KeyNum; //按键值
extern uint8_t key_tick;      // 用于按键计数
extern uint8_t key_Flag;  //按键串口发送标志位
extern uint8_t key_Release_Flag; //按键发送一次性标志位

extern uint8_t Tim3_Count; //定时器3测试
extern uint8_t PID_Start;//运行pid控制标志位



/************************** 按键监测 应用层：按键事件回调函数 -· **************************/
static void Key_Event_Callback(Key_EventTypeDef event, uint8_t click_cnt)
{  Serial_Printf("Key Key_Event_Callback\r\n");
    switch (event)
    {
        case KEY_EVENT_SINGLE_CLICK:
            // 单击业务逻辑（示例：LED翻转、串口打印
            // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
            //打印单击
            Serial_Printf("Key Single Click\r\n");
            break;

        case KEY_EVENT_DOUBLE_CLICK:
            // 双击业务逻辑
            Serial_Printf("Key Double Click\r\n");
            break;

        case KEY_EVENT_MULTI_CLICK:
            // N次点击业务逻辑（click_cnt为实际次数）
            Serial_Printf("Key Multi Click: %d\r\n", click_cnt);
            break;

        case KEY_EVENT_LONG_PRESS:
            // 长按业务逻辑
            Serial_Printf("Key Long Press\r\n");
            break;

        default:
            break;
    }
}

/************************** 按键监测 应用层：按键事件回调函数 -end **************************/

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


/****** 参数定义 -begin ******/



/****** 参数定义 -end ******/


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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  

  /****** 初始化 -begin ******/
  SYSTEM_PowerOn(POWER_ON_TIMER);//执行开机程序  


    //定时器+串口初始化
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_ORE | UART_FLAG_PE);//在串口初始化时，提前清除一次错误标志
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxBuff, LENGTH);	//使能接收中断
    HAL_TIM_Base_Start_IT(&htim2);//10ms
    HAL_TIM_Base_Start_IT(&htim3);//10ms
    TIM3_SetInterruptTime(1); //设置TIM3为1000ms中断



    /******** 按键监测  -begin ********/

    // 按键初始化
    Key_HW_Init();

    // 注册按键事件回调函数
    Key_Register_Event_Callback(Key_Event_Callback);

    /******** 按键监测  -end ********/


    

  /****** 初始化 -end ******/



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {	

    /****** 事件循环 -begin ******/
//    Serial_Printf("Key 66666666666\r\n");
//    //延时100ms
    HAL_Delay(3000);


    /****** 事件循环 -end ******/
    


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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

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