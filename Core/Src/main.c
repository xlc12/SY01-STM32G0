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

#include "motor_manage.h"

#include "compass_manage.h"

#include "usart_manage.h"

#include "my_flash.h"


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



/*********** app_user 应用层 ***************/

uint8_t isPowerOff_flag = 0; //关机标志位
HouseRotateStruct house_rotate = {0};
//校准标志位
uint8_t isCalibration_flag = 0;

//初始化完成标志位
uint8_t isInit_flag = 0;

//转回起始位标志
uint8_t isBackInit_flag = 0;
int32_t Calibration_Offset = 0; //校准偏移量

//磁力计方位标定标志位
uint8_t isCompassCalibration_dir_flag = 0;

//初始位置
int32_t INITIAL_ANGLE_Flash = INITIAL_ANGLE;
//初始位校准标志位
uint16_t isCompassInitAngle_flag = 0;


/************************** 按键监测 应用层：按键事件回调函数 -· **************************/
//按键监听   --*功能需求 5*--
static void Key_Event_Callback(Key_EventTypeDef event, uint8_t click_cnt)
{  
  
    
    //按键事件命令构建
    uint8_t cmd[5] = {USART_CMD_HEAD1, USART_CMD_HEAD2, USART_S_CMD_KEY_COUNT, click_cnt, USART_CMD_TAIL};
    Serial_SendHexCmd(cmd, sizeof(cmd));
    
    
    // Serial_Printf("Key Single Click，event=%d, click_cnt=%d\r\n", event, click_cnt);

    switch (event)
    {
        case KEY_EVENT_SINGLE_CLICK:
            // Serial_Printf("Key Single Click\r\n");
            //回到原位
            MOTOR_RotateToAngle(INITIAL_ANGLE);
            
            break;

        case KEY_EVENT_DOUBLE_CLICK:
            // 双击业务逻辑
            // Serial_Printf("Key Double Click\r\n");
            //停止旋转
            // if(isInit_flag == 1)
            // {
            //   MOTOR_Stop();
            // }

            MOTOR_Stop();
             
            break;

        case KEY_EVENT_MULTI_CLICK:
            // N次点击业务逻辑（click_cnt为实际次数）
            // Serial_Printf("Key Multi Click: %d\r\n", click_cnt);
            //点击三次 business logic
            if(click_cnt == 3)
            {
              //一直旋转
              MOTOR_SetDirection(STEP_MOTOR_FORWARD);
                // Serial_Printf("Key Multi Click: %d\r\n", click_cnt);
            }
            if(click_cnt == 7)
            {
              //校准
              isCalibration_flag = 1;
            }
            break;

        case KEY_EVENT_LONG_PRESS:
            // 长按关机
            Serial_Printf("Key Long Press\r\n");
            isPowerOff_flag = 1;
            
            break;

        default:
            break;
    }
}

/************************** 按键监测 应用层：按键事件回调函数 -end **************************/





/************************** 串口命令处理 应用层：串口命令回调函数 -· **************************/
// --*功能需求 6*--  串口接收命令处理

static void UART_Command_Callback(uint8_t cmd, uint8_t* data, uint16_t len)
{
    // 处理命令
    Uart_CommandHandler(cmd, data, len);
}
/************************** 串口命令处理 应用层：串口命令回调函数 -end **************************/







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

  //执行开机程序  --*功能需求 1*--
    SYSTEM_PowerOn(POWER_ON_TIMER);


    //定时器+串口初始化
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_ORE | UART_FLAG_PE);//在串口初始化时，提前清除一次错误标志
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxBuff, LENGTH);	//使能接收中断
    HAL_TIM_Base_Start_IT(&htim2);//10ms
    HAL_TIM_Base_Start_IT(&htim3);//10ms
    TIM3_SetInterruptTime(1); //设置TIM3为1000ms中断


    Serial_Printf("66666666666main start\r\n");






    // 按键初始化
    Key_HW_Init();
    // 注册按键事件回调函数
    /*业务需求-1、按键事件*/
    Key_Register_Event_Callback(Key_Event_Callback);

    // 步进电机初始化
     
     MOTOR_Init();
    //  MOTOR_SetDirection(STEP_MOTOR_FORWARD);


    // QMC_IIC_Init();
    // QMC5883_Init();
    

    //串口命令回调函数注册
    /*业务需求-2、串口命令接收处理*/  
	  UART_RegisterCallback(UART_Command_Callback);





    // //开机获取flash的数据
    // int32_t Read_QMC_Data[3] = {0,0,0};
    // // 读取寄存器值
    //  FLASH_ReadInt32(FLASH_PAGE_29_ADDRESS_1,&Read_QMC_Data[0]); //上电读取  
    //  FLASH_ReadInt32(FLASH_PAGE_29_ADDRESS_2,&Read_QMC_Data[1]); //上电读取
    //  FLASH_ReadInt32(FLASH_PAGE_29_ADDRESS_3,&Read_QMC_Data[2]); //上电读取
    //  QMC_Offset_X = Read_QMC_Data[0];
    //  QMC_Offset_Y = Read_QMC_Data[1];
    //  QMC_Offset_Z = Read_QMC_Data[2];
    //  //打印QMC_Offset_X,QMC_Offset_Y,QMC_Offset_Z
    //  Serial_Printf("QMC_Offset_X: %d, QMC_Offset_Y: %d, QMC_Offset_Z: %d\r\n", QMC_Offset_X, QMC_Offset_Y, QMC_Offset_Z);



   
    // //从flash中读取初始位角度
    // int32_t Read_InitAngle[1] = {0};
    // FLASH_ReadInt32(FLASH_PAGE_30_ADDRESS_1,&Read_InitAngle[0]);
    // if( (INITIAL_ANGLE_Flash != Read_InitAngle[0])  && (Read_InitAngle[0] < 360) && (Read_InitAngle[0] > 0) )
    // {
    //   INITIAL_ANGLE_Flash = Read_InitAngle[0];
    //   Serial_Printf("INITIAL_ANGLE_Flash: %d\r\n", INITIAL_ANGLE_Flash);
    // }

    // //打印初始位角度
    // Serial_Printf("***************** INITIAL_ANGLE_Flash 22222 ************: %d\r\n", INITIAL_ANGLE_Flash);


     //开机回到初始位  --*功能需求 3*--
    MOTOR_RotateToAngle(INITIAL_ANGLE);



    // //待ADC采集稳定
    // isBatteryVoltageStable(); //有延时操作




  // //电机
  // while (getMOTOR_State() != STEP_MOTOR_STOP)  
  // {
  //   //延时
  //   HAL_Delay(500);
  //   //日志
  //   Serial_Printf("666666666666\r\n");

  //    //长按关机，由于带延时操作，不在中断中执行  --*功能需求 2*--
  //    if(isPowerOff_flag)
  //    {
  //      Serial_Printf("Key Long Press\r\n");
  //      MOTOR_PowerOff();  //执行开关机点击动作，待实现
  //      SYSTEM_PowerOff(POWER_OFF_TIMER);  //系统延时关机
  //    }
    
  // }



  // //从flash中读取标定偏移量
  //   int32_t Read_Calibration_Offset_Data[1] = {0};

  //   FLASH_ReadInt32(FLASH_PAGE_31_ADDRESS_1,&Read_Calibration_Offset_Data[0]);
  //   //从flash中读取标定偏移量
  //   if(Calibration_Offset != Read_Calibration_Offset_Data[0] && Read_Calibration_Offset_Data[0] != 0)
  //   {

  //     Calibration_Offset = Read_Calibration_Offset_Data[0];
  //     //日志
  //     Serial_Printf("  *********** Read_Calibration_Offset_Data *************: %d, %d\r\n", Read_Calibration_Offset_Data[0], Calibration_Offset);
  //   }
  //   Serial_Printf("  *********** Read_Calibration_Offset_Data 22*************: %d, %d\r\n", Read_Calibration_Offset_Data[0], Calibration_Offset);

  // //获取当前方位
  //  for(int i = 0; i < 10; i++)
  //  {
  //    //延时
  //    HAL_Delay(100);
  //    //获取初始位
  //    house_rotate.Current_dir = getCompassDirection();
  //    getCompassAngle();
  //  }
  // //  house_rotate.Current_dir = getCompassDirection();
  
  // Serial_Printf("6666666Initial_angle = %f\r\n", getCompassAngle());//
  // isInit_flag = 1;

    

   
    

  /****** 初始化 -end ******/




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {	

     HAL_Delay(100);

    // /************** 测试用代码 -begin **************/
    // //获取转盘角度
    // float turntableAngle = 0;
    // turntableAngle = getTurntableAdcConvertToAngle();
    
    // #if DEBUG_LOG
    // Serial_Printf("ADC: %d, turntableAngle = %f\r\n", ADC_PB1_ReadRawValue(), turntableAngle);
    // #endif
    // /************** 测试用代码 -end **************/


    /************************************ 事件循环 -begin ********************************/

    //长按关机，由于带延时操作，不在中断中执行  --*功能需求 2*--
    if(isPowerOff_flag)
    {
      Serial_Printf("Key Long Press\r\n");
      MOTOR_PowerOff();  //执行开关机点击动作，待实现
      SYSTEM_PowerOff(POWER_OFF_TIMER);  //系统延时关机
    }

     //串口-设备信息上报  --*功能需求 4*--
    //低电压、充电、充满上报
    DeviceInfo_CycleSend();//周期性监测设备信息





    // /******************** 磁力计校准 -begin *****************/
    // if(isCalibration_flag)
    // {
    //   // //如果不在初始位，则先回到初始位，再进行校准
    //   // if(!isTurntableInInitialPosition())
    //   // {
    //   //   MOTOR_RotateToAngle(INITIAL_ANGLE_Flash);
    //   //   return;
    //   // }
    //   isCalibration_flag = 0;
    //   //日志
    //   Serial_Printf("111111111111111111111111  QMC5883_Calibrate()  11111111111111111111111111111111\r\n");
    //   //延时1秒
    //   HAL_Delay(100);
    //   //串口发送数据
    //   uint8_t cmd[5] = {USART_CMD_HEAD1, USART_CMD_HEAD2, USART_S_CMD_CALIBRATION_ANGLE, 00, USART_CMD_TAIL};
    //   Serial_SendHexCmd(cmd, sizeof(cmd));
    //   //延时1秒
    //   HAL_Delay(1000);
    //   QMC5883_Calibrate();
    // }

    // /***** 磁力计校准 -end ******/




    // /***************** 磁力计方位标定 -begin *****************/
    // //重新标定，更新flash中的Calibration_Offset
    // if(isCompassCalibration_dir_flag == 1)
    // {
 
    //    //保存到flash
    //    FLASH_ErasePage(FLASH_PAGE_31_ADDRESS_1);
    //    HAL_Delay(50);
    //    FLASH_WriteInt32(FLASH_PAGE_31_ADDRESS_1,Calibration_Offset);

    //    HAL_Delay(50);
    //   //判断是否写入成功
    //   int32_t Read_Calibration_Offset_Data[1] = {0};
    //   FLASH_ReadInt32(FLASH_PAGE_31_ADDRESS_1,&Read_Calibration_Offset_Data[0]);
    //   if(Read_Calibration_Offset_Data[0] == Calibration_Offset)
    //   {
    //     Serial_Printf("USART_CMD_CALIBRATION_ANGLE: %d, %d, success\r\n", Calibration_Offset, Read_Calibration_Offset_Data[0]);
        
    //     uint8_t data[5] = {USART_CMD_HEAD1, USART_CMD_HEAD1, USART_CMD_CALIBRATION_DIR, isCompassCalibration_dir_flag, USART_CMD_TAIL};
    //     Serial_SendHexCmd(data, sizeof(data));
    //     isCompassCalibration_dir_flag = 0;

    //   }
    //   else
    //   {
    //     Serial_Printf("USART_CMD_CALIBRATION_ANGLE: %d, %d, fail\r\n", Calibration_Offset, Read_Calibration_Offset_Data[0]);
    //   }
    // }
    
    /***** 磁力计方位标定 -end ******/



    // /***************** 初始角度校准 -begin *****************/
    // if(isCompassInitAngle_flag !=0 && getMOTOR_State() == STEP_MOTOR_STOP)
    // {
    //   //获取当前转盘角度
    //   int32_t Angle = 0;
    //   Angle = (int32_t)(getTurntableAdcConvertToAngle());
      
    //   //将当前角度写入flash
    //   FLASH_ErasePage(FLASH_PAGE_30_ADDRESS_1);
    //   HAL_Delay(100);
    //   FLASH_WriteInt32(FLASH_PAGE_30_ADDRESS_1,Angle);
    //   HAL_Delay(100);
    //   //判断是否写入成功
    //   int32_t Read_InitAngle_Data[1] = {0};
    //   FLASH_ReadInt32(FLASH_PAGE_30_ADDRESS_1,&Read_InitAngle_Data[0]);
    //   if(Read_InitAngle_Data[0] == Angle)
    //   {
    //       Serial_Printf("USART_CMD_INITIAL_ANGLE: %d, %d, success\r\n", Angle, Read_InitAngle_Data[0]);
    //       INITIAL_ANGLE_Flash = Read_InitAngle_Data[0];
          
    //       uint8_t data[5] = {USART_CMD_HEAD1, USART_CMD_HEAD1, USART_CMD_INITIAL_ANGLE, isCompassInitAngle_flag, USART_CMD_TAIL};
    //       Serial_SendHexCmd(data, sizeof(data));
    //       isCompassInitAngle_flag = 0;
    //   }
    //   else
    //   {
    //       Serial_Printf("USART_CMD_INITIAL_ANGLE: %d, %d, fail\r\n", Angle, Read_InitAngle_Data[0]);
    //   }
    // }
    /***** 初始角度校准 -end ******/



    
    
    /***************** 财神位转动 -begin *****************/
    /*业务需求-3、财神位转动*/
    //财神位转动算法 -begin 
    
  //   float angle = getCompassAngle();  

    
  //   //判断是否回到起始位
  //   if (getMOTOR_State() == STEP_MOTOR_STOP && isTurntableInInitialPosition())
  //   {
  //     /* code */
  //     //延时1秒
  //     HAL_Delay(500);
  //     isBackInit_flag = 0; // 回到起始位结束
      
  //     for(int i = 0; i < 10; i++)
  //     { 
  //       //延时50ms
  //       HAL_Delay(50);
  //       //获取当前磁力计方位
  //       house_rotate.Current_dir = getCompassDirection();
  //     }

  //     // Serial_Printf("6666666_isBackInit_flag = %d, Current_angle = %f\r\n", isBackInit_flag, getCompassAngle());//
      
  //     // Serial_Printf("666666666666666666666666");// 
  //   }

    
  //   // house_rotate.Current_dir = 1;
  //   #if DEBUG_LOG
  // //  Serial_Printf("housePoint_dir = %d, angle = %f\r\n", house_rotate.Current_dir, angle);
  //   #endif


    if(house_rotate.Target_dir != 0) //如果目标方向不为0，则执行旋转
    {
      int target_dir = house_rotate.Target_dir;

      house_rotate.Target_dir = 0; //旋转完成后，将目标方向设为0

        //计算目标角度
        float target_angle = (target_dir - 1) * 45 + INITIAL_ANGLE;
        //设置电机转动到目标角度
        if(target_angle < 0)
        {
          target_angle = 360 + target_angle;
        }
        target_angle = 360 - target_angle;  //反向

        // //判断当前角度是否在目标角度附近，如果在，则不转，如果不在则先回到初始位，再转到目标角度
        // if(abs(target_angle - getTurntableAngle()) < 2)
        // {
        //   continue;
        // }
        
        //再转到目标角度
        MOTOR_RotateToAngle(target_angle);
        Serial_Printf("target_angle = %f\r\n", target_angle);
      }

      //财神位转动算法 -end

      /*********** 财神位转动 -end ********/



   



    // HAL_Delay(500);


    /****** 事件循环 -end ******/
    


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



      //while(1);
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