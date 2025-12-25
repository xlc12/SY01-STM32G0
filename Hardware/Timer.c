#include "main.h"
#include "QMC5883P.h"
#include "qmc_5883p_data.h"
#include <string.h>
//#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>      // 用于atan2()函数
#include <stdbool.h>   // 包含标准布尔类型定义
#include <stdarg.h>
#include "Timer.h"            
#include "My_USART.h"

#include "key_manage.h"
//  internal clock  64-1   10000-1  记得使能定时器

char buf[32];  // 在文件开头或函数内声明


uint8_t Timer_Count; //定时器测试变量
uint8_t key_Flag = 0;   // 串口发送锁定标志（0=可发送 1=已发送）
uint8_t key_Release_Flag = 0;           // 按键松手发送标志（0=未触发 1=触发）

// 按键阈值定义（10ms中断一次）
     // 防抖时间(10xms)
#define LONG_PRESS_THRESHOLD     200    // 长按2秒阈值
#define DOUBLE_CLICK_INTERVAL    40     // 双击400ms间隔阈值
#define CONTINUOUS_CLICK_THRESH  5      // 连续点击5次阈值
#define KEY_IDLE_TIMEOUT         100    // 1秒无操作清零阈值

//核心变量
uint8_t KeyNum = 0;                     // 按键事件：0=无 1=短按 2=长按 3=双击 4=连续≥5次
uint8_t key_tick = 0;                   // 按键按下次数统计，1秒无操作置零





void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {

  /******* 处理磁力计滤波 -begin *******/ 

	 qmc5883l_filter();// 滤波函数调用

   /******* 处理磁力计滤波 -end *******/ 

  
   /******* 按键扫描 -begin *******/ 
   Key_Scan_All(); 
   /******* 按键扫描 -end *******/
   

    

  }
}

// uint8_t KEY_GetMultiClickCount(void)  //获取多击时点击次数
// {
//     return key_tick;
// }

/*
参数：延时时间，单位ms

参数说明：delay_ms = 0时直接关机；delay_ms ！= 0时，延时delay_ms ms后再关机
返回：无

接口具体实现： 将 IO（PA12）拉低即关机
*/
// void SYSTEM_PowerOff(int delay_ms) //长按两秒关机
// {
// 	//执行长按延迟关机程序
//     if(KeyNum == 2)
//     {
// 	   // 组装数据包（5字节：包头0X55 0X52 + 功能码0XA4 + 关机 + 包尾0X2B）
// 		uint8_t data_State4[5] = {0X55, 0X52, 0XA4, 0X01, 0X2B};//关机指令发送上位机       
//       // 发送数据包（仅发送一次）
//       Serial_SendArray(data_State4, 5);         
//       HAL_Delay(delay_ms);
//       // 立即执行关机：拉低PA12
//       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

//     }
// }

/*
参数：延时时间，单位ms

参数说明：delay_ms = 0时直接开机；delay_ms ！= 0时，延时delay_ms ms后再开机

返回：无

接口具体实现： 将 IO（PA12）拉高
// */
// void SYSTEM_PowerOn(int delay_ms) //长按两秒开机
// {
// 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);//PA12拉低
// 	HAL_Delay(delay_ms);//长按2s后自动识别开机
//     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); //PA12拉低--开机
// }

