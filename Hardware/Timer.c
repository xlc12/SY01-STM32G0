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


#include "app_user.h"
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
  //  Serial_Printf("Timer\r\n");

  }


  if (htim->Instance == TIM3)

  {

    StepMotor_TimerIRQHandler();

  }




}
