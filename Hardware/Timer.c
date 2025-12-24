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
//  internal clock  64-1   10000-1  记得使能定时器



uint8_t Timer_Count; //定时器测试变量
uint8_t key_Flag = 0;   // 串口发送锁定标志（0=可发送 1=已发送）
uint8_t key_Release_Flag = 0;           // 按键松手发送标志（0=未触发 1=触发）

// 按键阈值定义（10ms中断一次）
#define KEY_DEBOUNCE_TIME    2        // 防抖时间(10xms)
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
	 qmc5883l_filter();// 滤波函数调用
	  
    // 中断内静态变量（仅保留核心状态，无冗余）
    static uint8_t debounce_cnt = 0;    // 消抖计数（20ms确认按下）
    static uint8_t long_press_cnt = 0;  // 长按计时
    static uint8_t double_click_cnt = 0;// 双击计数
    static uint8_t double_interval_cnt = 0; // 双击间隔计时
    static uint8_t continuous_click_cnt = 0; // 连续点击计数
    static uint8_t idle_cnt = 0;        // 无操作计时
    static uint8_t key_state = 0;       // 按键状态：0=无 1=按下 2=长按

    // 按键按下检测（GPIOB_PIN8 低电平有效）
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET)
    {
      idle_cnt = 0; // 有按键动作，重置无操作计时
      debounce_cnt++;

      // 消抖确认按键按下
      if (debounce_cnt >= KEY_DEBOUNCE_TIME) //到达消抖时间点
      {
        key_state = 1;
        debounce_cnt = 2; // 防止溢出
      }

      // 长按判定
      long_press_cnt++;
      if (long_press_cnt >= LONG_PRESS_THRESHOLD)
      {
        KeyNum = 2;          // 标记长按事件
        key_state = 2;       // 标记长按状态
        // 清空其他计数，避免干扰
        continuous_click_cnt = 0;
        double_click_cnt = 0;
        double_interval_cnt = 0;
      }
    }
    // 按键松开/未按下
    else
    {
      // 重置长按相关状态
      long_press_cnt = 0;
      idle_cnt++; // 递增无操作计时

      // 处理短按/双击/连续点击逻辑（未触发长按）
      if (key_state == 1)
      {
        continuous_click_cnt++; // 统计连续点击次数
        key_tick = continuous_click_cnt; // 更新按键次数统计

        // 双击判定
        double_click_cnt++;
        if (double_click_cnt == 1)
        {
          double_interval_cnt = 0; // 第一次点击重置间隔计时
        }
        else if (double_click_cnt == 2 && double_interval_cnt < DOUBLE_CLICK_INTERVAL)
        {
          KeyNum = 3; // 标记双击事件
          double_click_cnt = 0; // 重置双击计数
        }

        // 连续≥5次点击判定
        if (continuous_click_cnt >= CONTINUOUS_CLICK_THRESH)
        {
		  key_Flag = 0;
		  key_Release_Flag = 1;
          KeyNum = 4; // 标记连续点击事件
        }
        // 单次短按判定（双击间隔超时）
        else if (continuous_click_cnt == 1 && double_interval_cnt >= DOUBLE_CLICK_INTERVAL)
        {
		  key_Flag = 0;
		  key_Release_Flag = 1;
          KeyNum = 1; // 标记短按事件
        }

        // 重置按键状态
        key_state = 0;
        debounce_cnt = 0;
      }
      // 长按松手后重置状态
      else if (key_state == 2)
      {
        key_state = 0;
        debounce_cnt = 0;
      }

      // 双击间隔超时，判定为单次短按
      if (double_click_cnt == 1)
      {
        double_interval_cnt++;
        if (double_interval_cnt >= DOUBLE_CLICK_INTERVAL)
        {
		  key_Flag = 0;
		  key_Release_Flag = 1;
          KeyNum = 1; // 标记短按事件
          double_click_cnt = 0;
          double_interval_cnt = 0;
        }
      }

      // 1秒无操作，清零所有状态
      if (idle_cnt >= KEY_IDLE_TIMEOUT)
      {
        KeyNum = 0;
        key_tick = 0;
        idle_cnt = 0;
        continuous_click_cnt = 0;
        double_click_cnt = 0;
        double_interval_cnt = 0;
        key_state = 0;
        debounce_cnt = 0;
        long_press_cnt = 0;
      }
    }
  }
}

uint8_t KEY_GetMultiClickCount(void)  //获取多击时点击次数
{
    return key_tick;
}

/*
参数：延时时间，单位ms

参数说明：delay_ms = 0时直接关机；delay_ms ！= 0时，延时delay_ms ms后再关机
返回：无

接口具体实现： 将 IO（PA12）拉低即关机
*/
void SYSTEM_PowerOff(int delay_ms) //长按两秒关机
{
	//执行长按延迟关机程序
    if(KeyNum == 2)
    {
	   // 组装数据包（5字节：包头0X55 0X52 + 功能码0XA4 + 关机 + 包尾0X2B）
		uint8_t data_State4[5] = {0X55, 0X52, 0XA4, 0X01, 0X2B};//关机指令发送上位机       
      // 发送数据包（仅发送一次）
      Serial_SendArray(data_State4, 5);         
      HAL_Delay(delay_ms);
      // 立即执行关机：拉低PA12
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

    }
}

/*
参数：延时时间，单位ms

参数说明：delay_ms = 0时直接开机；delay_ms ！= 0时，延时delay_ms ms后再开机

返回：无

接口具体实现： 将 IO（PA12）拉高
*/
void SYSTEM_PowerOn(int delay_ms) //长按两秒开机
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);//PA12拉低
	HAL_Delay(delay_ms);//长按2s后自动识别开机
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); //PA12拉低--开机
}

