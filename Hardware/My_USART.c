#include "My_USART.h"
#include "main.h"
#include "usart.h"
#include "my_adc.h"
//C语言函数库
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>      // 用于atan2()函数
#include <stdbool.h>   // 包含标准布尔类型定义
#include <stdio.h>
#include <stdarg.h>

extern int16_t qmcdata[3]; //原始磁力计数据
extern int16_t xd, yd, zd; //转换XYZ
extern float MagX, MagY, MagZ, MagH; // 滤波后的数据
extern float Azimuth;         //方位角（单位：度，0°=磁北，顺时针递增）
extern float Azimuth_off;         //校准角度差值
float Target_Azimuth; //校准后的最终角度

extern float ADC_Hourse;//马儿角度

extern uint8_t KeyNum; //按键值
extern uint8_t key_tick;      // 用于按键计数
extern uint8_t key_Flag;  //按键串口发送标志位
extern uint8_t key_Release_Flag; //按键发送一次性标志位

/****************************串口*****************************************/

void Serial_SendByte(uint8_t Byte) //发送一个字节
{
  HAL_UART_Transmit(&huart1, &Byte, 1, 100);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length) //发送一个数组
{
  HAL_UART_Transmit(&huart1, Array, Length, 1000);
}

void Serial_SendString(char *String) //发送一个字符串
{
  uint16_t len = 0;

  while (String[len] != '\0') len++;

  HAL_UART_Transmit(&huart1, (uint8_t*)String, len, 1000);
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y) //printf配合使用
{
  uint32_t Result = 1;

  while (Y --)
  {
    Result *= X;
  }

  return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)//发送一个数字
{
  char buffer[10];

  for (int i = Length - 1; i >= 0; i--)
  {
    buffer[i] = (Number % 10) + '0';
    Number /= 10;
  }

  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, Length, 1000);
}

int fputc(int ch, FILE *f) //配合printf使用
{
  HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 100);
  return ch;
}

void Serial_Printf(char *format, ...) //printf函数
{
  char String[100];
  va_list arg;
  va_start(arg, format);
  vsprintf(String, format, arg);
  va_end(arg);

  uint16_t len = 0;

  while (String[len] != '\0') len++;

  HAL_UART_Transmit(&huart1, (uint8_t*)String, len, HAL_MAX_DELAY);
}

//角度归一化函数（确保0~360°）
static float Normalize_Angle(float angle)
{
  angle = fmod(angle, 360.0f);

  if (angle < 0) angle += 360.0f;

  return angle;
}

// 计算浮点数绝对值（基础版）
float my_abs(float a)
{
  return a > 0 ? a : (-a);
}

uint8_t RxBuff[LENGTH];		//接收缓冲区
uint8_t Motor_Direction = 0;   // 存储解析出的电机旋转方位
uint8_t Send_Batter_Flag = 0;   //允许上报电量标志位(百分比)

extern uint8_t PID_Start;//运行pid控制标志位


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if(huart -> Instance == USART1)	// 判断发生接收中断的串口
  {
    //Serial_Printf("\r\n已经接收到不定长数据，数据如下：");
    //HAL_UART_Transmit_IT(&huart1, RxBuff, Size); // 回显发送的数据

    /**************************************数据解析****************************************/
	  
    //先判断接收长度是否满足完整数据包（5个字节：55 52 1A 方位 5B）
    if(Size == 5)
    {
      // 2. 直接判断包头、命令字、包尾
      if(RxBuff[0] == 0x55 && RxBuff[1] == 0x52 && RxBuff[2] == 0x1A && RxBuff[4] == 0x5B)
      {
		  PID_Start = 1;//运行PID控制
        // 解析电机旋转方位（第4个字节，数组下标3）
        Motor_Direction = RxBuff[3];

        //解析成功后的操作
        // HAL_UART_Transmit_IT(&huart1, &Motor_Direction, 1); // 回显方位数据
      }
    }

    // 上位机要求读取电量(百分比)
    if(Size == 4) // 匹配4字节数据包：55 52 A8 5B
    {
      if(RxBuff[0] == 0x55 && RxBuff[1] == 0x52 && RxBuff[2] == 0xA8 && RxBuff[3] == 0x5B)
      {
        Send_Batter_Flag = 1; // 上位机要求读取电量(百分比)
        //HAL_UART_Transmit_IT(&huart1, &Send_Batter_Flag, 1); // 回显标志位
      }
    }
	// 上位机校准磁力计（扩展为6字节：55 52 1B 高字节 低字节 5B）
	if(RxBuff[0] == 0x55 && RxBuff[1] == 0x52 && RxBuff[2] == 0x1B && RxBuff[5] == 0x5B)
	{
		// 拼接2个字节为16位整数（小端模式：低字节在前，高字节在后）
		uint16_t target_val = (uint16_t)RxBuff[4] | ((uint16_t)RxBuff[3] << 8);
		
		Azimuth_off = my_abs(target_val-Azimuth);

		// 回显OK确认
	//    Serial_Printf("OK: %.2f\r\n", Azimuth_off);
	}

    HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxBuff, LENGTH);	//使能接收中断
  }
}

// 【必须新增】串口错误回调函数（解决错误导致的卡死）
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    // 清除所有错误标志
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_ORE | UART_FLAG_PE);
    // 复位状态机
    huart->gState = HAL_UART_STATE_READY;
    huart->RxState = HAL_UART_STATE_READY;
    // 重新使能接收
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxBuff, LENGTH);
  }
}


/***************************串口上报状态***************************/
void Key_State(void) //返回按键次数
{
  if(key_tick !=0 && key_Release_Flag == 1 && key_Flag == 0)
  {
    // 组装数据包（5字节：包头0X55 0X52 + 功能码0XA1 + 按键计数 + 包尾0X2B）
    uint8_t data_State[5] = {0X55, 0X52, 0XA1, key_tick, 0X2B};
    // 发送数据包（仅发送一次）
    Serial_SendArray(data_State, 5);

    key_Flag = 1; // 锁定发送，避免重复
    key_Release_Flag = 0; // 重置松手标志（确保单次动作仅发送一次）
  }
  // 解锁条件：key_tick=0（1秒无操作/按键动作结束），允许下次发送
  if((key_tick == 0) && (key_Flag == 1))
  {
    key_Flag = 0;
  }
}
void  Battery_State(void) //上报电量情况
{
  //电压低于20%是上传数据包
  static uint8_t Send_Flag = 0;//发送标志位
  static uint8_t Flag = 0;//标志位(0--电压大于20,1--电压小于20)

  if(GetBattery() <= 20 && Send_Flag == 0)  
  {
    Flag = 1;
    // 组装数据包（5字节：包头0X55 0X52 + 功能码0XA2 + 低电压 + 包尾0X2B）
    uint8_t data_State1[5] = {0X55, 0X52, 0XA2, Flag, 0X2B};
    // 发送数据包（仅发送一次）
    Serial_SendArray(data_State1, 5);

    Send_Flag = 1;
  }

  //允许下次发送
  if(GetBattery() >= 20)
  {
    Flag = 0;
    Send_Flag = 0;
  }

  if(Send_Batter_Flag == 1) //上位机要求读取电量(百分比)
  {
    uint8_t data_State2[5] = {0X55, 0X52, 0XA8, GetBattery(), 0X2B};     
    // 发送数据包（仅发送一次）
    Serial_SendArray(data_State2, 5);
//		Send_Flag = 0;
    Send_Batter_Flag = 0;
  }
}

void Change_Battery(void) //上报充电状态
{
  //准备充电
  static uint8_t Send_Flag2;//发送标志位

  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 0 && Send_Flag2 == 0) //插入数据线充电且-当前电量低于 80%  
  {
    // 组装数据包（5字节：包头0X55 0X52 + 功能码0XA3 + 电 + 包尾0X2B）
    uint8_t data_State2[5] = {0X55, 0X52, 0XA3, 0X01, 0X2B};
    // 发送数据包（仅发送一次）
    Serial_SendArray(data_State2, 5);

    Send_Flag2 = 1;
  }

  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 1)
  {
    Send_Flag2 = 0;
  }

}

void Change_Battery_Full(void) //上报充满电
{
  static uint8_t Send_Flag3;//发送标志位

  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 0 && Send_Flag3 == 0) //充满电--低电平
  {
    // 组装数据包（5字节：包头0X55 0X52 + 功能码0XA3 + 电 + 包尾0X2B）
    uint8_t data_State2[5] = {0X55, 0X52, 0XA3, 0X02, 0X2B};
    // 发送数据包（仅发送一次）
    Serial_SendArray(data_State2, 5);

    Send_Flag3 = 1;
  }

  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 1)
  {
    Send_Flag3 = 0;
  }
}