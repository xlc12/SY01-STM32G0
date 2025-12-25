#ifndef __MY_USART_H
#define __MY_USART_H
#include "main.h"

#define LENGTH  64  		//接收缓冲区大小

//弱2函数，实现在回调函数中处理具体的命令
// __weak void UART_CommandCallback(uint8_t cmd, uint8_t* data, uint16_t len);
//回调方式
typedef void (*UART_Callback_t)(uint8_t cmd, uint8_t* data, uint16_t len);
// 注册函数
void UART_RegisterCallback(UART_Callback_t callback);



void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_Printf(char *format, ...);

float my_abs(float a);
void Key_State(void);
void  Battery_State(void) ;//上报电量情况
void Change_Battery(void); //上报充电状态
void Change_Battery_Full(void) ;//上报充满电

#endif
