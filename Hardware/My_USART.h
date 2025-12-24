#ifndef __MY_USART_H
#define __MY_USART_H
#include "main.h"

#define LENGTH  64  		//接收缓冲区大小

void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_Printf(char *format, ...);

float my_abs(float a);
void Key_State(void);
void  Battery_State(void) ;//上报电量情况
void Change_Battery(void); //上报充电状态
void Change_Battery_Full(void) ;//上报充满电

#endif
