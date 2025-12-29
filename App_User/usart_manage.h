#ifndef __USART_MANAGE_H__
#define __USART_MANAGE_H__

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
#include "my_adc.h"

#include "motor_manage.h"
#include "app_user.h"

#include "motor_manage.h"



//串口发送16进制命令
void Serial_SendHexCmd(uint8_t *data, uint16_t len);

//串口命令处理函数
void Uart_CommandHandler(uint8_t cmd, uint8_t* data, uint16_t len);

//设备信息循环检测发送接口
void DeviceInfo_CycleSend(void);

#endif
