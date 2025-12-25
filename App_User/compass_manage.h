#ifndef COMPASS_MANAGE_H
#define COMPASS_MANAGE_H

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





//获取指南针角度
float getCompassAngle();


//获取指南针方位
uint8_t getCompassDirection ();


#endif