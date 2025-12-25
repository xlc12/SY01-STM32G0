#include "main.h"

// #include "QMC5883P.h"
// #include "qmc_5883p_data.h"
// #include <string.h>
// //#include <stdio.h>
// #include <stdint.h>
// #include <stdlib.h>
// #include <string.h>

// #include <math.h>      // 用于atan2()函数
// #include <stdbool.h>   // 包含标准布尔类型定义
// #include <stdarg.h>
// #include "Timer.h"            
// #include "My_USART.h"
// #include "my_adc.h"

// #include "power_manage.h"

// #include "key_manage.h"
// #include "step_motor.h"


#define POWER_ON_TIMER 2000 //开机延时时间--2s


//开机转到初始角度
#define INITIAL_ANGLE 5//定义转盘初始位角度，根据实际测试马嘴对着按键的位置来确定 不能小于5度




// -------------------------- 电机参数配置（按实际硬件修改）--------------------------
#define STEP_ANGLE       18.0f      // 步距角，单位：度/步
#define REDUCTION_RATIO  118.0f     // 减速比
#define STEPS_PER_CIRCLE (360.0f / STEP_ANGLE * REDUCTION_RATIO)  // 转一圈所需步数

//电机速度配置
#define DEFAULT_SPEED_MS 1500 // 默认速度（ms）
#define MIN_SPEED_MS     1000 // 最快速度（ms）
#define MAX_SPEED_MS     5000 // 最慢速度（ms）

//pid 控制参数配置
#define PID_KP 0.5f
#define PID_KI 0.1f
#define PID_KD 0.1f

//设置电机转动到指定角度的角度容忍度为1度--MOTOR_RotateToAngle(int angle) 
#define ANGLE_TOLERANCE 1.0f
