#ifndef __APP_USER_H__
#define __APP_USER_H__

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

#define POWER_OFF_TIMER 3000 //关机延时时间--2s

//充电中状态
#define POWER_CHARGING_STATUS 0x01
//充满状态
#define POWER_FULL_STATUS 0x02




//开机转到初始角度
#define INITIAL_ANGLE 5//定义转盘初始位角度，根据实际测试马嘴对着按键的位置来确定 不能小于5度





// ************************* 电机状态定义 *************************
typedef enum {
    STEP_MOTOR_STOP = 0,  // 停止状态
    STEP_MOTOR_FORWARD,   // 正转状态
    STEP_MOTOR_REVERSE    // 反转状态
} StepMotor_StateTypeDef;




//财神位旋转结构体定义：
typedef struct  
{
    int Current_dir; //当前底座指向方位编码（0x01-0x08）
    int Target_dir;  //马头目标指向方位编码（0x01-0x08）
    float Current_Angle;         //当前马头指向角度
} HouseRotateStruct;

extern HouseRotateStruct house_rotate;




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


//串口命令定义，命令格式为帧头0x55 0x52 + 命令字 +命令 + 帧尾
#define USART_CMD_HEAD1 0x55
#define USART_CMD_HEAD2 0x52
#define USART_CMD_TAIL  0x5B //帧尾

//串口*接收*命令字定义，
/*
1、电机一直转：0为正转，1为反转
2、转指定角度：正数为顺时针，负数为逆时针
3、停止电机转动
4、设置速度：1000-5000us
5、接收旋转目标方位
6、接收磁力计校准角度
7、接收心跳：返回设备信息：电量、转盘角度、磁力计角度、电机状态
8、关机
*/
//电机一直转命令
#define USART_CMD_MOTOR_RUN_CONTINUOUS 0x01
//转指定角度命令
#define USART_CMD_MOTOR_ROTATE_TO_ANGLE 0x02
//停止电机转动命令
#define USART_CMD_MOTOR_STOP 0x03
//设置速度命令
#define USART_CMD_MOTOR_SET_SPEED 0x04
//接收旋转目标方位
#define USART_CMD_ROTATION_TARGET_DIRECTION 0x05
//接收磁力计校准角度命令
#define USART_CMD_CALIBRATION_ANGLE 0x06
//接收心跳命令
#define USART_CMD_HEARTBEAT 0x07
//关机命令
#define USART_CMD_SHUTDOWN 0x08



//串口*发送*命令字定义，
/*
1、低电压警告上报
2、充电提示上报
3、充满提示上报
4、按键次数上报
*/
//低电压警告上报命令
#define USART_S_CMD_LOW_BATTERY 0x0A1
//充电提示上报命令
#define USART_S_CMD_CHARGE 0x0A2
//充满提示上报命令
#define USART_S_CMD_FULL 0x0A3
//按键次数上报命令
#define USART_S_CMD_KEY_COUNT 0x0A4
//磁力计方向上报命令
#define USART_S_CMD_COMPASS_ANGLE 0x0A5









#endif /* __APP_USER_H__ */





