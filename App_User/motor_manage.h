#ifndef __MOTOR_MANAGE_H__
#define __MOTOR_MANAGE_H__

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

#include "power_manage.h"

#include "key_manage.h"
#include "step_motor.h"


//电机类型枚举，直流电机、步进电机、
typedef enum {
    MOTOR_TYPE_DC = 0,        // 直流电机
    MOTOR_TYPE_STEP,          // 步进电机
} Enum_Motor_TypeTypeDef;


// 电机初始化
void MOTOR_Init(void );

// 设置电机转动方向
void MOTOR_SetDirection(StepMotor_StateTypeDef state); 

// 电机转动到指定角度
void MOTOR_RotateToAngle(int angle);

// 获取电机当前状态
uint8_t getMOTOR_State();


/************************* PID控制相关定义 -begin *************************/

// PID控制器结构体
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float setpoint;     // 目标值
    float output;       // 输出值
    float last_error;   // 上一次的偏差
    float integral;     // 积分累积
    float derivative;   // 微分
    float max_output;   // 最大输出限制
    float min_output;   // 最小输出限制
} PID_ControllerTypeDef;





// ********** PID函数声明 **********

// 获取电机（转盘）当前角度
float getTurntableAngle();

// PID控制器初始化
void PID_Init(PID_ControllerTypeDef *pid, float Kp, float Ki, float Kd, float min_output, float max_output);

// 设置PID目标值
void PID_SetSetpoint(PID_ControllerTypeDef *pid, float setpoint);

// PID控制计算
float PID_Compute(PID_ControllerTypeDef *pid, float feedback);

// 使用PID控制电机转动到指定角度
bool MOTOR_RotateToAngleWithPID(int target_angle, float tolerance);

// 更新PID控制（需定时调用）
bool MOTOR_UpdatePIDControl(void);

/************************* PID控制相关定义 -end *************************/

#endif