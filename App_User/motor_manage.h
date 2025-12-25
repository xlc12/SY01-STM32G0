#ifndef __MOTOR_MANAGE_H__
#define __MOTOR_MANAGE_H__


#include "app_user.h"

//电机类型枚举，直流电机、步进电机、
typedef enum {
    MOTOR_TYPE_DC = 0,        // 直流电机
    MOTOR_TYPE_STEP,          // 步进电机
} Enum_Motor_TypeTypeDef;


// 电机状态枚举
typedef enum {
    MOTOR_STATE_STOP = 0,      // 停止
    MOTOR_STATE_FORWARD,       // 正转
    MOTOR_STATE_REVERSE        // 反转
} Enum_Motor_StateTypeDef;


// 电机初始化
void MOTOR_Init(void );

// 设置电机转动方向
void MOTOR_SetDirection(Enum_Motor_StateTypeDef state); 

// 电机转动到指定角度
bool MOTOR_RotateToAngle(int angle);

// 获取电机当前状态
uint8_t getMOTOR_State();

// 获取电机（转盘）当前角度
uint8_t getTurntableAngle();

#endif