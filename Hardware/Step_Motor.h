#ifndef __STEP_MOTOR_H
#define __STEP_MOTOR_H

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

#include "tim.h"   // 定时器3头文件
#include "app_user.h"



// ************************* 引脚宏定义（按要求修改） *************************
#define MOTOR_PORTA    GPIOA
#define MOTOR_PORTB    GPIOB
#define MOTOR_PIN_AIN1 GPIO_PIN_15  // AIN1 -> GPIOA_15
#define MOTOR_PIN_AIN2 GPIO_PIN_3   // AIN2 -> GPIOB_3
#define MOTOR_PIN_BIN1 GPIO_PIN_4   // BIN1 -> GPIOB_4
#define MOTOR_PIN_BIN2 GPIO_PIN_5   // BIN2 -> GPIOB_5



// ************************* 函数声明 *************************
/**
 * @brief  步进电机初始化（配置GPIO为输出模式和定时器3）
 */
void StepMotor_Init(void);

/**
 * @brief  设置电机速度
 * @param  speed_ms: 相序切换间隔（单位：ms，值越小速度越快，范围1-10ms）
 */
void StepMotor_SetSpeed(uint16_t speed_ms);

/**
 * @brief  立即停止电机
 */
void StepMotor_Stop(void);

/**
 * @brief  电机一直转
 * @param  dir: 方向选择（STEP_MOTOR_FORWARD-正转，STEP_MOTOR_REVERSE-反转）
 */
void StepMotor_RunContinuously(StepMotor_StateTypeDef dir);

/**
 * @brief  控制电机转动指定步数
 * @param  steps：需要转动的步数（正数：正转，负数：反转）
 */
void StepMotor_RotateSteps(int32_t steps);

/**
 * @brief  获取电机当前状态
 * @retval 当前状态：STEP_MOTOR_STOP/STEP_MOTOR_FORWARD/STEP_MOTOR_REVERSE
 */
StepMotor_StateTypeDef StepMotor_GetState(void);

/**
 * @brief  定时器3中断处理函数（在tim.c中调用）
 */
void StepMotor_TimerIRQHandler(void);

#endif /* __STEP_MOTOR_H */