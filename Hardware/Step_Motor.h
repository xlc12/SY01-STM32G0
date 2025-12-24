#ifndef __STEP_MOTOR_H
#define __STEP_MOTOR_H

#include "main.h"  // 适配STM32G0系列HAL库

// ************************* 引脚宏定义（按要求修改） *************************
#define MOTOR_PORTA    GPIOA
#define MOTOR_PORTB    GPIOB
#define MOTOR_PIN_AIN1 GPIO_PIN_15  // AIN1 -> GPIOA_15
#define MOTOR_PIN_AIN2 GPIO_PIN_3   // AIN2 -> GPIOB_3
#define MOTOR_PIN_BIN1 GPIO_PIN_4   // BIN1 -> GPIOB_4
#define MOTOR_PIN_BIN2 GPIO_PIN_5   // BIN2 -> GPIOB_5

// ************************* 函数声明 *************************
/**
 * @brief  步进电机初始化（配置GPIO为输出模式）
 */
void StepMotor_Init(void);

/**
 * @brief  步进电机正反转控制
 * @param  dir: 方向选择（1-正转，0-反转）
 * @param  speed_ms: 相序切换间隔（单位：ms，值越小速度越快）
 */
void StepMotor_Control(uint8_t dir, uint16_t speed_ms);

/**
 * @brief  控制电机转动指定步数
 * @param  steps：需要转动的步数（正数：正转，负数：反转）
 * @param  speed_ms：每步间隔时间（ms，控制速度，值越小越快）
 */
void StepMotor_RotateSteps(int32_t steps, uint16_t speed_ms);

/**
 * @brief  控制电机转到目标角度
 * @param  target_angle：目标角度（单位：度）
 * @param  speed_ms：每步间隔时间（ms）
 */
void StepMotor_RotateToAngle(float target_angle, uint16_t speed_ms);

#endif /* __STEP_MOTOR_H */