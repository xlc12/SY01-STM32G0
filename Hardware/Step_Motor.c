#include "step_motor.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
// -------------------------- 1. 电机参数定义（根据实际硬件修改！）--------------------------
#define STEP_ANGLE       18.0f      // 步距角（单位：度/步）
#define REDUCTION_RATIO  118.0f    // 减速比
#define STEPS_PER_CIRCLE (360.0f / STEP_ANGLE * REDUCTION_RATIO)  // 转一圈的总步数

uint32_t g_motor_timer_ms; // 电机计时（单位：10ms）

// 正转四拍序列：A相正向→B相正向→A相反向→B相反向
static const uint8_t forward_seq[4][4] =
{
  {1, 0, 0, 0}, // A+高，A-低（A相正向通电），B相不通
  {0, 0, 1, 0}, // B+高，B-低（B相正向通电），A相不通
  {0, 1, 0, 0}, // A+低，A-高（A相反向通电），B相不通
  {0, 0, 0, 1} // B+低，B-高（B相反向通电），A相不通
};

// 反转四拍序列：与正转相反
static const uint8_t reverse_seq[4][4] =
{
  {0, 0, 0, 1}, // B相反向
  {0, 1, 0, 0}, // A相反向
  {0, 0, 1, 0}, // B相正向
  {1, 0, 0, 0}  // A相正向
};

/**
 * @brief  步进电机GPIO初始化
 */
void StepMotor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  // 1. 使能GPIOA和GPIOB时钟（STM32G0 HAL库时钟使能宏）
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // 2. 配置AIN1（GPIOA_15）为推挽输出
  GPIO_InitStructure.Pin = MOTOR_PIN_AIN1;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MOTOR_PORTA, &GPIO_InitStructure);

  // 3. 配置AIN2/BIN1/BIN2（GPIOB_3/4/5）为推挽输出
  GPIO_InitStructure.Pin = MOTOR_PIN_AIN2 | MOTOR_PIN_BIN1 | MOTOR_PIN_BIN2;
  HAL_GPIO_Init(MOTOR_PORTB, &GPIO_InitStructure);


  // 4. 上电默认低电平（电机不转动）
  HAL_GPIO_WritePin(MOTOR_PORTA, MOTOR_PIN_AIN1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_PORTB, MOTOR_PIN_AIN2 | MOTOR_PIN_BIN1 | MOTOR_PIN_BIN2, GPIO_PIN_RESET);
}

/**
 * @brief  步进电机单步控制（正/反转）
 * @param  dir: 1-正转，0-反转
 * @param  speed_ms: 步间隔时间（ms）
 */
void StepMotor_Control(uint8_t dir, uint16_t speed_ms)
{
  static uint8_t step_index = 0;
  const uint8_t (*seq)[4] = dir ? forward_seq : reverse_seq;

  // 输出当前步相序（适配不同端口的引脚）
  HAL_GPIO_WritePin(MOTOR_PORTA, MOTOR_PIN_AIN1, (GPIO_PinState)seq[step_index][0]);
  HAL_GPIO_WritePin(MOTOR_PORTB, MOTOR_PIN_AIN2, (GPIO_PinState)seq[step_index][1]);
  HAL_GPIO_WritePin(MOTOR_PORTB, MOTOR_PIN_BIN1, (GPIO_PinState)seq[step_index][2]);
  HAL_GPIO_WritePin(MOTOR_PORTB, MOTOR_PIN_BIN2, (GPIO_PinState)seq[step_index][3]);

  // 更新步索引（四拍模式）
  step_index = (step_index + 1) % 4;

  HAL_Delay(speed_ms);
}

/**
 * @brief  控制电机转动指定步数
 * @param  steps：步数（正-正转，负-反转）
 * @param  speed_ms：步间隔时间（ms）
 */
void StepMotor_RotateSteps(int32_t steps, uint16_t speed_ms)
{
  uint8_t dir = 1;  // 1-正转，0-反转
  uint32_t abs_steps;

  // 处理反转逻辑
  if (steps < 0)
  {
    dir = 0;
    abs_steps = (uint32_t)(-steps);
  }
  else
  {
    abs_steps = (uint32_t)steps;
  }

  // 逐步转动
  for (uint32_t i = 0; i < abs_steps; i++)
  {
    StepMotor_Control(dir, speed_ms);
  }
}

/**
 * @brief  控制电机转到目标角度
 * @param  target_angle：目标角度（度）
 * @param  speed_ms：步间隔时间（ms）
 */
void StepMotor_RotateToAngle(float target_angle, uint16_t speed_ms)
{
  static float current_angle = 0.0f;

  // 计算相对角度差
  float delta_angle = target_angle - current_angle;
  // 角度转步数（四舍五入）
  int32_t steps = (int32_t)(delta_angle / STEP_ANGLE * REDUCTION_RATIO + 0.5f);
  // 执行转动
  StepMotor_RotateSteps(steps, speed_ms);
  // 更新当前角度（假设无丢步）
  current_angle = target_angle;
}
