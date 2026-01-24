#include "step_motor.h"
#include "main.h"
#include "tim.h"
#include <stdint.h>
#include <math.h>


//1是铜电机，0是钢电机
#if IS_OLD_MOTOR
// 反转序列表：A+相→B+相→A-相→B-相
static const uint8_t reverse_seq[4][4] =
{
  {1, 0, 0, 0}, // A+相
  {0, 0, 1, 0}, // B+相
  {0, 1, 0, 0}, // A-相
  {0, 0, 0, 1}  // B-相
  
};

// 正转序列表：与正转相反
static const uint8_t forward_seq[4][4] =
{
  
  {0, 0, 0, 1}, // B-相
  {0, 1, 0, 0}, // A-相
  {0, 0, 1, 0}, // B+相
  {1, 0, 0, 0}  // A+相
};
#else
// 反转序列表：A+相→B+相→A-相→B-相
static const uint8_t reverse_seq[4][4] =
{
  {0, 0, 0, 1}, // B-相
  {0, 1, 0, 0}, // A-相
  {0, 0, 1, 0}, // B+相
  {1, 0, 0, 0}  // A+相
  
};

// 正转序列表：与正转相反
static const uint8_t forward_seq[4][4] =
{
  
  {1, 0, 0, 0}, // A+相
  {0, 0, 1, 0}, // B+相
  {0, 1, 0, 0}, // A-相
  {0, 0, 0, 1}  // B-相
};
#endif


#define MOTOR_PORTA GPIOA


// -------------------------- 全局变量 --------------------------
static StepMotor_StateTypeDef g_motor_state = STEP_MOTOR_STOP;  // 电机当前状态
static uint8_t g_step_index = 0;  // 当前步进索引
static int32_t g_remaining_steps = 0;  // 剩余步数（0表示连续转动）
static uint16_t g_current_speed = DEFAULT_SPEED_MS;  // 当前速度（ms），默认1ms

extern StepMotor_StateTypeDef motor_state;



/**
 * @brief  设置电机相位状态
 * @param  phase: 相位值数组
 */
static void StepMotor_SetPhase(const uint8_t phase[4])
{
  HAL_GPIO_WritePin(MOTOR_PORTA, MOTOR_PIN_AIN1, (GPIO_PinState)phase[0]);
  HAL_GPIO_WritePin(MOTOR_PORTB, MOTOR_PIN_AIN2, (GPIO_PinState)phase[1]);
  HAL_GPIO_WritePin(MOTOR_PORTB, MOTOR_PIN_BIN1, (GPIO_PinState)phase[2]);
  HAL_GPIO_WritePin(MOTOR_PORTB, MOTOR_PIN_BIN2, (GPIO_PinState)phase[3]);
}

/**
 * @brief  步进电机初始化（配置GPIO为输出模式）
 */
void StepMotor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  // 1. 使能GPIOA和GPIOB时钟
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // 2. 配置AIN1为输出
  GPIO_InitStructure.Pin = MOTOR_PIN_AIN1;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MOTOR_PORTA, &GPIO_InitStructure);

  // 3. 配置AIN2/BIN1/BIN2为输出
  GPIO_InitStructure.Pin = MOTOR_PIN_AIN2 | MOTOR_PIN_BIN1 | MOTOR_PIN_BIN2;
  HAL_GPIO_Init(MOTOR_PORTB, &GPIO_InitStructure);

  // 4. 初始化为低电平，确保电机不转动
  StepMotor_SetPhase((uint8_t[]){0, 0, 0, 0});
}

/**u
 * @brief  设置电机速度
 * @param  speed_ms: 相序切换间隔（单位：ms，值越小速度越快，范围1000-5000us）
 */
void StepMotor_SetSpeed(uint16_t speed_ms)
{
  // 验证速度范围
  if (speed_ms < 1000) speed_ms = 1000;
  if (speed_ms > 5000) speed_ms = 5000;
  
  g_current_speed = speed_ms;
  
  // 如果电机正在运行，更新定时器周期
  // if (g_motor_state != STEP_MOTOR_STOP)
  // {
    TIM3_SetInterruptTime(speed_ms);
  // }
}

/**
 * @brief  立即停止电机
 */
void StepMotor_Stop(void)
{
  // 停止定时器3中断
  HAL_TIM_Base_Stop_IT(&htim3);
  
  // 关闭所有电机相位
  StepMotor_SetPhase((uint8_t[]){1, 1, 1, 1});
  
  // 更新状态
  motor_state = STEP_MOTOR_STOP;
  g_motor_state = STEP_MOTOR_STOP;
  g_remaining_steps = 0;
  g_step_index = 0;
}

/**
 * @brief  电机一直转
 * @param  dir: 方向选择（STEP_MOTOR_FORWARD-正转，STEP_MOTOR_REVERSE-反转）
 */
void StepMotor_RunContinuously(StepMotor_StateTypeDef dir)
{
  // 验证方向参数
  if (dir != STEP_MOTOR_FORWARD && dir != STEP_MOTOR_REVERSE)
  {
    return;
  }
  
  // 更新状态
  g_motor_state = dir;
  g_remaining_steps = 0;  // 0表示连续转动
  
  // 设置定时器速度
  TIM3_SetInterruptTime(g_current_speed);
  
  // 启动定时器3中断
  // HAL_TIM_Base_Start_IT(&htim3);
}

/**
 * @brief  控制电机转动指定步数
 * @param  steps：需要转动的步数（正数：逆时针转，负数：顺时针转）
 */
void StepMotor_RotateSteps(int32_t steps)
{
  if (steps == 0)
  {
    return;
  }
  
  // 更新状态和剩余步数
  if (steps > 0)
  {
    g_motor_state = STEP_MOTOR_REVERSE;
    g_remaining_steps = steps;
  }
  else
  {
    g_motor_state = STEP_MOTOR_FORWARD;
    g_remaining_steps = -steps;
  }
  
  // 设置定时器速度
  TIM3_SetInterruptTime(g_current_speed);
  
  // 启动定时器3中断
  // HAL_TIM_Base_Start_IT(&htim3);
}

/**
 * @brief  获取电机当前状态
 * @retval 当前状态：STEP_MOTOR_STOP/STEP_MOTOR_FORWARD/STEP_MOTOR_REVERSE
 */
StepMotor_StateTypeDef StepMotor_GetState(void)
{
  return g_motor_state;
}

/**
 * @brief  定时器3中断处理函数（在tim.c中调用）
 */
void StepMotor_TimerIRQHandler(void)
{

  const uint8_t (*seq)[4] = NULL;
  
  // 根据电机状态选择转向序列
  if (g_motor_state == STEP_MOTOR_FORWARD)
  {
    seq = forward_seq;
  }
  else if (g_motor_state == STEP_MOTOR_REVERSE)
  {
    seq = reverse_seq;
  }
  else
  {
    return;  // 如果停止状态，直接返回
  }
  
  // 设置当前相位
  StepMotor_SetPhase(seq[g_step_index]);
  
  // 更新步进索引
  g_step_index = (g_step_index + 1) % 4;
  
  // 如果是指定步数转动，更新剩余步数
  if (g_remaining_steps > 0)
  {
    g_remaining_steps--;
    
    // 如果步数完成，停止电机
    if (g_remaining_steps == 0)
    {
      StepMotor_Stop();
    }
  }
}





/****************** 直流电机控制 ******************/
void DC_Motor_Stop(void)
{
  StepMotor_SetPhase((uint8_t[]){1, 1, 1, 1});
}

//正反转
void DC_Motor_SetDirection(StepMotor_StateTypeDef dir)
{
  if(dir == STEP_MOTOR_FORWARD)
  {
    StepMotor_SetPhase((uint8_t[]){0, 1, 1, 1});
  }
  else if(dir == STEP_MOTOR_REVERSE)
  {
    StepMotor_SetPhase((uint8_t[]){1, 0, 1, 1});
  }
}