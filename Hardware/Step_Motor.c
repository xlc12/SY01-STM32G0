#include "step_motor.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
// -------------------------- 1. ����������壨����ʵ��Ӳ���޸ģ���--------------------------
#define STEP_ANGLE       18.0f      // ����ǣ���λ����/����
#define REDUCTION_RATIO  118.0f    // ���ٱ�
#define STEPS_PER_CIRCLE (360.0f / STEP_ANGLE * REDUCTION_RATIO)  // תһȦ���ܲ���

uint32_t g_motor_timer_ms; // �����ʱ����λ��10ms��

// ��ת�������У�A�������B�������A�෴���B�෴��
static const uint8_t forward_seq[4][4] =
{
  {1, 0, 0, 0}, // A+�ߣ�A-�ͣ�A������ͨ�磩��B�಻ͨ
  {0, 0, 1, 0}, // B+�ߣ�B-�ͣ�B������ͨ�磩��A�಻ͨ
  {0, 1, 0, 0}, // A+�ͣ�A-�ߣ�A�෴��ͨ�磩��B�಻ͨ
  {0, 0, 0, 1} // B+�ͣ�B-�ߣ�B�෴��ͨ�磩��A�಻ͨ
};

// ��ת�������У�����ת�෴
static const uint8_t reverse_seq[4][4] =
{
  {0, 0, 0, 1}, // B�෴��
  {0, 1, 0, 0}, // A�෴��
  {0, 0, 1, 0}, // B������
  {1, 0, 0, 0}  // A������
};

/**
 * @brief  �������GPIO��ʼ��
 */
void StepMotor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  // 1. ʹ��GPIOA��GPIOBʱ�ӣ�STM32G0 HAL��ʱ��ʹ�ܺ꣩
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // 2. ����AIN1��GPIOA_15��Ϊ�������
  GPIO_InitStructure.Pin = MOTOR_PIN_AIN1;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MOTOR_PORTA, &GPIO_InitStructure);

  // 3. ����AIN2/BIN1/BIN2��GPIOB_3/4/5��Ϊ�������
  GPIO_InitStructure.Pin = MOTOR_PIN_AIN2 | MOTOR_PIN_BIN1 | MOTOR_PIN_BIN2;
  HAL_GPIO_Init(MOTOR_PORTB, &GPIO_InitStructure);


  // 4. �ϵ�Ĭ�ϵ͵�ƽ�������ת����
  HAL_GPIO_WritePin(MOTOR_PORTA, MOTOR_PIN_AIN1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_PORTB, MOTOR_PIN_AIN2 | MOTOR_PIN_BIN1 | MOTOR_PIN_BIN2, GPIO_PIN_RESET);
}

/**
 * @brief  ��������������ƣ���/��ת��
 * @param  dir: 1-��ת��0-��ת
 * @param  speed_ms: �����ʱ�䣨ms��
 */
void StepMotor_Control(uint8_t dir, uint16_t speed_ms)
{
  static uint8_t step_index = 0;
  const uint8_t (*seq)[4] = dir ? forward_seq : reverse_seq;

  // �����ǰ���������䲻ͬ�˿ڵ����ţ�
  HAL_GPIO_WritePin(MOTOR_PORTA, MOTOR_PIN_AIN1, (GPIO_PinState)seq[step_index][0]);
  HAL_GPIO_WritePin(MOTOR_PORTB, MOTOR_PIN_AIN2, (GPIO_PinState)seq[step_index][1]);
  HAL_GPIO_WritePin(MOTOR_PORTB, MOTOR_PIN_BIN1, (GPIO_PinState)seq[step_index][2]);
  HAL_GPIO_WritePin(MOTOR_PORTB, MOTOR_PIN_BIN2, (GPIO_PinState)seq[step_index][3]);

  // ���²�����������ģʽ��
  step_index = (step_index + 1) % 4;

  HAL_Delay(speed_ms);
}

/**
 * @brief  ���Ƶ��ת��ָ������
 * @param  steps����������-��ת����-��ת��
 * @param  speed_ms�������ʱ�䣨ms��
 */
void StepMotor_RotateSteps(int32_t steps, uint16_t speed_ms)
{
  uint8_t dir = 1;  // 1-��ת��0-��ת
  uint32_t abs_steps;

  // ������ת�߼�
  if (steps < 0)
  {
    dir = 0;
    abs_steps = (uint32_t)(-steps);
  }
  else
  {
    abs_steps = (uint32_t)steps;
  }

  // ��ת��
  for (uint32_t i = 0; i < abs_steps; i++)
  {
    StepMotor_Control(dir, speed_ms);
  }
}

/**
 * @brief  ���Ƶ��ת��Ŀ��Ƕ�
 * @param  target_angle��Ŀ��Ƕȣ��ȣ�
 * @param  speed_ms�������ʱ�䣨ms��
 */
void StepMotor_RotateToAngle(float target_angle, uint16_t speed_ms)
{
  static float current_angle = 0.0f;

  // ������ԽǶȲ�
  float delta_angle = target_angle - current_angle;
  // �Ƕ�ת�������������룩
  int32_t steps = (int32_t)(delta_angle / STEP_ANGLE * REDUCTION_RATIO + 0.5f);
  // ִ��ת��
  StepMotor_RotateSteps(steps, speed_ms);
  // ���µ�ǰ�Ƕȣ������޶�����
  current_angle = target_angle;
}
