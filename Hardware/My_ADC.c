#include "my_adc.h"
#include "main.h"
#include <stdio.h>
#include "My_USART.h"
float ADC_Hourse;//马儿角度

/**
 * @brief  私有函数：配置指定ADC通道（匹配STM32G0 CubeMX配置）
 * @param  channel: 目标通道（ADC_CHANNEL_8/9）
 * @retval HAL状态：HAL_OK成功，其他失败
 */
static HAL_StatusTypeDef ADC_ConfigChannel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  // 完全对齐CubeMX生成的通道配置参数
  sConfig.Channel = channel;                          // 目标通道
  sConfig.Rank = ADC_REGULAR_RANK_1;                  // 与CubeMX一致的Rank
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;   // 使用CubeMX配置的公共采样时间（1.5周期）

  // 应用通道配置（切换到目标通道）
  return HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/**
 * @brief  读取ADC1_IN8（电池）原始值
 * @return 原始采集值（0~4095）
 */
uint32_t ADC_BAT_ReadRawValue(void)
{
  uint32_t adc_val = 0;

  // 第一步：切换到电池通道（ADC1_IN8）
  if (ADC_ConfigChannel(ADC_CHANNEL_BAT) != HAL_OK)
  {
    return 0; // 通道配置失败，返回0
  }

  // 第二步：启动ADC单次转换（软件触发，与CubeMX一致）
  HAL_ADC_Start(&hadc1);

  // 第三步：等待转换完成（超时10ms，避免卡死）
  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
  {
    // 读取12位原始值（右对齐，与CubeMX配置一致）
    adc_val = HAL_ADC_GetValue(&hadc1);
  }

  // 第四步：停止ADC转换
  HAL_ADC_Stop(&hadc1);

  return adc_val;
}

/**
 * @brief  计算电池实际电压
 * @return 电池电压（V）
 */
float ADC_BAT_ReadVoltage(void)
{
  float voltage = 0.0f;

  // 原始值转电压：(ADC值/分辨率)*参考电压 → 乘以分压比
  voltage = (uint16_t)ADC_BAT_ReadRawValue() * ADC_REF_VOLTAGE / ADC_RESOLUTION ;
  return voltage * ADC_BAT_RATIO;
}

/**
 * @brief  读取ADC1_IN9（PB1）原始值
 * @return 原始采集值（0~4095）
 */
uint16_t ADC_PB1_ReadRawValue(void)
{
  uint32_t adc_val = 0;

  // 第一步：切换到PB1通道（ADC1_IN9）
  if (ADC_ConfigChannel(ADC_CHANNEL_PB1) != HAL_OK)
  {
    return 0; // 通道配置失败，返回0
  }

  // 第二步：启动ADC单次转换
  HAL_ADC_Start(&hadc1);

  // 第三步：等待转换完成（超时10ms）
  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
  {
    adc_val = HAL_ADC_GetValue(&hadc1);
  }

  // 第四步：停止ADC转换
  HAL_ADC_Stop(&hadc1);

  return (uint16_t)adc_val;
}

/**
 * @brief  将PB1的ADC原始值（0~4095）线性转换为0~360度
 * @return 转换后的角度值（单位：度，范围0.0~360.0）
 * @note   线性映射：0→0°，4095→360°，中间值按比例转换
 */
float ADC_PB1_ConvertToAngle(void)
{
  uint16_t adc_raw = ADC_PB1_ReadRawValue();
  
  // 线性转换公式：角度 = (ADC值 / ADC最大值) * 目标角度最大值
  float angle = (adc_raw / ADC_RESOLUTION) * 360.0f;

  // 边界值修正（避免浮点误差导致超过360°）
  if (angle > 360.0f) angle = 360.0f;

  if (angle < 0.0f)   angle = 0.0f;

  return angle;
}

/**
 * @brief 将ADC采样值转换为锂电池电压值（带简单滤波）
 *
 * 该函数接收原始ADC采样值，通过简单的移动平均滤波算法处理，
 * 并根据电路参数计算出实际的锂电池电压值。
 *
 * @param adc_value 原始ADC采样值
 * @return float 转换后的锂电池电压值（单位：V）
 *
 * @note 电路参数（分压比、参考电压、ADC分辨率）已根据原理图调整
 */
float convert_battery_voltage(uint16_t adc_value)
{
  // 电路参数配置（根据原理图调整）
  const float data = 3.15f;       // ADC参考电压（单位：V）
  const float dat2 = 3.0f; // 分压比（20K+10K）/10K = 3.0
  const uint16_t dat3 = 4096;     // ADC分辨率（12位ADC为4096）

  // 滤波系数（0.0-1.0，越大滤波效果越弱，响应越快）
  const float FILTER_FACTOR = 0.5f;

  // 静态变量存储上一次的电压值（用于滤波）
  static float last_voltage = 0.0f;

  // 计算本次原始电压值
  float current_voltage = (float)adc_value * data / dat3 * dat2;

  // 简单滤波：使用一阶低通滤波器
  float filtered_voltage = last_voltage + FILTER_FACTOR * (current_voltage - last_voltage);

  // 更新上一次的电压值
  last_voltage = filtered_voltage;

  // 锂电池电压范围保护（3.7V电池的合理范围：2.5V-4.2V）
  if (filtered_voltage > 4.2f) filtered_voltage = 4.2f;

  if (filtered_voltage < 2.5f) filtered_voltage = 2.5f;

  return filtered_voltage;
}

float GetTurntableAngle(void)  //获取当前转盘位置(0-360度)
{
	 return ADC_PB1_ConvertToAngle();
}

uint8_t GetBattery(void)  //获取当前电量位置(直接转换为100%了,0-100%)
{
	float ADC_BAT;//电池电压
    uint8_t ADC_BAT_Cent;//电池电压(百分比)
	
	ADC_BAT = convert_battery_voltage(ADC_BAT_ReadRawValue());
    ADC_BAT_Cent = (ADC_BAT >= 4.2f) ? 100 : ((ADC_BAT <= 3.0f) ? 0 : (uint8_t)((ADC_BAT - 3.0f) / (4.2f - 3.0f) * 100));//得到电量百分比
	
    return ADC_BAT_Cent;    
}

