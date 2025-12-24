#ifndef __MY_ADC_H
#define __MY_ADC_H

#include "main.h"

  // 硬件参数定义（匹配原理图分压电路）
  #define ADC_BAT_RATIO      3.0f    // 分压比（R19=20K，R20=10K，总倍数3倍）
  #define ADC_REF_VOLTAGE    3.3f    // ADC参考电压（单位：V）
  #define ADC_RESOLUTION     4096.0f // 12位ADC分辨率（0~4095）

  #define ADC_CHANNEL_BAT    ADC_CHANNEL_8  // 电池电压采集通道
  #define ADC_CHANNEL_PB1    ADC_CHANNEL_9  // PB1采集通道

  // ADC句柄声明（需与c文件中一致）
  extern ADC_HandleTypeDef hadc1;

  /**
  * @brief  读取ADC1_IN8（电池）原始值
  * @return 原始采集值（0~4095）
  */
  uint32_t ADC_BAT_ReadRawValue(void);

  /**
  * @brief  计算电池实际电压
  * @return 电池电压值（单位：V，如3.7V、4.2V）
  */
  float ADC_BAT_ReadVoltage(void);

  /**
  * @brief  读取ADC1_IN9（PB1）原始值
  * @return 原始采集值（0~4095）
  */
  uint16_t ADC_PB1_ReadRawValue(void);

  /**
  * @brief  将PB1的ADC原始值线性转换为0~360度
  * @return 转换后的角度值（单位：度，范围0.0~360.0）
  */
  float ADC_PB1_ConvertToAngle(void);
  float convert_battery_voltage(uint16_t adc_value);
  float GetTurntableAngle(void);
  uint8_t GetBattery(void);

#endif /* __ADC_BAT_H */

// 新增空行，修复 #1-D 警告