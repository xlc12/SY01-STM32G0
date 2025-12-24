#ifndef __QMC5883P_H
#define __QMC5883P_H

#include "stm32g0xx_hal.h"
#include <stdint.h>
#include "main.h"

// 引脚定义（可根据实际硬件修改）
#define QMC_IIC_SCL_GPIO_PORT GPIOA
#define QMC_IIC_SCL_GPIO_PIN  GPIO_PIN_6
#define QMC_IIC_SDA_GPIO_PORT GPIOA
#define QMC_IIC_SDA_GPIO_PIN  GPIO_PIN_5

// 引脚操作宏
#define QMC_W_SCL(x)   HAL_GPIO_WritePin(QMC_IIC_SCL_GPIO_PORT, QMC_IIC_SCL_GPIO_PIN, (GPIO_PinState)(x))
#define QMC_W_SDA(x)   HAL_GPIO_WritePin(QMC_IIC_SDA_GPIO_PORT, QMC_IIC_SDA_GPIO_PIN, (GPIO_PinState)(x))
#define QMC_R_SDA()    HAL_GPIO_ReadPin(QMC_IIC_SDA_GPIO_PORT, QMC_IIC_SDA_GPIO_PIN)

// 函数声明
void QMC_IIC_Init(void);

uint8_t QMC_IIC_ReadBit  (uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
uint8_t QMC_IIC_ReadBits (uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
uint8_t QMC_IIC_ReadByte (uint8_t devAddr, uint8_t regAddr, uint8_t *data);
uint8_t QMC_IIC_ReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
uint8_t QMC_IIC_ReadWord (uint8_t devAddr, uint8_t regAddr, uint16_t *data);
uint8_t QMC_IIC_ReadWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);

uint8_t QMC_IIC_WriteBit  (uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
uint8_t QMC_IIC_WriteBits (uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
uint8_t QMC_IIC_WriteByte (uint8_t devAddr, uint8_t regAddr, uint8_t data);
uint8_t QMC_IIC_WriteBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
uint8_t QMC_IIC_WriteWord (uint8_t devAddr, uint8_t regAddr, uint16_t data);
uint8_t QMC_IIC_WriteWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data);

#endif