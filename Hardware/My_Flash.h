#ifndef __MY_FLASH_H
#define __MY_FLASH_H
#include "main.h"

/* STM32G031G8U6 Flash参数 */
#define FLASH_START_ADDRESS       0x0800F800  // 最后一页起始地址

#define FLASH_START_ADDRESS1      0x0800F810  // 第27页起始（倒数第5页）
#define FLASH_START_ADDRESS2      0x0800F820  // 第28页起始（倒数第4页）
#define FLASH_START_ADDRESS3      0x0800F830  // 第29页起始（倒数第3页）
#define FLASH_START_ADDRESS4      0x0800F840  // 第30页起始（倒数第2页）

#define FLASH_START_ADDRESS5      0x0800F850  // 第27页起始（倒数第5页）
#define FLASH_START_ADDRESS6      0x0800F860  // 第28页起始（倒数第4页）
#define FLASH_START_ADDRESS7      0x0800F870  // 第29页起始（倒数第3页）
#define FLASH_START_ADDRESS8      0x0800F880  // 第30页起始（倒数第2页）

/* 函数声明 */
HAL_StatusTypeDef FLASH_ErasePage(uint32_t pageAddress);
HAL_StatusTypeDef FLASH_WriteInt32(uint32_t address, int32_t data);
HAL_StatusTypeDef FLASH_ReadInt32(uint32_t address, int32_t* data);

/* 数组操作函数 */
HAL_StatusTypeDef FLASH_WriteInt32Array(uint32_t startAddress, const int32_t* data, uint32_t length);
HAL_StatusTypeDef FLASH_ReadInt32Array(uint32_t startAddress, int32_t* data, uint32_t length);

#endif

