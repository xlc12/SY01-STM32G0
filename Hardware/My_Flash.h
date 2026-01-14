#ifndef __MY_FLASH_H
#define __MY_FLASH_H
#include "main.h"

/* STM32G031G8U6 Flash参数 */
//第29页
#define FLASH_PAGE_29_ADDRESS_0       0x0800E800  // 第29页起始地址
#define FLASH_PAGE_29_ADDRESS_1       0x0800E810  
#define FLASH_PAGE_29_ADDRESS_2       0x0800E820 
#define FLASH_PAGE_29_ADDRESS_3       0x0800E830  

//第30页
#define FLASH_PAGE_30_ADDRESS_0       0x0800F000  // 第30页起始地址
#define FLASH_PAGE_30_ADDRESS_1       0x0800F010  
#define FLASH_PAGE_30_ADDRESS_2       0x0800F020 
#define FLASH_PAGE_30_ADDRESS_3       0x0800F030  


//第31页
#define FLASH_PAGE_31_ADDRESS_0       0x0800F800  // 第31页起始地址
#define FLASH_PAGE_31_ADDRESS_1       0x0800F810
#define FLASH_PAGE_31_ADDRESS_2       0x0800F820







#define FLASH_START_ADDRESS       0x0800F800  // 最后一页起始地址

#define FLASH_START_ADDRESS1      0x0800F810  // 
#define FLASH_START_ADDRESS2      0x0800F820  // 
#define FLASH_START_ADDRESS3      0x0800F830  // 
#define FLASH_START_ADDRESS4      0x0800F840  // 

#define FLASH_START_ADDRESS5      0x0800F850  // 
#define FLASH_START_ADDRESS6      0x0800F860  // 
#define FLASH_START_ADDRESS7      0x0800F870  // 
#define FLASH_START_ADDRESS8      0x0800F880  // 



/* 函数声明 */
HAL_StatusTypeDef FLASH_ErasePage(uint32_t pageAddress);
HAL_StatusTypeDef FLASH_WriteInt32(uint32_t address, int32_t data);
HAL_StatusTypeDef FLASH_ReadInt32(uint32_t address, int32_t* data);

/* 数组操作函数 */
HAL_StatusTypeDef FLASH_WriteInt32Array(uint32_t startAddress, const int32_t* data, uint32_t length);
HAL_StatusTypeDef FLASH_ReadInt32Array(uint32_t startAddress, int32_t* data, uint32_t length);

#endif

