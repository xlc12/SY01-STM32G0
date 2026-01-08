#include "main.h"
#include "my_flash.h"

HAL_StatusTypeDef FLASH_ErasePage(uint32_t pageAddress)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  FLASH_EraseInitTypeDef eraseInit;
  uint32_t pageError = 0;

  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                         FLASH_FLAG_PGSERR);

  eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  eraseInit.Page = (pageAddress - FLASH_BASE) / FLASH_PAGE_SIZE;
  eraseInit.NbPages = 1;

  if (HAL_FLASHEx_Erase(&eraseInit, &pageError) == HAL_OK)
  {
    status = HAL_OK;
  }

  HAL_FLASH_Lock();
  return status;
}

HAL_StatusTypeDef FLASH_WriteInt32(uint32_t address, int32_t data)
{
  HAL_StatusTypeDef status = HAL_ERROR;

  // 确保地址是8字节对齐（双字写入要求）
  if ((address % 8) != 0 ||
      address < FLASH_START_ADDRESS ||
      address > (FLASH_START_ADDRESS + FLASH_PAGE_SIZE - 8))
  {
    return HAL_ERROR;
  }

  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                         FLASH_FLAG_PGSERR);

  // 使用FLASH_TYPEPROGRAM_DOUBLEWORD写入32位数据（低32位）
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, (uint64_t)data) == HAL_OK)
  {
    status = HAL_OK;
  }

  HAL_FLASH_Lock();
  return status;
}

HAL_StatusTypeDef FLASH_ReadInt32(uint32_t address, int32_t* data)
{
  if (address < FLASH_START_ADDRESS ||
      address > (FLASH_START_ADDRESS + FLASH_PAGE_SIZE - 4))
  {
    return HAL_ERROR;
  }

  *data = *((int32_t*)address);
  return HAL_OK;
}

HAL_StatusTypeDef FLASH_WriteInt32Array(uint32_t startAddress, const int32_t* data, uint32_t length)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint32_t address = startAddress;
  uint32_t i;

  // 确保起始地址是8字节对齐
  if ((startAddress % 8) != 0 ||
      startAddress < FLASH_START_ADDRESS ||
      (startAddress + length * 4) > (FLASH_START_ADDRESS + FLASH_PAGE_SIZE))
  {
    return HAL_ERROR;
  }

  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                         FLASH_FLAG_PGSERR);

  // 每次写入两个int32_t（一个64位双字）
  for (i = 0; i < length; i += 2)
  {
    uint64_t doubleWordData = 0;

    // 准备64位数据
    doubleWordData = (uint64_t)data[i];

    if (i + 1 < length)
    {
      doubleWordData |= ((uint64_t)data[i + 1] << 32);
    }

    // 写入双字
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, doubleWordData) != HAL_OK)
    {
      status = HAL_ERROR;
      break;
    }

    address += 8;
    status = HAL_OK;
  }

  HAL_FLASH_Lock();
  return status;
}

HAL_StatusTypeDef FLASH_ReadInt32Array(uint32_t startAddress, int32_t* data, uint32_t length)
{
  uint32_t address = startAddress;
  uint32_t i;

  if (startAddress < FLASH_START_ADDRESS ||
      (startAddress + length * 4) > (FLASH_START_ADDRESS + FLASH_PAGE_SIZE))
  {
    return HAL_ERROR;
  }

  for (i = 0; i < length; i++)
  {
    data[i] = *((int32_t*)address);
    address += 4;
  }

  return HAL_OK;
}