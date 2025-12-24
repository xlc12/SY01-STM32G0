#include "QMC5883P.h"
#include "main.h"

/***********************************************************
*	QMC iic 引脚初始化
***********************************************************/
void QMC_IIC_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  // 使能GPIOB时钟
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // 配置SCL和SDA引脚为开漏输出
  GPIO_InitStructure.Pin = QMC_IIC_SCL_GPIO_PIN | QMC_IIC_SDA_GPIO_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;  // 开漏输出
  GPIO_InitStructure.Pull = GPIO_PULLUP;          // 上拉
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(QMC_IIC_SCL_GPIO_PORT, &GPIO_InitStructure);

  // 初始化为高电平
  QMC_W_SCL(1);
  QMC_W_SDA(1);
}

/***********************************************************
*	QMC iic 起始信号
***********************************************************/
static void QMC_IIC_Begin(void)
{
  QMC_W_SDA(1);
  QMC_W_SCL(1);
  QMC_W_SDA(0);
  QMC_W_SCL(0);
}

/***********************************************************
*	QMC iic 终止信号
***********************************************************/
static void QMC_IIC_End(void)
{
  QMC_W_SDA(0);
  QMC_W_SCL(1);
  QMC_W_SDA(1);
}

/***********************************************************
*	QMC iic 发送应答
***********************************************************/
static void QMC_IIC_SendACK(uint8_t ack)
{
  QMC_W_SDA(ack);
  QMC_W_SCL(1);
  QMC_W_SCL(0);
}

/***********************************************************
*	QMC iic 接收应答
***********************************************************/
static uint8_t QMC_IIC_RecvACK(void)
{
  uint8_t ack;
  QMC_W_SDA(1); // 释放SDA
  QMC_W_SCL(1);
  ack = QMC_R_SDA();
  QMC_W_SCL(0);
  return ack;
}

/***********************************************************
*	QMC iic 发送一个字节
***********************************************************/
static void QMC_IIC_SendByte(uint8_t byte)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    QMC_W_SDA( (byte & (0x80 >> i)) );
    QMC_W_SCL(1);
    QMC_W_SCL(0);
  }
}

/***********************************************************
*	QMC iic 接收一个字节
***********************************************************/
static uint8_t QMC_IIC_RecvByte(void)
{
  uint8_t byte = 0;
  QMC_W_SDA(1);  // 释放SDA

  for (uint8_t i = 0; i < 8; i++)
  {
    QMC_W_SCL(1);

    if (QMC_R_SDA() == 1)
    {
      byte |= (0x80 >> i);
    }

    QMC_W_SCL(0);
  }

  return byte;
}

/***********************************************************
*	QMC iic 读一个字节
***********************************************************/
uint8_t QMC_IIC_ReadByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data)
{
  return QMC_IIC_ReadBytes(devAddr, regAddr, 1, data);
}

/***********************************************************
*	QMC iic 读多个字节
***********************************************************/
uint8_t QMC_IIC_ReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
  uint8_t ack;
  QMC_IIC_Begin();
  QMC_IIC_SendByte(devAddr);
  ack = QMC_IIC_RecvACK();

  if (ack != 0) return 1;

  QMC_IIC_SendByte(regAddr);
  ack = QMC_IIC_RecvACK();

  if (ack != 0) return 1;

  QMC_IIC_Begin();
  QMC_IIC_SendByte(devAddr | 0x01);
  ack = QMC_IIC_RecvACK();

  if (ack != 0) return 1;

  *data = QMC_IIC_RecvByte();

  for (int i = 1; i < length; i++)
  {
    QMC_IIC_SendACK(0);
    *(data + i) = QMC_IIC_RecvByte();
  }

  QMC_IIC_SendACK(1);
  QMC_IIC_End();
  return 0;
}

/***********************************************************
*	QMC iic 写一个字节
***********************************************************/
uint8_t QMC_IIC_WriteByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
  return QMC_IIC_WriteBytes(devAddr, regAddr, 1, &data);
}

/***********************************************************
*	QMC iic 写多个字节
***********************************************************/
uint8_t QMC_IIC_WriteBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
  uint8_t ack;
  QMC_IIC_Begin();
  QMC_IIC_SendByte(devAddr);
  ack = QMC_IIC_RecvACK();

  if (ack != 0) return 1;

  QMC_IIC_SendByte(regAddr);
  ack = QMC_IIC_RecvACK();

  if (ack != 0) return 1;

  for (int i = 0; i < length; i++)
  {
    QMC_IIC_SendByte((*(data + i)));
    ack = QMC_IIC_RecvACK();

    if (ack != 0) return 1;
  }

  QMC_IIC_End();
  return 0;
}

/***********************************************************
*	QMC iic 读一个比特
***********************************************************/
uint8_t QMC_IIC_ReadBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
  uint8_t status = QMC_IIC_ReadByte(devAddr, regAddr, data);

  if (status != 0) return 1;

  *data = (*data >> bitNum ) & 0x01;
  return 0;
}

/***********************************************************
*	QMC iic 写一个比特
***********************************************************/
uint8_t QMC_IIC_WriteBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
  uint8_t b;
  uint8_t status = QMC_IIC_ReadByte(devAddr, regAddr, &b);

  if (status != 0) return 1;

  b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
  return QMC_IIC_WriteByte(devAddr, regAddr, b);
}

/***********************************************************
*	QMC iic 读多个比特
***********************************************************/
uint8_t QMC_IIC_ReadBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
  uint8_t count, b;

  if ((count = QMC_IIC_ReadByte(devAddr, regAddr, &b)) == 0)
  {
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    b &= mask;
    b >>= (bitStart - length + 1);
    *data = b;
  }

  return count;
}

/***********************************************************
*	QMC iic 写多个比特
***********************************************************/
uint8_t QMC_IIC_WriteBits (uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
  uint8_t b;

  if (QMC_IIC_ReadByte(devAddr, regAddr, &b) == 0)
  {
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1);
    data &= mask;
    b &= ~(mask);
    b |= data;
    return QMC_IIC_WriteByte(devAddr, regAddr, b);
  }
  else
  {
    return 1;
  }
}

/***********************************************************
*	QMC iic 读多个字
***********************************************************/
uint8_t QMC_IIC_ReadWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data)
{
  uint8_t ack;
  uint16_t count = 0;
  QMC_IIC_Begin();
  QMC_IIC_SendByte(devAddr);
  ack = QMC_IIC_RecvACK();

  if (ack != 0) return 1;

  QMC_IIC_SendByte(regAddr);
  ack = QMC_IIC_RecvACK();

  if (ack != 0) return 1;

  QMC_IIC_Begin();
  QMC_IIC_SendByte(devAddr | 0x01);
  ack = QMC_IIC_RecvACK();

  if (ack != 0) return 1;

  for (int i = 0; i < length * 2; i++)
  {
    data[count] = QMC_IIC_RecvByte() << 8;
    QMC_IIC_SendACK(0);
    data[count++] |= QMC_IIC_RecvByte();

    if ( i == length * 2 - 1 )
      QMC_IIC_SendACK(1);
    else
      QMC_IIC_SendACK(0);
  }

  QMC_IIC_End();
  return 0;
}

/***********************************************************
*	QMC iic 读一个字
***********************************************************/
uint8_t QMC_IIC_ReadWord (uint8_t devAddr, uint8_t regAddr, uint16_t *data)
{
  return QMC_IIC_ReadWords(devAddr, regAddr, 1, data);
}

/***********************************************************
*	QMC iic 写多个字
***********************************************************/
uint8_t QMC_IIC_WriteWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data)
{
  uint8_t ack;
  QMC_IIC_Begin();
  QMC_IIC_SendByte(devAddr);
  ack = QMC_IIC_RecvACK();

  if (ack != 0) return 1;

  QMC_IIC_SendByte(regAddr);
  ack = QMC_IIC_RecvACK();

  if (ack != 0) return 1;

  for (int i = 0; i < length; i++)
  {
    QMC_IIC_SendByte(data[i] >> 8);
    ack = QMC_IIC_RecvACK();

    if (ack != 0) return 1;

    QMC_IIC_SendByte(data[i]);
    ack = QMC_IIC_RecvACK();

    if (ack != 0) return 1;
  }

  QMC_IIC_End();
  return 0;
}

/***********************************************************
*	QMC iic 写一个字
***********************************************************/
uint8_t QMC_IIC_WriteWord (uint8_t devAddr, uint8_t regAddr, uint16_t data)
{
  return QMC_IIC_WriteWords(devAddr, regAddr, 1, &data);
}