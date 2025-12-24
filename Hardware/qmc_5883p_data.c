#include "main.h"
#include "qmc_5883p_data.h"
#include "QMC5883P.h"
#include <stdint.h>
#include <string.h>  // 用于memcpy
#include <math.h>

//QMC5883L变量
#define PI 3.1415926535f     // 新增：圆周率（用于弧度/角度转换）

int16_t qmcdata[3]; //原始磁力计数据
int16_t xd, yd, zd; //转换XYZ
float MagX, MagY, MagZ, MagH; // 滤波后的数据
float Azimuth;         //方位角（单位：度，0°=磁北，顺时针递增）
float Set_Azimuth; //校准的差值

/***************************************************************************/
uint8_t QMC5883_Init()  //初始化模块
{
  uint8_t uChipID = 0;
  // 设备复位，检查设备
  QMC_IIC_WriteByte(QMC5883_DEV, 0x29, 0X06); 	//定义 X、Y 和 Z 轴的符号
  QMC_IIC_WriteByte(QMC5883_DEV, 0x0A, 0XCD);	//定义设置/复位模式，设置/复位开启，场范围 8Guass
  QMC_IIC_WriteByte(QMC5883_DEV, 0x0B, 0X08);	//设置正常模式，设置 ODR=200Hz

  //ChipIDd地址为0x00，默认值为0x80
  if(QMC_IIC_ReadByte(QMC5883_DEV, 0x00, &uChipID) == 0  &&  uChipID == 0X80)
  {
    return 0;
  }
  else return 1;
}

/***************************************************************************/

uint8_t QMC5883_GetData(int16_t *QMC_data)         //数据获取
{
  uint8_t DRDY, OVL, DOR;
  uint8_t buf[6];
  //状态寄存器地址为0x09，第0位为DRDY，DRDY: “0”：没有新数据，“1”：新数据已就绪
  //						第1位为OVFL，OVFL: “0”: 未发生数据溢出， “1”: 发生数据溢出。
  //						当任一轴代码输出超出[-30000,30000] LSB 的范围时，OVFL 位被置为高电平，并在该位被读取后重置为“0”。
  QMC_IIC_ReadBits(QMC5883_DEV, 0X09, 0, 1, &DRDY);   //读状态寄存器  读第0位

  //所以当第0位为1的时候，即DRDY为1时
  if(DRDY == 1)                                         //如果数据就绪且没溢出&& OVL==0
  {
    DRDY = 0;
    QMC_IIC_ReadBytes(QMC5883_DEV, 0x01, 6, buf);    // 读取数据部分是前六个寄存器0x0~0x5

    QMC_data[0] = (((int16_t)buf[1]) << 8) | buf[0];
    QMC_data[1] =  (((int16_t)buf[3]) << 8) | buf[2];
    QMC_data[2] =  (((int16_t)buf[5] ) << 8) | buf[4];
    return 0;       //成功
  }
  else return 1;   //失败
}

/***************************************************************************/

//QMC5883P没有温度寄存器
float QMC5883_GetTemp_C(void)  // 读取并转换为实际室温（摄氏度）
{
  int16_t temp_raw = 0;       // 存储温度传感器原始16位数据
  uint8_t temp_buf[2] = {0};  // 存储读取的2个字节（低字节+高字节）

  // 1. 读取温度寄存器：0x07（低字节）、0x08（高字节），共2个字节
  // 注意：QMC5883L温度寄存器地址为0x07（低）和0x08（高），原始代码“0x07”正确，但需明确顺序
  if (QMC_IIC_ReadBytes(QMC5883_DEV, 0x07, 2, temp_buf) != 0)
  {
    return 0;  // 读取失败时返回“非数字”，便于上层判断
  }

  // 2. 拼接16位有符号原始数据（Little-Endian：低字节在前，高字节在后）
  temp_raw = (int16_t)((temp_buf[1] << 8) | temp_buf[0]);  // temp_buf[0]=0x07（低），temp_buf[1]=0x08（高）

  // 3. 按芯片手册公式转换为实际摄氏度（分辨率0.01℃，偏移25℃）
  float temp_c = (float)temp_raw / 100.0f + 40.0f;

  return temp_c;  // 返回实际室温（如25.36℃）
}

/***********************************************************
*	                       算法处理
***********************************************************/

// 滤波参数配置（不变）
#define WINDOW_SIZE 5       // 滑动窗口大小（越大越稳定，建议8-16）
#define LIMIT_THRESHOLD 50   // 限幅阈值（超过此值视为跳变，按需调整）
#define FILTER_ALPHA 0.2f    // 指数滤波系数（辅助平滑）
#define PI 3.1415926535f     // 新增：圆周率（用于弧度/角度转换）

// 滑动窗口缓冲区（静态变量，仅内部使用）
static int16_t x_buf[WINDOW_SIZE] = {0};
static int16_t y_buf[WINDOW_SIZE] = {0};
static int16_t z_buf[WINDOW_SIZE] = {0};
static uint8_t buf_index = 0;       // 缓冲区索引
static uint8_t is_initialized = 0;  // 初始化标志

// 限幅滤波：限制单次数据跳变幅度（不变）
static int16_t limit_filter(int16_t new_val, int16_t last_val)
{
  int32_t diff = (int32_t)new_val - last_val;

  if (diff > LIMIT_THRESHOLD)
  {
    return last_val + LIMIT_THRESHOLD;  // 超过上限，取上限值
  }
  else if (diff < -LIMIT_THRESHOLD)
  {
    return last_val - LIMIT_THRESHOLD;  // 低于下限，取下限值
  }

  return new_val;  // 正常范围内，取新值
}

// 滑动平均滤波：计算窗口内平均值（不变）
static float moving_average_filter(int16_t new_val, int16_t *buf)
{
  buf[buf_index] = new_val;  // 存入新值（覆盖最旧的值）

  // 计算窗口内总和
  int32_t sum = 0;

  for (uint8_t i = 0; i < WINDOW_SIZE; i++)
  {
    sum += buf[i];
  }

  return (float)sum / WINDOW_SIZE;  // 返回平均值
}

extern float Set_Azimuth; //校准的差值

//方位角计算函数（基于滤波后的X/Y轴数据）
static void calculate_azimuth(void)
{
  // 1. 核心逻辑：方位角 = arctan2(MagY, MagX) → 转换为角度（0°~360°）
  // 注：atan2(Y, X)返回弧度值，范围[-π, π]；需转换为[0, 360]度，0°=磁北，顺时针递增
  float rad = atan2(MagY, MagX);  // 关键：用Y轴为对边、X轴为邻边（匹配磁北坐标系）
  float angle = rad * 180.0f / PI;  // 弧度 → 角度（范围[-180°, 180°]）

  // 2. 调整角度范围：将[-180°, 180°]转为[0°, 360°]
  if (angle < 0.0f)
  {
    angle += 360.0f;  // 负角度加360，例如-90°→270°
  }

  // 3. 平滑处理：避免方位角跳变（可选，若角度波动大可加指数滤波）
  static float last_azimuth = 0.0f;
  Azimuth = 0.1f * angle + 0.9f * last_azimuth;  // 弱平滑（系数可调整）
  last_azimuth = Azimuth;

  // 4. 异常值保护：若X/Y均为0（无有效磁场），保持上次方位角
  if ((MagX < 0.1f && MagX > -0.1f) && (MagY < 0.1f && MagY > -0.1f))
  {
    Azimuth = last_azimuth;
  }
}

// 增强型组合滤波函数（无参数调用，新增方位角计算）
void qmc5883l_filter(void)
{
  // 首次初始化：填充缓冲区（不变）
  if (!is_initialized)
  {
    for (uint8_t i = 0; i < WINDOW_SIZE; i++)
    {
      x_buf[i] = qmcdata[0];
      y_buf[i] = qmcdata[1];
      z_buf[i] = qmcdata[2];
    }

    MagX = (float)qmcdata[0];
    MagY = (float)qmcdata[1];
    MagZ = (float)qmcdata[2];
    is_initialized = 1;
    return;
  }

  // 1. 限幅滤波：先去除突发跳变（不变）
  int16_t x_limit = limit_filter(qmcdata[0], (int16_t)MagX);
  int16_t y_limit = limit_filter(qmcdata[1], (int16_t)MagY);
  int16_t z_limit = limit_filter(qmcdata[2], (int16_t)MagZ);

  // 2. 滑动平均滤波：平滑数据（不变）
  float x_avg = moving_average_filter(x_limit, x_buf);
  float y_avg = moving_average_filter(y_limit, y_buf);
  float z_avg = moving_average_filter(z_limit, z_buf);

  // 3. 指数滤波：进一步细化平滑（不变）
  MagX = FILTER_ALPHA * x_avg + (1 - FILTER_ALPHA) * MagX;
  MagY = FILTER_ALPHA * y_avg + (1 - FILTER_ALPHA) * MagY;
  MagZ = FILTER_ALPHA * z_avg + (1 - FILTER_ALPHA) * MagZ;

  // 4. 更新缓冲区索引（不变）
  buf_index = (buf_index + 1) % WINDOW_SIZE;

  // 5. 计算磁场强度（不变）
  MagH = sqrt(MagX * MagX + MagY * MagY + MagZ * MagZ);

  // 6. 新增：计算方位角（调用自定义函数）
  calculate_azimuth();
}

// 定义全局变量：存储方向编码（0x01-0x08
uint8_t dir_code = 0;

/*
0x01 东
0x02 南
0x03 西
0x04 北
0x05 东南
0x06 东北
0x07 西南
0x08 西北
*/


/**
 * @brief 直接根据方位角给方向编码变量赋值
 * 角度区间（磁北0°顺时针）：
 * 东北(0x06)：337.5°~22.5° | 东(0x01)：22.5°~67.5° | 东南(0x05)：67.5°~112.5°
 * 南(0x02)：112.5°~157.5° | 西南(0x07)：157.5°~202.5° | 西(0x03)：202.5°~247.5°
 * 西北(0x08)：247.5°~292.5° | 北(0x04)：292.5°~337.5°
 */
void qmc_set_dir_code(void)
{
  if (Azimuth >= 337.5f || Azimuth < 22.5f)
  {
    dir_code = 0x06;  // 东北
  }
  else if (Azimuth >= 22.5f && Azimuth < 67.5f)
  {
    dir_code = 0x01;  // 东
  }
  else if (Azimuth >= 67.5f && Azimuth < 112.5f)
  {
    dir_code = 0x05;  // 东南
  }
  else if (Azimuth >= 112.5f && Azimuth < 157.5f)
  {
    dir_code = 0x02;  // 南
  }
  else if (Azimuth >= 157.5f && Azimuth < 202.5f)
  {
    dir_code = 0x07;  // 西南
  }
  else if (Azimuth >= 202.5f && Azimuth < 247.5f)
  {
    dir_code = 0x03;  // 西
  }
  else if (Azimuth >= 247.5f && Azimuth < 292.5f)
  {
    dir_code = 0x08;  // 西北
  }
  else if (Azimuth >= 292.5f && Azimuth < 337.5f)
  {
    dir_code = 0x04;  // 北
  }
}

extern  float Target_Azimuth; //校准后的最终角度
float Azimuth_off;         //校准角度差值

void Get_QMC5883P_Data(void) //获取数据
{
    // 磁力计数据读取
    QMC5883_GetData(qmcdata);//获取原始数据
    qmc_set_dir_code();  //得到磁力计实测方位角Azimuth（如318）

    // 核心计算：实测值 + 补偿偏移量 = 目标值（318+2=320）
    Target_Azimuth = Azimuth + Azimuth_off;

    // 范围修正：确保异常情况下仍在0~360（正常校准后此处不会触发，仅做容错）
    if (Target_Azimuth > 360)
    {
        Target_Azimuth -= 360;
    }
    else if (Target_Azimuth < 0)
    {
        Target_Azimuth += 360;
    }
}
/*
参数：无

参数说明：无

返回：1-8；东为1，东南为2，南为3 ，西南为4，西为5，西北为6 , 北为7，东北为8
*/

uint8_t getCompassDirection(void) //获取指南针方位
{
	return dir_code;  
}