#include "main.h"
#include "qmc_5883p_data.h"
#include "QMC5883P.h"
#include <stdint.h>
#include <string.h>  // 用于memcpy
#include <math.h>
#include "My_USART.h"
#include "app_user.h"
#include "usart_manage.h"
#include "my_flash.h"


// 全局校准偏移量（存储三轴硬铁干扰偏移值）
int16_t QMC_Offset_X = 0;
int16_t QMC_Offset_Y = 0;
int16_t QMC_Offset_Z = 0;

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
    uint8_t status_reg = 0;
    uint8_t buf[6];
    
    // 读取完整状态寄存器（0x09），一次性获取DRDY、OVL、DOR
    if(QMC_IIC_ReadByte(QMC5883_DEV, 0X09, &status_reg) != 0)
    {
        return 1; // IIC读取失败
    }
    
    DRDY = (status_reg >> 0) & 0x01; // 第0位：数据就绪
    OVL  = (status_reg >> 1) & 0x01; // 第1位：数据溢出
    DOR  = (status_reg >> 2) & 0x01; // 第2位：数据覆盖

    // 仅当数据就绪、无溢出、无覆盖时，才读取数据
    if(DRDY == 1 && OVL == 0 && DOR == 0)                                        
    {
        QMC_IIC_ReadBytes(QMC5883_DEV, 0x01, 6, buf);    // 读取0x01~0x06（原始数据）

        QMC_data[0] = (((int16_t)buf[1]) << 8) | buf[0];
        QMC_data[1] =  (((int16_t)buf[3]) << 8) | buf[2];
        QMC_data[2] =  (((int16_t)buf[5] ) << 8) | buf[4];
        return 0;       //成功
    }
    else 
    {
        // 若溢出/覆盖，可尝试清除状态（可选）
        if(OVL == 1 || DOR == 1)
        {
            uint8_t temp = 0;
            QMC_IIC_ReadByte(QMC5883_DEV, 0X09, &temp); // 读取状态寄存器自动清0
        }
        return 1;   //失败
    }
}

/***************************************************************************/
// 椭圆校准函数：自动采集10秒数据，计算三轴偏移量
void QMC5883_Calibrate(void)
{
    int16_t raw_data[3] = {0};
    int16_t max_x = -32768, min_x = 32767;  // 初始化X轴最大/最小值
    int16_t max_y = -32768, min_y = 32767;  // 初始化Y轴最大/最小值
    int16_t max_z = -32768, min_z = 32767;  // 初始化Z轴最大/最小值
    uint32_t calib_count = 0;               // 采集计数
    uint32_t total_count = CALIBRATION_TIME / CALIBRATION_INTERVAL;  // 总采集次数

    // 循环采集数据，持续6秒
    for(calib_count = 0; calib_count < total_count; calib_count++)
    {
        // 读取原始三轴数据
        if(QMC5883_GetData(raw_data) == 0)
        {
            // 更新X轴最大/最小值
            if(raw_data[0] > max_x) max_x = raw_data[0];
            if(raw_data[0] < min_x) min_x = raw_data[0];
            // 更新Y轴最大/最小值
            if(raw_data[1] > max_y) max_y = raw_data[1];
            if(raw_data[1] < min_y) min_y = raw_data[1];
            // 更新Z轴最大/最小值
            if(raw_data[2] > max_z) max_z = raw_data[2];
            if(raw_data[2] < min_z) min_z = raw_data[2];
        }
        HAL_Delay(CALIBRATION_INTERVAL);  // 采集间隔延迟
        
    }

    
    // Serial_Printf("QMC5883 Calibration Results:\n");
    // Serial_Printf("X: max=%d, min=%d, offset=%d\n", max_x, min_x, QMC_Offset_X);
    // Serial_Printf("Y: max=%d, min=%d, offset=%d\n", max_y, min_y, QMC_Offset_Y);
    // Serial_Printf("Z: max=%d, min=%d, offset=%d\n", max_z, min_z, QMC_Offset_Z);
    // 计算三轴偏移量（硬铁校准核心：偏移量 = (最大值 + 最小值) / 2）
    QMC_Offset_X = (max_x + min_x) / 2;
    QMC_Offset_Y = (max_y + min_y) / 2;
    QMC_Offset_Z = (max_z + min_z) / 2;

    // 擦除芯片
    FLASH_ErasePage(FLASH_START_ADDRESS1);
    FLASH_ErasePage(FLASH_START_ADDRESS2);
    FLASH_ErasePage(FLASH_START_ADDRESS3);
    
    HAL_Delay(200);
    //  写入数据
    FLASH_WriteInt32(FLASH_START_ADDRESS1,QMC_Offset_X);
    FLASH_WriteInt32(FLASH_START_ADDRESS2,QMC_Offset_Y);
    FLASH_WriteInt32(FLASH_START_ADDRESS3,QMC_Offset_Z);

    //判断是否写入成功
    int32_t Read_Data[3] = {0,0,0};

    FLASH_ReadInt32(FLASH_START_ADDRESS1,&Read_Data[0]);
    FLASH_ReadInt32(FLASH_START_ADDRESS2,&Read_Data[1]);
    FLASH_ReadInt32(FLASH_START_ADDRESS3,&Read_Data[2]);
    if(Read_Data[0] == QMC_Offset_X &&
        Read_Data[1] == QMC_Offset_Y &&
        Read_Data[2] == QMC_Offset_Z)
    {
        Serial_Printf("QMC5883 Calibration Success!\n");
    }
    else
    {
        Serial_Printf("QMC5883 Calibration Failed!\n");
    } 

    //打印QMC_Offset_X,QMC_Offset_Y,QMC_Offset_Z
    Serial_Printf("QMC5883_Calibrate**********QMC_Offset_X: %d, QMC_Offset_Y: %d, QMC_Offset_Z: %d\r\n", QMC_Offset_X, QMC_Offset_Y, QMC_Offset_Z);
    
    uint8_t cmd[5] = {USART_CMD_HEAD1, USART_CMD_HEAD2, USART_S_CMD_CALIBRATION_ANGLE, 01, USART_CMD_TAIL};
    Serial_SendHexCmd(cmd, sizeof(cmd));
}

/***********************************************************
*	                       算法处理
***********************************************************/
// 滤波参数配置（关键修改：调大限幅阈值，避免正常旋转被限制）
#define WINDOW_SIZE 5       // 保留原有窗口大小，无需修改
#define LIMIT_THRESHOLD 50   // 从50→200（核心修改1：放开正常旋转的数据限制）
#define FILTER_ALPHA 0.1f    // 保留原有系数，无需修改

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
    if (diff > LIMIT_THRESHOLD) {
        return last_val + LIMIT_THRESHOLD;  // 超过上限，取上限值
    } else if (diff < -LIMIT_THRESHOLD) {
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
    for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
        sum += buf[i];
    }
    return (float)sum / WINDOW_SIZE;  // 返回平均值
}
extern float Set_Azimuth; //校准的差值

//方位角计算函数
static void calculate_azimuth(void)
{
    // 1. 核心逻辑：方位角 = arctan2(MagY, MagX) → 转换为角度（0°~360°）
    float rad = atan2(MagY, MagX);  // 关键：用Y轴为对边、X轴为邻边（匹配磁北坐标系）
    float angle = rad * 180.0f / PI;  // 弧度 → 角度（范围[-180°, 180°]）
    
    // 2. 调整角度范围：将[-180°, 180°]转为[0°, 360°]
    if (angle < 0.0f) {
        angle += 360.0f;  // 负角度加360，例如-90°→270°
    }

    // 添加90度偏移，使北方向为0度
    angle = fmod(angle + 90.0f, 360.0f);
    
    // 3. 平滑处理：避免方位角跳变（可选，若角度波动大可加指数滤波）
    static float last_azimuth = 0.0f;
    Azimuth = 0.1f * angle + 0.9f * last_azimuth;  // 弱平滑（系数可调整）
    last_azimuth = Azimuth;
    
    // 4. 异常值保护：若X/Y均为0（无有效磁场），保持上次方位角
    if ((MagX < 0.1f && MagX > -0.1f) && (MagY < 0.1f && MagY > -0.1f)) {
        Azimuth = last_azimuth;
    }
}
// 增强型组合滤波函数（无参数调用，新增方位角计算）
void qmc5883l_filter(void)
{
    // 先对原始数据进行校准（减去硬铁偏移量）
    int16_t cali_x = qmcdata[0] - QMC_Offset_X;
    int16_t cali_y = qmcdata[1] - QMC_Offset_Y;
    int16_t cali_z = qmcdata[2] - QMC_Offset_Z;

    // 首次初始化：填充缓冲区（不变）
    if (!is_initialized) {
        for (uint8_t i = 0; i < WINDOW_SIZE; i++) 
        {
            x_buf[i] = cali_x;
            y_buf[i] = cali_y;
            z_buf[i] = cali_z;
        }
        MagX = (float)cali_x;
        MagY = (float)cali_y;
        MagZ = (float)cali_z;

        is_initialized = 1;
        return;
    }
    
    // 1. 限幅滤波：先去除突发跳变（不变）
    int16_t x_limit = limit_filter(cali_x, (int16_t)MagX);
    int16_t y_limit = limit_filter(cali_y, (int16_t)MagY);
    int16_t z_limit = limit_filter(cali_z, (int16_t)MagZ);
    
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
    qmc5883l_filter();// 滤波函数调用
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
