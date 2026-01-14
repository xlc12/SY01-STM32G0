#ifndef __QMC_5883P_DATA_H
#define __QMC_5883P_DATA_H

#include <stdint.h>

#define QMC5883_DEV (0x58)   //i2c地址

/*********************************************************************/
#define QMC5883L_REG_CONF1 0x09     //模式控制寄存器地址
// MOD模式（0x09的第0、1位）
#define QMC5883L_MODE 0x01          //0x01连续模式,
//#define QMC5883L_MODE 0x00        //0x00待机
/***************************************************************///10011101
// ODR输出速率（0x09的第2、3位）
//#define QMC5883L_ODR (0x00 << 2)   //10HZ   (0x00 << 2)
//#define QMC5883L_ODR (0x01 << 2)   //50HZ   (0x01 << 2)
//#define QMC5883L_ODR (0x02 << 2)   //100HZ  (0x02 << 2)
#define QMC5883L_ODR (0x03 << 2)     //200HZ	(0x03 << 2)
/***************************************************************/
//#define QMC5883L_RNG (0x01 << 4)    //RNG磁场量程（0x09的第4、5位）
//#define QMC5883L_RNG (0x01 << 4   //2G  (0x00 << 4)
#define QMC5883L_RNG (0x01 << 4)    //8G  (0x01 << 4)
/***************************************************************/
//OSR 采样滤波（0x09的第6、7位）越大波纹越小
//#define QMC5883L_OSR  (0x00 << 6)           //512   (0x00 << 6)
//#define QMC5883L_OSR  (0x01 << 6)           //256   (0x01 << 6)
//#define QMC5883L_OSR  (0x10 << 6)           //128   (0x10 << 6)
#define QMC5883L_OSR  (0x00 << 6)           //64    (0x11 << 6)

#define CONF1  QMC5883L_MODE | QMC5883L_ODR | QMC5883L_RNG | QMC5883L_OSR//数据拼接成8位 
/*********************************************************************/
#define QMC5883L_REG_CONF2  0x0A              //控制寄存器2地址
#define QMC5883L_INT_ENB    0x00           //0x00关闭引脚中断，0x01开启引脚中断，数据可读时触发
#define QMC5883L_ROL_PNT    (0x00 << 6)    //数据地址指针自增 ,0x00关闭，0x01开启
#define QMC5883L_SOFT_RST   (0x00 << 7)      //软复位，清除所有寄存器的值，0x01复位，0x00不复位

#define CONF2 QMC5883L_INT_ENB | QMC5883L_ROL_PNT |QMC5883L_SOFT_RST       //数据拼接
/*********************************************************************/

// 校准相关配置
#define CALIBRATION_TIME 20000U  // 校准时间：10秒（单位：毫秒）
#define CALIBRATION_INTERVAL 10U // 采集间隔：10毫秒（单位：毫秒）

// 全局校准偏移量（供外部调用）
extern int32_t QMC_Offset_X;
extern int32_t QMC_Offset_Y;
extern int32_t QMC_Offset_Z;

uint8_t QMC5883_Init();//初始化
uint8_t QMC5883_GetData(int16_t *QMC_data);//读数据
void qmc5883l_filter(void);
void qmc_set_dir_code(void);
void Get_QMC5883P_Data(void);
void qmc5883l_filter(void);//组合滤波
void QMC5883_Calibrate(void);// 椭圆校准函数（自动校准）
/*********************************************************************/

#define printf

#endif
