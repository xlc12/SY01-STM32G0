#include "RTE_Components.h"             // Component selection
#include "Control.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "OLED.h"
#include "Timer.h"
#include "My_USART.h"
#include "QMC5883P.h"
#include "qmc_5883p_data.h"
#include "step_motor.h"
#include "my_adc.h"

extern int16_t qmcdata[3]; //原始磁力计数据
extern int16_t xd, yd, zd; //转换XYZ
extern float MagX, MagY, MagZ, MagH; // 滤波后的数据
extern float Azimuth;         //方位角（单位：度，0°=磁北，顺时针递增）
extern float Target_Azimuth; //校准后的最终角度

extern float ADC_BAT;//电池电压
extern float ADC_BAT_Cent;//电池电压(百分比)

float Step_Count = 0; //步进电需要的转动步数(步数)

uint8_t PID_Start;//运行pid控制标志位

float angle_P = 0.2, angle_I = 0.1;     //PID参数

float MPU_Turn(int actual_angle, int target_angle)
{
  float error = 0, error_last = 0, integral = 0, angle_out = 0;

  error = target_angle - actual_angle;
  integral += error; // 累加误差作为积分项

  /* PI算法实现 */
  angle_out = angle_P * error + angle_I * integral;

  /* 误差传递 */
  error_last = error;

  return angle_out;
}

extern uint8_t dir_code;          // 磁力计当前方向编码（0x01-0x08）
extern uint8_t Motor_Direction;   // 串口解析出的目标方向控制命令（0x01-0x08）

/*
方向编码定义（以代码注释为准）：
0x01 东
0x02 南
0x03 西
0x04 北
0x05 东南
0x06 东北
0x07 西南
0x08 西北
*/

// 记录上一次的总转动角度
static uint16_t Last_Total_Angle = 0;

void Step_PID_Control(void)//步进电机PID控制
{
    uint16_t Current_Angle;  // 当前方向对应的角度
    uint16_t Target_Angle;   // 目标方向对应的角度
	
   Current_Angle =  GetTurntableAngle();//获取当前角度(马儿的)
   Target_Angle = my_abs(dir_code-Motor_Direction)*45-Current_Angle; //3-1=2  6-8=2
   

	//pid
	Step_Count = MPU_Turn(GetTurntableAngle(), Target_Angle);        
	
	// 3. 控制步进电机转动对应角度
//    StepMotor_RotateSteps(Step_Count, 1); 
}


#define TURN_TABLE_INIT_ANGLE 35  //定义转盘初始位角度，根据实际测试马嘴对着按键的位置来确定 不能小于5度
uint8_t Flag = 0;
float Zero_Step_Count = 0; //PID

void  Start_Zero_Hourse(void) //执行上电复位马儿位置程序
{
	if(PID_Start==0&&Flag==0)
	{
		Zero_Step_Count = MPU_Turn(GetTurntableAngle(), TURN_TABLE_INIT_ANGLE);//PID运算 
//		StepMotor_RotateSteps(Zero_Step_Count, 1); //角度控制
		
		// 计算两个角度的绝对差值，判断是否在1度以内
		if(my_abs(GetTurntableAngle() - TURN_TABLE_INIT_ANGLE) <= 1) // 达到目标角度±1度范围内     
		{
			Flag = 1; //退出程序
		}
	}
}
	
// /**
//  * @brief  充电状态枚举定义
//  */
// typedef enum {
//     CHARGING_STATUS_DISABLE = 0,    /**< 禁止充电 (已连接充电器但未充电) */
//     CHARGING_STATUS_CHARGING = 1,   /**< 正在充电 */
//     CHARGING_STATUS_CHARGED = 2     /**< 充电完成 */
// } ChargingStatus_t; 

/**
 * @brief  获取当前充电状态
 * @note   充电中：读取PA11引脚状态；充电完成：读取PC6引脚状态；禁止充电：默认状态（暂不处理PA8）
 * @param  无
 * @retval ChargingStatus_t - 充电状态枚举值（禁止充电/正在充电/充电完成）
 */
// uint8_t getChargingStatus(void)
// {
//     // 1. 检测是否正在充电（PA11，高电平表示充电中）
//     if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET)
//     {
//         return CHARGING_STATUS_CHARGING;
//     }
    
//     // 2. 检测是否充电完成（PC6，高电平表示充电完成）
//     if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_SET)
//     {
//         return CHARGING_STATUS_CHARGED;
//     }
    
//     // 3. 默认返回禁止充电状态
//     return CHARGING_STATUS_DISABLE;
// }


