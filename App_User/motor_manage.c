#include "motor_manage.h"

void MOTOR_Init(void )
{
    //初始化电机
//    StepMotor_Init();



}


void MOTOR_SetDirection(Enum_Motor_StateTypeDef state) 
{
    // 设置电机转动方向
    // StepMotor_Control(state);
}


bool MOTOR_RotateToAngle(int angle) 
{
    // 电机转动到指定角度
    // StepMotor_RotateToAngle(angle);
    return true;
}


// 获取电机当前状态
uint8_t getMOTOR_State() 
{
    // 获取电机当前状态
    // return StepMotor_GetState();
    return MOTOR_STATE_STOP;
}


// 获取电机（转盘）当前角度
uint8_t getTurntableAngle()
{
    // 获取电机（转盘）当前角度
    // return StepMotor_GetAngle();
    return 0;
}

