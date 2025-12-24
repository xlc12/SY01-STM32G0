#ifndef __CONTROL_H
#define __CONTROL_H
#include "main.h"             // Component selection

float MPU_Turn(int actual_angle, int target_angle);
void Step_PID_Control(void);//步进电机PID控制
void  Start_Zero_Hourse(void);

#endif
