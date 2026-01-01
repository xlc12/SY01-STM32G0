#include "motor_manage.h"

// 全局变量定义
static PID_ControllerTypeDef motor_pid;  // 电机PID控制器
static float target_angle = 0.0f;        // 目标角度
static float angle_tolerance = 1.0f;     // 角度容忍度（默认1度）
static bool pid_control_active = false;  // PID控制激活标志

//电机类型定义
Enum_Motor_TypeTypeDef motor_type = MOTOR_TYPE_STEP;
// Enum_Motor_TypeTypeDef motor_type = MOTOR_TYPE_DC;

void MOTOR_Init(void )
{
    //初始化电机
    StepMotor_Init();
    
    // 初始化PID控制器（参数可根据实际情况调整）
    PID_Init(&motor_pid, PID_KP, PID_KI, PID_KD, -10.0f, 10.0f);
}

// 电机正反转，-一直转动STEP_MOTOR_FORWARD:1 ; STEP_MOTOR_REVERSE:2
void MOTOR_SetDirection(StepMotor_StateTypeDef state) 
{
    if(motor_type == MOTOR_TYPE_STEP)
    {
        StepMotor_RunContinuously(state);
    }
    else if(motor_type == MOTOR_TYPE_DC)
    {
        DC_Motor_SetDirection(state);
    }
    
}

//电机转动到指定步数
void MOTOR_RotateSteps(int32_t steps)
{
    StepMotor_RotateSteps(steps);
}

// 电机转动到指定角度
void MOTOR_RotateToAngle(int angle) 
{   
    angle_tolerance = ANGLE_TOLERANCE;  // 设置角度容忍度为1度
    MOTOR_RotateToAngleWithPID(angle, angle_tolerance);

}

// 电机停止转动
void MOTOR_Stop(void)
{
    if(motor_type == MOTOR_TYPE_STEP)
    {
        StepMotor_Stop();
    }
    else if(motor_type == MOTOR_TYPE_DC)
    {
        DC_Motor_Stop();
    }
}


// 获 取电机当前状态
uint8_t getMOTOR_State() 
{
    return StepMotor_GetState();
}


// 获取电机（转盘）当前角度
float getTurntableAngle()
{
    return ADC_PB1_ConvertToAngle();
}


//关机动作
void MOTOR_PowerOff(void)
{   
    MOTOR_RotateToAngle(INITIAL_ANGLE+20);
    // while(pid_control_active)
    // {
    //     // 等待PID控制完成
    // }
    // MOTOR_RotateToAngle(INITIAL_ANGLE-20);
    // while(pid_control_active)
    // {
    //     // 等待PID控制完成
    // }
    // MOTOR_RotateToAngle(INITIAL_ANGLE);
}




/************************** PID控制器 -begin **************************/
//初始化PID控制器
void PID_Init(PID_ControllerTypeDef *pid, float Kp, float Ki, float Kd, float min_output, float max_output)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = 0.0f;
    pid->output = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->min_output = min_output;
    pid->max_output = max_output;
}

// 设置PID目标值
void PID_SetSetpoint(PID_ControllerTypeDef *pid, float setpoint)
{
    pid->setpoint = setpoint;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
}

// PID控制计算
float PID_Compute(PID_ControllerTypeDef *pid, float feedback)
{
    // 计算当前偏差
    float error = pid->setpoint - feedback;
    
    // 计算积分项
    pid->integral += error;
    
    // 计算微分项
    pid->derivative = error - pid->last_error;
    pid->last_error = error;
    
    // PID公式计算
    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * pid->derivative;
    
    // // 输出限幅
    // if (pid->output > pid->max_output)
    // {
    //     pid->output = pid->max_output;
    // }
    // else if (pid->output < pid->min_output)
    // {
    //     pid->output = pid->min_output;
    // }
    
    return pid->output;
}

// 使用PID控制电机转动到指定角度
bool MOTOR_RotateToAngleWithPID(int target_angle_degree, float tolerance)
{
    // 设置目标角度和容忍度
    target_angle = (float)target_angle_degree;
    angle_tolerance = tolerance;
    
    // 设置PID目标值
    PID_SetSetpoint(&motor_pid, target_angle);
    
    // 激活PID控制
    pid_control_active = true;
    
    return true;
}

// 更新PID控制（需定时调用，建议10ms调用一次）
bool MOTOR_UpdatePIDControl(void)
{
    if (!pid_control_active)
    {
        return true;  // PID控制未激活，返回完成
    }
    
    // 获取当前角度
    float current_angle = getTurntableAngle();
    
    // 计算角度偏差
    float angle_error = fabs(target_angle - current_angle);
    
    // 检查是否达到目标角度
    if (angle_error <= angle_tolerance)
    {
        StepMotor_Stop();  // 停止电机
        pid_control_active = false;  // 关闭PID控制
        return true;  // 返回完成
    }
    
    // 执行PID计算
    float pid_output = PID_Compute(&motor_pid, current_angle);
    
    // 根据PID输出控制电机转动方向和速度
    if (pid_output > 0)
    {
        // 反转
        MOTOR_SetDirection(STEP_MOTOR_REVERSE);
    }
    else if (pid_output < 0)
    {
        // 正转
        MOTOR_SetDirection(STEP_MOTOR_FORWARD);
        
    }
    else
    {
        // 停止
        StepMotor_Stop();
    }
    
    // // 根据PID输出绝对值调整电机速度（这里简化处理，实际可以更复杂）
    // uint16_t speed_ms = (uint16_t)(10.0f - fabs(pid_output));  // 速度范围1-10ms
    // if (speed_ms < 1)
    //     speed_ms = 1;
    // if (speed_ms > 10)
    //     speed_ms = 10;
    
    // StepMotor_SetSpeed(speed_ms);
    
    return false;  // 未完成
}


/************************** PID控制器 -end **************************/