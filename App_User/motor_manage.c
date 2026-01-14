#include "motor_manage.h"

// 全局变量定义
static PID_ControllerTypeDef motor_pid;  // 电机PID控制器
static float target_angle = 0.0f;        // 目标角度
static float angle_tolerance = 1.0f;     // 角度容忍度（默认1度）
static bool pid_control_active = false;  // PID控制激活标志

//超出PID角度范围处理标志
static int8_t is_out_of_pid_range = 0;   //负数为小于最小角度范围，正数为大于最大角度范围


//电机状态
 StepMotor_StateTypeDef motor_state = STEP_MOTOR_STOP;

extern uint8_t isBackInit_flag;
extern int32_t INITIAL_ANGLE_Flash;


uint16_t adc_max = 0;
uint16_t adc_min = 0;

//ADC值限制
#define ADC_MIN_LIMIT 250
#define ADC_MAX_LIMIT 4090
// 角度范围0-330
#define ANGLE_MIN_LIMIT 20
#define ANGLE_MAX_LIMIT 330

//电机类型定义
Enum_Motor_TypeTypeDef motor_type = MOTOR_TYPE_STEP;
// Enum_Motor_TypeTypeDef motor_type = MOTOR_TYPE_DC;

void MOTOR_Init(void )
{
    //初始化电机
    StepMotor_Init();
    motor_state = STEP_MOTOR_STOP;
    
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
    motor_state = state;
    
}

//电机转动到指定步进角度,STEPS_PER_CIRCLE 步为一圈,映射为360度
void MOTOR_RotateSteps_To_Angle(int32_t angle)
{
    // STEPS_PER_CIRCLE步为一圈,映射为360度
    int32_t steps = 0;
    steps = angle * STEPS_PER_CIRCLE / 360;
    StepMotor_RotateSteps(steps);
}

// 电机转动到指定角度
void MOTOR_RotateToAngle(int angle) 
{   
    angle_tolerance = ANGLE_TOLERANCE;  // 设置角度容忍度为1度
    MOTOR_RotateToAngleWithPID(angle, angle_tolerance);
    if(angle == INITIAL_ANGLE_Flash)
    {
        isBackInit_flag = 1;
    }

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
    pid_control_active = false;  // 关闭PID控制
    motor_state = STEP_MOTOR_STOP;
    
}


// 获 取电机当前状态
uint8_t getMOTOR_State() 
{
    // // return StepMotor_GetState();
    // if(StepMotor_GetState() == STEP_MOTOR_STOP)
    // {
    //     motor_state = STEP_MOTOR_STOP;
    // }
    return motor_state;
}


// 获取电机（转盘）当前角度
float getTurntableAngle()
{
    return ADC_PB1_ConvertToAngle();
}


//判断转盘是否在初始位
bool isTurntableInInitialPosition(void)
{
    float turntableAngle = 0;

    if(getMOTOR_State() != STEP_MOTOR_STOP){
        return false;
    }

    turntableAngle = getTurntableAdcConvertToAngle();
    //判断turntableAngle的值和初始位置的差值绝对值小于2度，则认为是初始位置，否则不是初始位置
    if(turntableAngle < INITIAL_ANGLE_Flash +INITIAL_ANGLE_THRESHOLD  
        && turntableAngle >  INITIAL_ANGLE_Flash -INITIAL_ANGLE_THRESHOLD)
    { 
      return true;
    }

    return false;
}


//将ADC值转换为角度,ADC值限制在250-4090,映射角度范围20-330
float getTurntableAdcConvertToAngle(void)
{
    float angle = 0.0f;
    uint16_t adc_raw = ADC_PB1_ReadRawValue();
    // 计算ADC值的最大最小值,限制在80-4090
    if(adc_raw > ADC_MAX_LIMIT)
    {
        adc_raw = ADC_MAX_LIMIT;
    }
    if(adc_raw < ADC_MIN_LIMIT)
    {
        adc_raw = ADC_MIN_LIMIT;
    }
    // // 映射ADC值到角度范围0-360
    // angle = (float)(adc_raw - ADC_MIN_LIMIT) / (float)(ADC_MAX_LIMIT - ADC_MIN_LIMIT) * 360.0f;
    

    
    // 将角度值限制在20-330之间，计算公式：(ADC - ADC_MIN) * (330 - 20) / (ADC_MAX - ADC_MIN) + 20
    angle = (float)(adc_raw - ADC_MIN_LIMIT) * (ANGLE_MAX_LIMIT - ANGLE_MIN_LIMIT) / (float)(ADC_MAX_LIMIT - ADC_MIN_LIMIT)  + ANGLE_MIN_LIMIT;
    
   
    return angle;
}

//开机转一圈获取ADC最大最小值
void getTurntableAdcMaxMinValue(void)
{
    
    Serial_Printf("adc_max = %d, adc_min = %d\r\n", adc_max, adc_min);
}



//关机动作
void MOTOR_PowerOff(void)
{   
    MOTOR_RotateToAngle(INITIAL_ANGLE_Flash+20);
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
    //角度限制0-360度
    if(target_angle_degree < 0)
    {
        target_angle_degree = 0;
    }
    else if(target_angle_degree >= 360)
    {
        target_angle_degree = 360;
    }

    // 检查是否超出PID控制范围
    //限制角度ANGLE_MIN_LIMIT-ANGLE_MAX_LIMIT
    if(target_angle_degree < ANGLE_MIN_LIMIT)
    {
        is_out_of_pid_range = target_angle_degree - ANGLE_MIN_LIMIT;
        target_angle_degree = ANGLE_MIN_LIMIT;
        //日志输出
        Serial_Printf("target_angle_degree = %d, is_out_of_pid_range = %d,\r\n", target_angle_degree, is_out_of_pid_range);
        
    }
    else if(target_angle_degree >= ANGLE_MAX_LIMIT)
    {
        is_out_of_pid_range = target_angle_degree - ANGLE_MAX_LIMIT;
        target_angle_degree = ANGLE_MAX_LIMIT;
        Serial_Printf("target_angle_degree = %d, is_out_of_pid_range = %d,\r\n", target_angle_degree, is_out_of_pid_range);

    }
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
    // float current_angle = getTurntableAngle();
    float current_angle = getTurntableAdcConvertToAngle();
    
    // 计算角度偏差
    float angle_error = fabs(target_angle - current_angle);
    
    // 检查是否达到目标角度
    if (angle_error <= angle_tolerance)
    {
        if(is_out_of_pid_range != 0)
        {
            // 超出PID角度范围，进行补偿转动
            MOTOR_RotateSteps_To_Angle(is_out_of_pid_range);
            is_out_of_pid_range = 0;
            // StepMotor_Stop();  // 停止电机
            pid_control_active = false;  // 关闭PID控制
            // motor_state = STEP_MOTOR_STOP;  // 更新电机状态
            // return true;  // 返回完成
        }
        else
        {
            StepMotor_Stop();  // 停止电机
            pid_control_active = false;  // 关闭PID控制
            motor_state = STEP_MOTOR_STOP;  // 更新电机状态
        }
            
        
        return true;  // 返回完成
    }
    
    // 执行PID计算
    float pid_output = PID_Compute(&motor_pid, current_angle);
    
    // 根据PID输出控制电机转动方向和速度
    if (pid_output > 0)
    {
        // 反转
        MOTOR_SetDirection(STEP_MOTOR_REVERSE);
        motor_state = STEP_MOTOR_REVERSE;
    }
    else if (pid_output < 0)
    {
        // 正转
        MOTOR_SetDirection(STEP_MOTOR_FORWARD);
        motor_state = STEP_MOTOR_FORWARD;
        
    }
    else
    {
        // 停止
        StepMotor_Stop();
        motor_state = STEP_MOTOR_STOP;
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