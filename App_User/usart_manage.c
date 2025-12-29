#include "usart_manage.h"

extern uint8_t houseRotateTargetPoint_dir;   //目标指向方位编码（0x01-0x08）
extern uint8_t isPowerOff_flag;

// 串口发送命令
void Serial_SendHexCmd(uint8_t *data, uint16_t len)
{
    Serial_SendArray(data, len);
}


// 串口命令处理函数
void Uart_CommandHandler(uint8_t cmd, uint8_t* data, uint16_t len)
{
    // 处理命令
    switch (cmd)
    {
        //设置电机正反转1:正转，2:反转;
        case USART_CMD_MOTOR_RUN_CONTINUOUS:
            //STEP_MOTOR_FORWARD:1 ; STEP_MOTOR_REVERSE:2
            MOTOR_SetDirection(data[0]);  
            // Serial_Printf("USART_CMD_MOTOR_RUN_CONTINUOUS: %d\r\n", data[0]);
            break;

        //转指定角度命令
        case USART_CMD_MOTOR_ROTATE_TO_ANGLE:
            MOTOR_RotateToAngle(data[0]);
            break;
        
        //停止电机转动命令
        case USART_CMD_MOTOR_STOP:
            MOTOR_Stop();
            Serial_Printf("USART_CMD_MOTOR_STOP\r\n");
            break;

        //设置速度命令
        case USART_CMD_MOTOR_SET_SPEED:
            // MOTOR_SetSpeed(data[0]);
            break;

        //接收旋转目标方位命令：
        //北：1，东北：2，东：3，东南：4，南：5，西南：6，西：7，西北：8
        case USART_CMD_ROTATION_TARGET_DIRECTION:
             house_rotate.Target_dir = data[0];
            break;

        //接收磁力计校准角度命令
        case USART_CMD_CALIBRATION_ANGLE:
            
            break;

        //接收接收心跳命令  //返回设备信息：电量、转盘角度、磁力计角度、电机状态（待定）
        case USART_CMD_HEARTBEAT:
            // Serial_Printf("USART_CMD_HEARTBEAT\r\n");

            //构建心跳数据包
            uint8_t heartbeat[10] = {USART_CMD_HEAD1, USART_CMD_HEAD1, USART_S_CMD_HEARTBEAT, getBatteryLevel(), getTurntableAngle(), getCompassDirection(), USART_CMD_TAIL};
            Serial_SendHexCmd(heartbeat, sizeof(heartbeat));
            break;
        
        //关机
        case USART_CMD_SHUTDOWN:
            //关机
            isPowerOff_flag = 1;
            break;
        
        default:
            break;
    }
}



//设备信息上报情况结构体

typedef struct 
{
    //电压稳定标志位
    uint8_t voltage_stable_flag;
    uint8_t low_battery_reported_flag;
    uint8_t charging_reported_flag;
    uint8_t full_reported_flag;
}DeviceInfo_Report_t;




/*业务需求-4、设备信息循环检测发送接口*/
//设备信息循环检测发送接口
void DeviceInfo_CycleSend(void)
{
    static DeviceInfo_Report_t deviceInfo_Report = {0,0,0};
    
    
    static uint8_t last_battery_level = 0;

    uint8_t battery_level = getBatteryLevel();
    // //打印电量
    // // Serial_Printf("battery_level = %d\r\n", battery_level);
    // //连续获取的电压的差值少于5的时候，则才认为电压稳定
    // last_battery_level = battery_level;
    // if (battery_level!=0 && abs(battery_level - last_battery_level) < 3){
    //     deviceInfo_Report.voltage_stable_flag ++;
    // }else{
    //     return;
    // }

    

    
    //低电压上报
    if ( battery_level < 20 && deviceInfo_Report.low_battery_reported_flag == 0)
    {
        
        deviceInfo_Report.low_battery_reported_flag = 1;
        // 电量低，发送提示信息
        uint8_t low_battery[5] = {USART_CMD_HEAD1, USART_CMD_HEAD1, USART_S_CMD_LOW_BATTERY, battery_level, USART_CMD_TAIL};

        Serial_SendHexCmd(low_battery, sizeof(low_battery));
    }else if ( battery_level >= 20 && deviceInfo_Report.low_battery_reported_flag == 1)
    {

        deviceInfo_Report.low_battery_reported_flag = 0;

    }

    //充电状态上报
    uint8_t charging_status = getChargingStatus();
    if ( charging_status == POWER_CHARGING_STATUS && deviceInfo_Report.charging_reported_flag == 0)
    {
        deviceInfo_Report.charging_reported_flag = 1;
        // 充电中，发送提示信息
        uint8_t charging[5] = {USART_CMD_HEAD1, USART_CMD_HEAD1, USART_S_CMD_CHARGE, charging_status, USART_CMD_TAIL};
        Serial_SendHexCmd(charging, sizeof(charging));
    }
    
    if ( charging_status == POWER_FULL_STATUS && deviceInfo_Report.full_reported_flag == 0)
    {
        deviceInfo_Report.full_reported_flag = 1;
        // 充电完成，发送提示信息
        uint8_t full[5] = {USART_CMD_HEAD1, USART_CMD_HEAD1, USART_S_CMD_FULL, charging_status, USART_CMD_TAIL};
        Serial_SendHexCmd(full, sizeof(full));
    }


}

