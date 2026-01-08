#include "usart_manage.h"

extern uint8_t houseRotateTargetPoint_dir;   //目标指向方位编码（0x01-0x08）
extern uint8_t isPowerOff_flag;
extern HouseRotateStruct house_rotate;
extern float Calibration_Offset; //校准偏移量
extern  float Target_Azimuth; //校准后的最终角度

// 串口发送命令
void Serial_SendHexCmd(uint8_t *data, uint16_t len)
{
    Serial_SendArray(data, len);
}


// 串口接收命令处理函数
void Uart_CommandHandler(uint8_t cmd, uint8_t* data, uint16_t len)
{
    Serial_Printf("1111111111111166666666666666:\r\n");
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
            MOTOR_RotateToAngle(data[0] * 2);
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
            QMC5883_Calibrate();
            
            break;

        //接收接收心跳命令  //返回设备信息：电量、转盘角度、磁力计角度、电机状态（待定）
        case USART_CMD_HEARTBEAT:
            // Serial_Printf("USART_CMD_HEARTBEAT\r\n");
            //打印心跳数据
            // Serial_Printf("USART_CMD_HEARTBEAT: %d, %d, %d, %d\r\n", 
            //     getBatteryLevel(),
            //     (uint16_t)getTurntableAngle(),
            //     (uint16_t)getCompassAngle(),
            //     getCompassDirection());
            //构建心跳数据包
            uint8_t heartbeat[] = {
                USART_CMD_HEAD1, USART_CMD_HEAD1, USART_S_CMD_HEARTBEAT,
                getBatteryLevel(),
                (uint8_t)((uint16_t)getTurntableAngle() / 2),    // 0-360映射到0-180
                (uint8_t)((uint16_t)getCompassAngle() / 2),      // 0-360映射到0-180
                getCompassDirection(),
                USART_CMD_TAIL
            };
            Serial_SendHexCmd(heartbeat, sizeof(heartbeat));
            break;

        //接收磁力计标定方位命令,上位机端发送需要除2的值，因为实际角度是0-360，而串口只能发送0-255
        //底盘端接收也需要乘2，因为实际角度是0-360，而串口只能发送0-255
        //
        case USART_CMD_CALIBRATION_DIR:
            Calibration_Offset = data[0] * 2 - Target_Azimuth; //计算偏移量
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
    //电池电量标志
    uint8_t ChaBattery_Level_Statusrging_Status;
    //充电状态标志
    uint8_t Charging_Status;
}DeviceInfo_Report_t;




/*业务需求-4、设备信息循环检测发送接口*/
//设备信息循环检测发送接口
void DeviceInfo_CycleSend(void)
{
    static DeviceInfo_Report_t deviceInfo_Report = {0};
    
    
    static uint8_t last_battery_level = 0;

    uint8_t battery_level = getBatteryLevel();

    uint8_t position_data[] = {
        USART_CMD_HEAD1, USART_CMD_HEAD1, USART_S_CMD_POSITION_STATUS,
        (uint8_t)((uint16_t)getTurntableAngle() / 2),    // 0-360映射到0-180
        (uint8_t)((uint16_t)getCompassAngle_Raw() / 2),  //原始角度    // 0-360映射到0-180
        getMOTOR_State(),
        USART_CMD_TAIL
    };
    Serial_SendHexCmd(position_data, sizeof(position_data));

    
    //低电压上报
    if ( battery_level < 20 && deviceInfo_Report.ChaBattery_Level_Statusrging_Status == 0)
    {
        deviceInfo_Report.ChaBattery_Level_Statusrging_Status = 1;
        // 电量低，发送提示信息
        uint8_t low_battery[5] = {USART_CMD_HEAD1, USART_CMD_HEAD1, USART_S_CMD_LOW_BATTERY, battery_level, USART_CMD_TAIL};

        Serial_SendHexCmd(low_battery, sizeof(low_battery));
    }else if ( battery_level >= 20 && deviceInfo_Report.ChaBattery_Level_Statusrging_Status == 1)
    {
        deviceInfo_Report.ChaBattery_Level_Statusrging_Status = 0;
    }


    //充电状态上报
    uint8_t charg_status = getChargingStatus();
    // Serial_Printf("1111111 Charging_Status = %d\r\n", charg_status);
    if(charg_status == deviceInfo_Report.Charging_Status)
    {
        // Serial_Printf("2222222 Charging_Status not change\r\n");
        return;
    }
    //充电中上报
    if ( charg_status == POWER_CHARGING_STATUS)
    {
        deviceInfo_Report.Charging_Status = POWER_CHARGING_STATUS;
        // Serial_Printf("3333333 Charging_Status = %d\r\n", charg_status);
        // 充电中，发送提示信息
        uint8_t charging[5] = {USART_CMD_HEAD1, USART_CMD_HEAD1, USART_S_CMD_CHARGE, charg_status, USART_CMD_TAIL};
        Serial_SendHexCmd(charging, sizeof(charging));
    }
    
    //充电完成上报
    if ( charg_status == POWER_FULL_STATUS)
    {
        // Serial_Printf("4444444 Charging_Status = %d\r\n", charg_status);
        deviceInfo_Report.Charging_Status = POWER_FULL_STATUS;
        // 充电完成，发送提示信息
        uint8_t full[5] = {USART_CMD_HEAD1, USART_CMD_HEAD1, USART_S_CMD_CHARGE, charg_status, USART_CMD_TAIL};
        Serial_SendHexCmd(full, sizeof(full));
    }

    //取消充电上报
    if ( charg_status == CHARGING_CANCEL_STATUS)
    {
        // Serial_Printf("5555555 Charging_Status = %d\r\n", charg_status);
        deviceInfo_Report.Charging_Status = CHARGING_CANCEL_STATUS;
        // 取消充电，发送提示信息
        // uint8_t not_charging[5] = {USART_CMD_HEAD1, USART_CMD_HEAD1, CHARGING_CANCEL_STATUS, Charging_Status, USART_CMD_TAIL};
        // Serial_SendHexCmd(not_charging, sizeof(not_charging));
    }

    // Serial_Printf("6666666 Charging_Status = %d\r\n", deviceInfo_Report.Charging_Status);


}

