#include "usart_manage.h"

extern uint8_t houseRotateTargetPoint_dir; //目标指向方位编码（0x01-0x08）

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
        
        default:
            break;
    }
}