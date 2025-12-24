
#include "######.h"

//定义转盘初始位角度，根据实际测试马嘴对着按键的位置来确定
#define TURN_TABLE_INIT_ANGLE 0  

static Struct_Key_HandleTypeDef key_handle;

uint8_t CompassDirection = 0;

int main(void)
{

    
    //各个模块初始化 -begin
    //......

    // 2. 按键初始化
    KEY_Init(&key_handle);

    //各个模块初始化 -end

    /****** 转盘回到初始位 -begin ******/
        
        //电机转动指定角度
        MOTOR_RotateToAngle(TURN_TABLE_INIT_ANGLE);

    /****** 转盘回到初始位 -end ******/

    
    
    
    while(1)
    {
        // 更新各个模块
        
        /****** 获取电量,低电量提示 -begin ******/
        
        if(getBatteryLevel() < = 20)
        {
            //发送串口数据：低电量提示
        }
        
        /****** 获取电量,低电量提示 -end ******/
        
        
        
        
        /****** 充电状态提示-begin ******/
        
        switch(getChargingStatus())
        {
            case CHARGING_UNABLE :
                //设置PA8电平，不使能充电芯片 (这个可以先不管)
                break;
            case CHARGING_STATUS_CHARGING:
                //发送串口数据：充电中
                break;
            case CHARGING_STATUS_FULL:
                //发送串口数据：已充电
                break;
        }

        /****** 充电状态提示 -end ******/




        /****** 按键状态监测 -begin ******/

        Enum_Key_StateTypeDef  key_state = KEY_GetState(&key_handle);
        // 处理按键状态
        if (key_state != KEY_STATE_IDLE) {
            switch (key_state)
            {
                case KEY_STATE_PRESSED:
                    // 处理按键按下事件
                    break;
                case KEY_STATE_RELEASED:
                    // 处理按键松开事件
                    break;
                case KEY_STATE_CLICK:
                    // 处理单击事件
                    break;
                case KEY_STATE_LONG_PRESS:
                    // 处理长按事件
                    break;
                case KEY_STATE_DOUBLE_CLICK:
                    // 处理双击事件
                    break;
                case KEY_STATE_MULTI_CLICK:
                    // 处理多击事件
                    uint8_t count = KEY_GetMultiClickCount(&key_handle); // 获取多击次数
                    break;
            }
        }
        
        /****** 按键状态监测 -end ******/




        /****** 获取当前指南针方位 -begin ******/

        //30秒获取一次指南针方位，不要用delay，使用运行时间来控制
        static uint32_t last_update_time = 0;
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_update_time >= 30000) {
            last_update_time = current_time;
            // 处理指南针方位数据
            CompassDirection = getCompassDirection ();
        }

        /****** 获取当前指南针方位 -end ******/



        /****** 串口接收数据 -begin ******/

        // 接收串口数据
        switch(getSerialData())
        {
            .....
        }

        /****** 串口接收数据 -end ******/
     

        

        
        
        
        // 处理系统状态
    }
}