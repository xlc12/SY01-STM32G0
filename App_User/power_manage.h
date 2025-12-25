#ifndef __POWER_MANAGE_H__
#define __POWER_MANAGE_H__

#include "app_user.h"



/**
 * @brief  充电状态枚举定义
 */
typedef enum {
    CHARGING_STATUS_CHARGING = 1,    /**< 正在充电 */
    CHARGING_STATUS_CHARGED = 2     /**< 充电完成 */
} Enum_ChargingStatus;

//关机接口
void SYSTEM_PowerOff(int delay_ms);

//开机接口
void SYSTEM_PowerOn(int delay_ms);

//电池电压检测接口
float getBatteryVoltage(void);

//电池电量检测接口
uint8_t getBatteryLevel(void);

//充电状态检测接口
uint8_t getChargingStatus(void);

#endif /* __POWER_MANAGE_H__ */