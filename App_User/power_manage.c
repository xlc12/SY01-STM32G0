#include "power_manage.h"






//关机接口
void SYSTEM_PowerOff(int delay_ms)
{
    /* 实现将 IO（PA12）拉低即关机的功能 */
    /* 如果需要延时，先延时 delay_ms ms */

    HAL_Delay(delay_ms);
    // 立即执行关机：拉低PA12
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

    
}



//开机接口
void SYSTEM_PowerOn(int delay_ms)
{
    /* 实现将 IO（PA12）拉高的功能 */
    /* 如果需要延时，先延时 delay_ms ms */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);//PA12拉低
	HAL_Delay(delay_ms);//长按2s后自动识别开机
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); //PA12拉低--开机
}



//电池电压检测接口
float getBatteryVoltage(void)
{
    /* 实现从 PB0 读取ADC值并转换为电压的功能 */
    return convert_battery_voltage(ADC_BAT_ReadRawValue());

}





//电池电量检测接口
uint8_t getBatteryLevel(void)
{
    /* 调用 getBatteryVoltage() 读取电压并转换为电量百分比 */
    return GetBattery();
}

//充电状态检测接口,取值0-100
uint8_t getChargingStatus(void)
{
    /* 实现充电状态检测功能 */
    /* 充电中（读取PA11）；充电完成（读取PC6）；禁止充电（设置PA8电平） */
    static uint8_t charging_status = 0;//发送标志位，1充电中POWER_CHARGING_STATUS，2充电完成POWER_FULL_STATUS

    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 0) //充电中
    {
        
        charging_status = POWER_CHARGING_STATUS;
       
    }

    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 0) //充电完成
    {
        charging_status = POWER_FULL_STATUS;
    }
    return charging_status;
}


