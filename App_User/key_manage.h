#ifndef __KEY_H
#define __KEY_H

#include "app_user.h"



/************************** 通用配置 **************************/
#define KEY_MAX_NUM        1       // 支持的最大按键数
#define SCAN_INTERVAL_MS   10      // 扫描间隔（ms）
#define DEBOUNCE_COUNT     3       // 消抖次数（2*10ms=20ms）
#define KEY_DEBOUNCE_TIME  20     // 短按判定时间（ms）
#define LONG_PRESS_MS      2000    // 长按判定时间（ms）
#define MULTI_CLICK_MS     400     // 多击间隔时间（ms）

/************************** 按键事件类型 **************************/
typedef enum {
    KEY_EVENT_NONE = 0,            // 无事件
    KEY_EVENT_SINGLE_CLICK,        // 单击
    KEY_EVENT_DOUBLE_CLICK,        // 双击
    KEY_EVENT_MULTI_CLICK,         // N次点击（≥3次）
    KEY_EVENT_LONG_PRESS           // 长按
} Key_EventTypeDef;

/************************** 按键配置结构体 **************************/
typedef struct {
    GPIO_TypeDef* port;            // 按键端口
    uint16_t pin;                  // 按键引脚
    uint8_t active_level;          // 有效电平（0=低电平按下，1=高电平按下）
} Key_ConfigTypeDef;

/************************** 函数声明 **************************/
// 按键硬件初始化
void Key_HW_Init(void);
//KEY_CheckPinState

// 按键扫描（定时器中断调用）
void Key_Scan_All(void);
// 注册按键事件回调函数（应用层实现）
void Key_Register_Event_Callback(void (*callback)(Key_EventTypeDef, uint8_t));

#endif /* __KEY_H */