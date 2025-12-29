#include "key_manage.h"



// 状态枚举（保留）
typedef enum {
    KEY_STATE_IDLE = 0,            
    KEY_STATE_PRESSED,             
    KEY_STATE_RELEASED,            
    KEY_STATE_LONG_PRESSED         
} Key_StateTypeDef;

// 按键控制结构体（保留）
/**
 * @brief 按键句柄结构体，用于存储按键的各种状态和配置信息
 */
typedef struct {
    Key_ConfigTypeDef config;            /**< 按键配置，包括引脚、模式等 */
    Key_StateTypeDef state;                /**< 按键当前状态，如按下、释放等 */
    uint8_t debounce_cnt;                    /**< 消抖计数器，用于按键消抖处理 */
    uint16_t press_time;                      /**< 按键按下持续时间，单位ms */
    uint16_t release_time;                  /**< 按键释放持续时间，单位ms */
    uint8_t click_cnt;                          /**< 按键点击次数计数器 */
    Key_EventTypeDef event;                /**< 按键事件类型，如单击、双击、长按等 */
    uint8_t long_press_triggered;    /**< 长按触发标志，1表示已触发长按事件 */
} Key_HandleTypeDef;

// 【关键1：确认硬件实际电平！】
// 若按键是“上拉输入+按下接地”→ active_level=0；若“下拉输入+按下接VCC”→ active_level=1
static const Key_ConfigTypeDef key1_config = {
    .port = GPIOB,
    .pin = GPIO_PIN_8,
    .active_level = 0  // 务必匹配实际硬件！
};

static Key_HandleTypeDef key_handle[KEY_MAX_NUM];  
static void (*key_event_callback)(Key_EventTypeDef, uint8_t) = NULL;

static void Key_Event_Process(Key_HandleTypeDef* key);

 

// 按键初始化（无修改）
void Key_HW_Init(void)
{
    // GPIO_InitTypeDef gpio_init = {0};

    // __HAL_RCC_GPIOA_CLK_ENABLE();

    // gpio_init.Pin = key1_config.pin;
    // gpio_init.Mode = GPIO_MODE_INPUT;
    // // 【关键2：上拉/下拉与active_level匹配】
    // // active_level=0 → 上拉（GPIO_PULLUP）；active_level=1 → 下拉（GPIO_PULLDOWN）
    // gpio_init.Pull = (key1_config.active_level == 0) ? GPIO_PULLUP : GPIO_PULLDOWN;
    // gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    // HAL_GPIO_Init(key1_config.port, &gpio_init);

    // 初始化句柄
    key_handle[0].config = key1_config;
    key_handle[0].state = KEY_STATE_IDLE;
    key_handle[0].debounce_cnt = 0;
    key_handle[0].press_time = 0;
    key_handle[0].release_time = 0;
    key_handle[0].click_cnt = 0;
    key_handle[0].event = KEY_EVENT_NONE;
    key_handle[0].long_press_triggered = 0;
}

void Key_Register_Event_Callback(void (*callback)(Key_EventTypeDef, uint8_t))
{
    if (callback != NULL)
    {
        key_event_callback = callback;
    }
}



static uint8_t KEY_CheckPinState(Key_HandleTypeDef* key) {

    
    static uint32_t debounce_time = 0;
    static uint8_t last_pressed_state = 0; // 初始为未按下
    
    
    uint8_t is_pressed = (HAL_GPIO_ReadPin(key->config.port, key->config.pin) == key->config.active_level) ? 1 : 0;
    
    // 防抖处理：连续20ms状态一致则确认
    if (is_pressed != last_pressed_state) {
        debounce_time = HAL_GetTick();
        last_pressed_state = is_pressed;
        return 2; // 返回2表示正在防抖
    } else {
        if (HAL_GetTick() - debounce_time >= KEY_DEBOUNCE_TIME) {
            // Serial_Printf("Key is %d\r\n", is_pressed);
            return is_pressed;
        }
        return 2; // 返回2表示正在防抖
    }
}




// 【核心修复：按键扫描逻辑】
void Key_Scan_All(void)
{
    for (uint8_t i = 0; i < KEY_MAX_NUM; i++)
    {
        Key_HandleTypeDef* key = &key_handle[i];

        uint8_t is_pressed = KEY_CheckPinState(key);

        if (is_pressed == 2)
        {
            /* code */
            continue; // 防抖中，跳过本次扫描
            // Serial_Printf("Key continue6666666666666666666\r\n");
        }
        

        

        // 2. 状态机处理
        switch (key->state)
        {
            case KEY_STATE_IDLE:
                if(is_pressed)
                {
                    key->state = KEY_STATE_PRESSED;
                    key->press_time = 0;       
                    key->click_cnt++;         
                    key->debounce_cnt = 0;    
                    key->long_press_triggered = 0;
                }
                break;

            case KEY_STATE_PRESSED:
                
                if (is_pressed)
                {
                    key->press_time += SCAN_INTERVAL_MS;
                    // 长按判定时间从1000ms→1500ms，降低误判概率
                    if (key->press_time >= LONG_PRESS_MS && !key->long_press_triggered)
                    {
                        key->event = KEY_EVENT_LONG_PRESS;
                        key->long_press_triggered = 1;
                        key->state = KEY_STATE_LONG_PRESSED;
                        key->click_cnt = 0;
                    }
                }
                else
                {
                   
                        key->state = KEY_STATE_RELEASED;
                        key->release_time = 0;
                        key->debounce_cnt = 0;
                        key->long_press_triggered = 0;
                    
                }
                break;

            case KEY_STATE_RELEASED:
                key->release_time += SCAN_INTERVAL_MS;

                // 检测再次按下（多击）
                if (is_pressed)
                {
                    
                        key->state = KEY_STATE_PRESSED;
                        key->press_time = 0;
                        key->release_time = 0;
                        key->click_cnt++;
                        key->debounce_cnt = 0;
                        key->long_press_triggered = 0;
                    
                }
                // 释放超时，判定点击事件
                else if (key->release_time >= MULTI_CLICK_MS)
                {
                    if (key->event == KEY_EVENT_NONE) 
                    {
                        if (key->click_cnt == 1)
                        {
                            key->event = KEY_EVENT_SINGLE_CLICK;
                        }
                        else if (key->click_cnt == 2)
                        {
                            key->event = KEY_EVENT_DOUBLE_CLICK;
                        }
                        else if (key->click_cnt >= 3)
                        {
                            key->event = KEY_EVENT_MULTI_CLICK;
                        }
                    }
                    // 重置所有状态
                    key->state = KEY_STATE_IDLE;
                    
                    key->long_press_triggered = 0;
                }
                break;

            case KEY_STATE_LONG_PRESSED:
                // 仅检测按键释放
                if (!is_pressed)
                {
                   
                        key->state = KEY_STATE_IDLE;
                        key->debounce_cnt = 0;
                        key->press_time = 0;
                        key->long_press_triggered = 0;
                        key->click_cnt = 0;
                    
                }
                break;

            default:
                key->state = KEY_STATE_IDLE;
                key->long_press_triggered = 0;
                key->click_cnt = 0;
                break;
        }

        // 3. 处理按键事件
        if (key->event != KEY_EVENT_NONE)
        {
            Key_Event_Process(key);
        }
    }
}

static void Key_Event_Process(Key_HandleTypeDef* key)
{
    if (key_event_callback != NULL)
    {
        key_event_callback(key->event, key->click_cnt);
    }
    key->event = KEY_EVENT_NONE;
    key->click_cnt = 0;
}