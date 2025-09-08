#ifndef MAIN_RGB_LED_H_
#define MAIN_RGB_LED_H_

typedef enum{
    LED_RED,            // 不在工作
    LED_GREEN,          // 在工作
    LED_BLUE_BLINK,     // 连接蓝牙
    LED_GREEN_BLINK,    // 在运动
    LED_OFF,            // 休眠
} LedState;

void led_init();
void set_led_state(LedState state);
void led_task(void *arg);

#endif // !MAIN_RGB_LED_H_