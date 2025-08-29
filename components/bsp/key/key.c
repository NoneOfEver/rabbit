#include "key.h"


/**
 * @brief       初始化按键
 * @param       无
 * @retval      无
 */
void key_init(void)
{
    gpio_config_t gpio_init_struct = {0};

    gpio_init_struct.intr_type = GPIO_INTR_DISABLE;         /* 失能引脚中断 */
    gpio_init_struct.mode = GPIO_MODE_INPUT;                /* 输入模式 */
    gpio_init_struct.pull_up_en = GPIO_PULLUP_ENABLE;       /* 使能上拉 */
    gpio_init_struct.pull_down_en = GPIO_PULLDOWN_DISABLE;  /* 失能下拉 */
    gpio_init_struct.pin_bit_mask = 1ull << BOOT_GPIO_PIN;  /* 设置的引脚的位掩码 */
    ESP_ERROR_CHECK(gpio_config(&gpio_init_struct));        /* 配置使能 */
}

/**
 * @brief       按键扫描函数
 * @note        该函数有响应优先级(同时按下多个按键): BOOT!!
 * @param       mode:0 / 1, 具体含义如下:
 *   @arg       0,  不支持连续按(当按键按下不放时, 只有第一次调用会返回键值,
 *                  必须松开以后, 再次按下才会返回其他键值)
 *   @arg       1,  支持连续按(当按键按下不放时, 每次调用该函数都会返回键值)
 * @retval      键值, 定义如下:
 *              BOOT_PRES, 1, BOOT按下
 */
uint8_t key_scan(uint8_t mode)
{
    static uint8_t key_up = 1;      /* 按键按松开标志 */
    uint8_t keyval = 0;

    if (mode) key_up = 1;           /* 支持连按 */

    if (key_up && (BOOT == 0))      /* 按键松开标志为1, 且有任意一个按键按下了 */
    {
        esp_rom_delay_us(20000);    /* 去抖动 */
        key_up = 0;

        if (BOOT == 0)  keyval = BOOT_PRES;


    }
    else if (BOOT == 1)             /* 没有任何按键按下, 标记按键松开 */
    {
        key_up = 1;
    }

    return keyval;                  /* 返回键值 */
}
