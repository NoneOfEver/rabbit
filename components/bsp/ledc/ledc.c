#include "ledc.h"


/**
 * @brief       计算一个周期的占空比计数值
 * @param       duty:   占空比
 * @param       m^n:    输入参数
 * @retval      返回一个周期的占空比计数值
 */
uint32_t ledc_duty_pow(uint32_t duty, uint8_t m, uint8_t n)
{
    uint32_t result = 1;

    while (n--)
    {
        result *= m;
    }

    return (result * duty) / 100;
}

/**
 * @brief       初始化LEDC
 * @param       ledc_config: ledc配置结构体
 * @retval      无
 */
void ledc_init(ledc_config_t *ledc_config)
{
    ledc_config->duty = ledc_duty_pow(ledc_config->duty, 2, ledc_config->duty_resolution);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,            /* 低速模式(ESP32P4仅支持低速模式) */
        .duty_resolution  = ledc_config->duty_resolution,   /* 占空比分辨率 */
        .timer_num        = ledc_config->timer_num,         /* 定时器选择 */
        .freq_hz          = ledc_config->freq_hz,           /* 设置频率 */
        .clk_cfg          = ledc_config->clk_cfg            /* 设置时钟源 */
    };
    /* 配置ledc定时器 */
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    /* 配置pwm通道 */
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,      /* 低速模式 */
        .channel        = ledc_config->channel,     /* PWM输出通道 */
        .timer_sel      = ledc_config->timer_num,   /* 那个定时器提供计数值 */
        .intr_type      = LEDC_INTR_DISABLE,        /* 关闭LEDC中断 */
        .gpio_num       = ledc_config->gpio_num,    /* 输出GPIO管脚 */
        .duty           = ledc_config->duty,        /* 占空比 */
        .hpoint         = 0                         /* 设置hpoint数值 */
    };
    /* Lpoint = duty + hpoint */

    /* 配置pwm通道 */
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

/**
 * @brief       设置占空比
 * @param       ledc_config: ledc配置结构体
 *              duty: 占空比
 * @retval      无
 */
void ledc_pwm_set_duty(ledc_config_t *ledc_config, uint16_t duty)
{
    ledc_config->duty = ledc_duty_pow(duty, 2, ledc_config->duty_resolution);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ledc_config->channel, ledc_config->duty);    /* 设置占空比 */
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ledc_config->channel);                    /* 更新占空比 */
}
