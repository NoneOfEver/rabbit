#include "pulse_counter.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "math.h"

static const char *TAG = "pulse_counter";

// 脉冲计数器句柄
static pcnt_unit_handle_t pcnt_unit = NULL;

// 编码器配置
static encoder_config_t s_encoder_config = {
    .pulses_per_mm = 1.0f // 默认值
};

void pulse_counter_init(int gpio_a, int gpio_b)
{
    ESP_LOGI(TAG, "初始化脉冲计数器");

    // 配置计数器单元 - 使用合理的计数范围
    pcnt_unit_config_t unit_config = {
        .high_limit = 10000,
        .low_limit = -10000,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    // 配置通道A
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = gpio_a,
        .level_gpio_num = gpio_b,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    
    // 配置通道B
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = gpio_b,
        .level_gpio_num = gpio_a,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    // 设置通道动作
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // 启用计数器
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

int64_t pulse_counter_get_count(void)
{
    int pulse_count = 0;
    // 获取当前计数值
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
    return pulse_count;
}

void pulse_counter_set_config(encoder_config_t config)
{
    s_encoder_config = config;
}

double pulse_counter_get_distance_mm(void)
{
    int64_t pulses = pulse_counter_get_count();
    double cycles = (double)pulses / 4.0;
    double distance = cycles / s_encoder_config.pulses_per_mm;
    return distance;
}

void pulse_counter_reset(void)
{
    if (pcnt_unit) {
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    }
}
