#ifndef PULSE_COUNTER_H
#define PULSE_COUNTER_H

#include <stdint.h>

// 编码器参数配置
typedef struct {
    float pulses_per_mm;  // 每毫米脉冲数
} encoder_config_t;

// 初始化脉冲计数器
// gpio_a - A相引脚号
// gpio_b - B相引脚号
void pulse_counter_init(int gpio_a, int gpio_b);

// 获取累计脉冲数
int64_t pulse_counter_get_count(void);

// 设置编码器参数
void pulse_counter_set_config(encoder_config_t config);

// 获取累计脉冲数转换为毫米距离（精确到小数点后4位）
double pulse_counter_get_distance_mm(void);

// 复位脉冲计数器
void pulse_counter_reset(void);

#endif // PULSE_COUNTER_H
