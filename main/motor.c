#include "freertos/FreeRTOS.h"
#include <math.h>
#include <stdbool.h>  // 添加bool类型支持
#include <string.h>
#include "pulse_counter.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "motor.h"
#include "esp_log.h"
#include "rgb_led.h"

#define TAG "POS_CTRL"

#define PIN_PWM_A   18
#define PIN_PWM_B   8
#define PIN_EN      7

#define RE_A_GPIO   16
#define RE_B_GPIO   17
#define RE_BTN_GPIO 5  // 不使用按钮

// ---------------- 电机 & 编码器参数 ----------------
#define PPR         7
#define GEAR_RATIO  298
#define SCREW_PITCH_MM 0.7f

// 计算每毫米物理脉冲数（未倍频）
#define PHYSICAL_PULSES_PER_MM ((PPR * GEAR_RATIO) / SCREW_PITCH_MM)

// 注意：新的脉冲计数器模块已做4倍频计数，所以配置参数时直接使用物理脉冲数
#define PULSES_PER_MM PHYSICAL_PULSES_PER_MM
// ---------------- PWM 配置 ----------------
#define PWM_FREQ    20000
#define PWM_RES     LEDC_TIMER_10_BIT
#define PWM_MAX     ((1<<PWM_RES)-1)
#define PWM_CH_A    LEDC_CHANNEL_0
#define PWM_CH_B    LEDC_CHANNEL_1
#define PWM_TIMER   LEDC_TIMER_0
// ---------------- PID 参数 ----------------
#define KP  2.6f // 2.6
#define KI  1.3f  // 0.3
#define KD  0.01f // 0.0

// ---------------- motor ----------------
static float target_pos_mm = 0.0f;
static int32_t encoder_count = 0;
static bool brake_requested = false;  // 刹车请求标志
// ---------------- PID ----------------
static float integral = 0;
static float last_error = 0;

// 前置声明
void set_motor_target(float delta);

// 前置声明motor_brake函数
void motor_brake(void);

void motor_init()
{
    // PWM
    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = PWM_TIMER,
        .duty_resolution  = PWM_RES,
        .freq_hz          = PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ch_conf = {
        .gpio_num   = PIN_PWM_A,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = PWM_CH_A,
        .timer_sel  = PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ch_conf);

    ch_conf.gpio_num = PIN_PWM_B;
    ch_conf.channel  = PWM_CH_B;
    ledc_channel_config(&ch_conf);

    gpio_reset_pin(PIN_EN);
    gpio_set_direction(PIN_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_EN, 1);

    // 初始化脉冲计数器（AB相编码器）
    pulse_counter_init(RE_A_GPIO, RE_B_GPIO);

    // 设置编码器参数
    encoder_config_t config = {
        .pulses_per_mm = PULSES_PER_MM
    };
    pulse_counter_set_config(config);
    
    // 初始化后立即刹车
    motor_brake();
}

// ---------------- PID ----------------
float pid_update(float target, float current, float dt)
{
    float error = target - current;

    // 当误差在目标值的 0.2% 内时，直接输出 0
    if (fabs(error) <= 0.01f * fabs(target))
    {
        integral = 0;       // 可选：清零积分，防止积分累积
        last_error = 0;     // 更新 last_error 避免跳变
        return 0.0f;
    }

    integral += error * dt;
    float derivative = (error - last_error) / dt;
    last_error = error;

    float output = KP * error + KI * integral + KD * derivative;

    if (output > 1.0f) output = 1.0f;
    if (output < -1.0f) output = -1.0f;
    return output;
}

void motor_brake(void)
{
    // 同时拉高两个PWM引脚（刹车）
    ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_A, PWM_MAX);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CH_A);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_B, PWM_MAX);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CH_B);
    brake_requested = true;
}

// 设置电机目标位置的增量（单位：mm）
void set_motor_target(float delta)
{
    // 计算新目标位置
    float new_target = target_pos_mm + delta;
    
    // // 限制目标位置在-1.0mm到1.0mm之间
    // if (new_target > 1.0f) {
    //     new_target = 1.0f;
    // } else if (new_target < -1.0f) {
    //     new_target = -1.0f;
    // }
    
    target_pos_mm = new_target;
    
    // 开始运动前解除刹车
    if (brake_requested) {
        // 解除刹车：设置两个PWM引脚为低电平
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_A, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CH_A);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_B, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CH_B);
        brake_requested = false;
    }
}

// ---------------- 电机输出 ----------------
void motor_set_output(float val)
{
    int pwm_val = (int)(fabs(val) * PWM_MAX);
    if (pwm_val > PWM_MAX) pwm_val = PWM_MAX;

    if (val > 0)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_A, pwm_val);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CH_A);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_B, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CH_B);
    }
    else
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_A, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CH_A);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_B, pwm_val);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CH_B);
    }
}


#define MAX_STEP_MM 0.7f   // 单次最大移动距离

static float step_mm = 0.0f;   // 每次步进量
static SemaphoreHandle_t step_mutex;

void control_task(void *arg)
{
    TickType_t last_tick = xTaskGetTickCount();

    while (1)
    {
        float pos_mm = pulse_counter_get_distance_mm();

        TickType_t now = xTaskGetTickCount();
        float dt = (now - last_tick) * portTICK_PERIOD_MS / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        last_tick = now;

        // 限制单次目标不超过 MAX_STEP_MM
        float segment_target = target_pos_mm;
        if (target_pos_mm >  MAX_STEP_MM) segment_target =  MAX_STEP_MM;
        if (target_pos_mm < -MAX_STEP_MM) segment_target = -MAX_STEP_MM;

        float u = pid_update(segment_target, pos_mm, dt);

        // 检查是否到达目标位置并触发刹车
        if (u == 0.0f && !brake_requested) {
            motor_brake();
            // 重置编码器计数和位置
            pulse_counter_reset();
            // 从总目标里减去已完成的段
            target_pos_mm -= segment_target;
            // 如果还有剩余距离，就准备下一段
            if (target_pos_mm != 0.0f) {
                brake_requested = false;   // <-- 关键：解除刹车，允许继续运动
            }
        }

        // 仅在未刹车时更新电机输出
        if (!brake_requested) {
            motor_set_output(u);
        }

        // 每100ms打印一次信息
        static TickType_t last_log_time = 0;
        if (now - last_log_time >= pdMS_TO_TICKS(100)) {
            ESP_LOGI(TAG, "剩余目标: %.2f mm | 当前: %.4f mm | PWM: %.2f | 刹车: %s",
                     target_pos_mm, pos_mm, u, brake_requested ? "是" : "否");
            last_log_time = now;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
