#include "freertos/FreeRTOS.h"
#include <math.h>
#include <string.h>
#include "encoder.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "rgb_led.h"
#include "motor.h"
#include "esp_log.h"

#define TAG "POS_CTRL"

#define PIN_PWM_A   8
#define PIN_PWM_B   9
#define PIN_EN      7

#define RE_A_GPIO   16
#define RE_B_GPIO   17
#define RE_BTN_GPIO 5  // 不使用按钮

// ---------------- 电机 & 编码器参数 ----------------
#define PPR         7
#define GEAR_RATIO  298
#define SCREW_PITCH_MM 0.7f
#define COUNTS_PER_REV (PPR)
#define MM_PER_REV (SCREW_PITCH_MM)
#define MM_PER_COUNT (MM_PER_REV / (COUNTS_PER_REV * GEAR_RATIO))
// ---------------- PWM 配置 ----------------
#define PWM_FREQ    20000
#define PWM_RES     LEDC_TIMER_10_BIT
#define PWM_MAX     ((1<<PWM_RES)-1)
#define PWM_CH_A    LEDC_CHANNEL_0
#define PWM_CH_B    LEDC_CHANNEL_1
#define PWM_TIMER   LEDC_TIMER_0
// ---------------- PID 参数 ----------------
#define KP  2.72f // 2.6
#define KI  3.0f  // 0.3
#define KD  0.01f // 0.0

// ---------------- motor ----------------
static float target_pos_mm = 0.0f;
static QueueHandle_t event_queue;
static rotary_encoder_t re;
static int32_t encoder_count = 0;
// ---------------- PID ----------------
static float integral = 0;
static float last_error = 0;


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

    // 编码器
    event_queue = xQueueCreate(10, sizeof(rotary_encoder_event_t));
    ESP_ERROR_CHECK(rotary_encoder_init(event_queue));
    memset(&re, 0, sizeof(rotary_encoder_t));
    re.pin_a = RE_A_GPIO;
    re.pin_b = RE_B_GPIO;
    re.pin_btn = RE_BTN_GPIO;
    ESP_ERROR_CHECK(rotary_encoder_add(&re));

}
// ---------------- PID ----------------
float pid_update(float target, float current, float dt)
{
    float error = target - current;

    // 当误差在目标值的 0.02% 内时，直接输出 0
    if (fabs(error) <= 0.0002f * fabs(target))
    {
        integral = 0;       // 可选：清零积分，防止积分累积
        last_error = 0;     // 更新 last_error 避免跳变
        set_led_state(LED_GREEN);
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

void set_motor_target(float target)
{
    target_pos_mm = target;
}

// ---------------- 电机输出 ----------------
void motor_set_output(float val)
{
    
    int pwm_val = (int)(fabs(val) * PWM_MAX);
    if (pwm_val > PWM_MAX) pwm_val = PWM_MAX;

    if (pwm_val > 1)
    {
        set_led_state(LED_GREEN_BLINK);
    }else{
        set_led_state(LED_GREEN);
    }

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

// ---------------- 控制任务 ----------------
static float step_mm = 0.0f;   // 每次步进量
static SemaphoreHandle_t step_mutex;
void control_task(void *arg)
{
    rotary_encoder_event_t e;
    TickType_t last_tick = xTaskGetTickCount();

    while (1)
    {
        while (xQueueReceive(event_queue, &e, 0) == pdTRUE)
        {
            if (e.type == RE_ET_CHANGED)
                encoder_count += e.diff;
        }

        float pos_mm = encoder_count * MM_PER_COUNT;

        TickType_t now = xTaskGetTickCount();
        float dt = (now - last_tick) * portTICK_PERIOD_MS / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        last_tick = now;

        float u = pid_update(target_pos_mm, pos_mm, dt);
        motor_set_output(u);

        // ESP_LOGI(TAG, "Target: %.2f mm | Pos: %.4f mm | PWM: %.2f",
        //          target_pos_mm, pos_mm, u);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
