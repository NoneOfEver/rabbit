#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>
#include "encoder.h"
#include "driver/uart.h"
#include <ds3231.h>
#include <led_strip.h>
#include <led_strip_rmt.h>
#include "nvs_flash.h"
#include "ble.h"

#define TAG "POS_CTRL"

// ---------------- GPIO ----------------
#define RE_A_GPIO   16
#define RE_B_GPIO   17
#define RE_BTN_GPIO 5  // 不使用按钮

#define PIN_PWM_A   8
#define PIN_PWM_B   9
#define PIN_EN      12

#define CONFIG_EXAMPLE_I2C_MASTER_SDA 15
#define CONFIG_EXAMPLE_I2C_MASTER_SCL 14
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

// ---------------- UART ----------------
#define UART_PORT   UART_NUM_0
#define BUF_SIZE    128

// ---------------- PID 参数 ----------------
#define KP  2.72f // 2.6
#define KI  3.0f // 0.3
#define KD  0.01f // 0.0

// ---------------- RGB LED ----------------
// Set to 1 to use DMA for driving the LED strip, 0 otherwise
// Please note the RMT DMA feature is only available on chips e.g. ESP32-S3/P4
#define LED_STRIP_USE_DMA  0

#if LED_STRIP_USE_DMA
// Numbers of the LED in the strip
#define LED_STRIP_LED_COUNT 1
#define LED_STRIP_MEMORY_BLOCK_WORDS 1024 // this determines the DMA block size
#else
// Numbers of the LED in the strip
#define LED_STRIP_LED_COUNT 1
#define LED_STRIP_MEMORY_BLOCK_WORDS 0 // let the driver choose a proper memory block size automatically
#endif // LED_STRIP_USE_DMA

// GPIO assignment
#define LED_STRIP_GPIO_PIN  48

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

// ---------------- BLE ----------------


// ---------------- 全局变量 ----------------
// ---------------- motro ----------------
static float target_pos_mm = 0.2f;
static QueueHandle_t event_queue;
static rotary_encoder_t re;
static int32_t encoder_count = 0;
// ---------------- PID ----------------
static float integral = 0;
static float last_error = 0;

// ---------------- RGB LED ----------------
static const char *TAG_LED = "LED";

// ---------------- BLE ----------------


void ds3231_task(void *pvParameters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(ds3231_init_desc(&dev, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // setup datetime: 2016-10-09 13:50:10
    struct tm time =
    {
        .tm_year = 125, //since 1900 (2016 - 1900)
        .tm_mon  = 8-1,  // 0-based
        .tm_mday = 20,
        .tm_hour = 22,
        .tm_min  = 17,
        .tm_sec  = 10
    };
    ESP_ERROR_CHECK(ds3231_set_time(&dev, &time));

    while (1)
    {
        float temp;

        vTaskDelay(pdMS_TO_TICKS(250));

        if (ds3231_get_temp_float(&dev, &temp) != ESP_OK)
        {
            printf("Could not get temperature\n");
            continue;
        }

        if (ds3231_get_time(&dev, &time) != ESP_OK)
        {
            printf("Could not get time\n");
            continue;
        }

        /* float is used in printf(). you need non-default configuration in
         * sdkconfig for ESP8266, which is enabled by default for this
         * example. see sdkconfig.defaults.esp8266
         */
        printf("%04d-%02d-%02d %02d:%02d:%02d, %.2f deg Cel\n", time.tm_year + 1900 /*Add 1900 for better readability*/, time.tm_mon + 1,
               time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec, temp);
    }
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

// ---------------- UART 命令 ----------------
void uart_cmd_task(void *arg)
{
    uint8_t data[BUF_SIZE];
    while (1)
    {
        int len = uart_read_bytes(UART_PORT, data, BUF_SIZE - 1, pdMS_TO_TICKS(10));
        if (len > 0)
        {
            data[len] = '\0';
            float new_target = atof((char*)data);
            target_pos_mm = new_target;
            ESP_LOGW(TAG, "New Target Position: %.2f mm", target_pos_mm);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ---------------- LED ----------------
led_strip_handle_t configure_led(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN, // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_COUNT,      // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,        // LED strip model
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .mem_block_symbols = LED_STRIP_MEMORY_BLOCK_WORDS, // the memory block size used by the RMT channel
        .flags = {
            .with_dma = LED_STRIP_USE_DMA,     // Using DMA can improve performance when driving more LEDs
        }
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG_LED, "Created LED strip object with RMT backend");
    return led_strip;
}

void led_task(void *arg)
{
    led_strip_handle_t led_strip = configure_led();
    bool led_on_off = false;

    ESP_LOGI(TAG_LED, "Start blinking LED strip");
    while (1) {
        if (led_on_off) {
            /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
            for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 5, 5, 5));
            }
            /* Refresh the strip to send data */
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            //ESP_LOGI(TAG_LED, "LED ON!");
        } else {
            /* Set all LED off to clear all pixels */
            ESP_ERROR_CHECK(led_strip_clear(led_strip));
            //ESP_LOGI(TAG_LED, "LED OFF!");
        }

        led_on_off = !led_on_off;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ---------------- BLE ----------------
void ble_task(void* param)
{
    uint16_t count1 = 1;
    uint16_t count2 = 100;
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(3000));
        // ble_set_ch1_value(count1++);
        vTaskDelay(pdMS_TO_TICKS(700));
        ble_set_ch2_value(count2++);
    }
}


// ---------------- app_main ----------------
void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

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

    // UART
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);

    ESP_ERROR_CHECK(i2cdev_init());

    ble_cfg_net_init();

    // 启动任务
    xTaskCreate(control_task, "control_task", 4096, NULL, 5, NULL);
    xTaskCreate(uart_cmd_task, "uart_cmd_task", 2048, NULL, 4, NULL);
    xTaskCreate(ds3231_task, "ds3231_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(led_task, "led_task", 2048, NULL, 3, NULL);
    xTaskCreatePinnedToCore(ble_task,"ble_task",6144,NULL,3,NULL,1);
}
