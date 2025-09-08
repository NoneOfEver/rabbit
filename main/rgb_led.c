#include <led_strip.h>
#include <led_strip_rmt.h>
#include "esp_log.h"
#include "rgb_led.h"

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
// ---------------- RGB LED ----------------
static const char *TAG_LED = "LED";
bool led_on_off = false;
LedState led_state;
led_strip_handle_t led_strip;
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

void led_init()
{
    led_strip = configure_led();
    ESP_LOGI(TAG_LED, "Start blinking LED strip");
}

void set_led_state(LedState state)
{
    led_state = state;
}

void led_task(void *arg)
{
    static bool blink_state = false; // 用于闪烁状态
    
    while (1) {
        switch (led_state)
        {
            case LED_RED:
                // 不在工作 - 红色
                for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 255, 0, 0));
                }
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                blink_state = false; // 重置闪烁状态
                break;
            case LED_GREEN:
                // 在工作 - 绿色（常亮）
                for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 255, 0));
                }
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                blink_state = false; // 重置闪烁状态
                break;
            case LED_BLUE_BLINK:
                // 连接蓝牙 - 蓝色（闪烁）
                if (blink_state) {
                    for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
                        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 255, 10));
                    }
                } else {
                    ESP_ERROR_CHECK(led_strip_clear(led_strip));
                }
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                blink_state = !blink_state; // 切换闪烁状态
                break;
            case LED_GREEN_BLINK:
                // 在运动 - 绿色（闪烁）
                if (blink_state) {
                    for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
                        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 255, 0));
                    }
                } else {
                    ESP_ERROR_CHECK(led_strip_clear(led_strip));
                }
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                blink_state = !blink_state; // 切换闪烁状态
                break;
            case LED_OFF:
                // 休眠 - 关闭所有LED
                ESP_ERROR_CHECK(led_strip_clear(led_strip));
                blink_state = false; // 重置闪烁状态
                break;
            default:
                ESP_ERROR_CHECK(led_strip_clear(led_strip));
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
