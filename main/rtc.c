#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include <ds3231.h>
#include "i2cdev.h"
#include "rtc.h"
// ---------------- GPIO ----------------
#define CONFIG_EXAMPLE_I2C_MASTER_SDA 15
#define CONFIG_EXAMPLE_I2C_MASTER_SCL 14

static i2c_dev_t dev;  // 静态全局变量，只初始化一次
static bool rtc_initialized = false;

void my_rtc_init() {
    if (!rtc_initialized) {
        ESP_ERROR_CHECK(i2cdev_init());
        memset(&dev, 0, sizeof(i2c_dev_t));
        ESP_ERROR_CHECK(ds3231_init_desc(&dev, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
        rtc_initialized = true;
    }
}

void rtc_get_time(struct tm *time)
{
    if (!rtc_initialized) {
        my_rtc_init();
    }
    ESP_ERROR_CHECK(ds3231_get_time(&dev, time));
}

// 设置RTC时间
void rtc_set_time(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute)
{
    struct tm new_time = {
        .tm_year = year + 100,  // 年份偏移（2000年为基准）
        .tm_mon = month - 1,    // 月份范围0-11
        .tm_mday = day,
        .tm_hour = hour,
        .tm_min = minute,
        .tm_sec = 0
    };
    
    ESP_ERROR_CHECK(ds3231_set_time(&dev, &new_time));
    ESP_LOGI("RTC", "时间已更新: 20%02d-%02d-%02d %02d:%02d", 
             year, month, day, hour, minute);
}

void ds3231_init()
{
    // 现在由rtc_init统一处理初始化
    my_rtc_init();
}
void ds3231_task(void *pvParameters)
{
    // ESP_ERROR_CHECK(i2cdev_init());

    // i2c_dev_t dev;
    // memset(&dev, 0, sizeof(i2c_dev_t));

    // ESP_ERROR_CHECK(ds3231_init_desc(&dev, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // setup datetime: 2016-10-09 13:50:10
    struct tm time ;
    // {
    //     .tm_year = 125, //since 1900 (2016 - 1900)
    //     .tm_mon  = 9-1,  // 0-based
    //     .tm_mday = 7,
    //     .tm_hour = 5,
    //     .tm_min  = 27,
    //     .tm_sec  = 10
    // };
    //ESP_ERROR_CHECK(ds3231_set_time(&dev, &time));

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
