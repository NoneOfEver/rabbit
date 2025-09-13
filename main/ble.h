#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "nvs_flash.h"
#include "rgb_led.h"

#define TAG "BLE_CFG"

//设备名称
#define BLE_DEVICE_NAME     "RABBIT"

#define ESP_APP_ID                  0x55

#define SVC_IND_ID1      0

typedef enum {
    BLE_EVENT_SHUTDOWN = 1,
} ble_event_t;

// 全局变量 (在 ble.c 中定义)
extern uint8_t g_ble_recive_flag;  // 注意：保持为 uint8_t 类型
extern uint8_t sv1_char1_value[20];
extern uint8_t sv1_char1_value_len;

// 蓝牙超时时间（毫秒）
#define BLE_TIMEOUT_MS 60000

// 蓝牙超时状态标志
extern bool g_ble_timeout_flag;       // 超时标志
extern bool g_ble_connected;          // 连接状态
extern TimerHandle_t ble_timeout_timer; // 超时计时器
extern QueueHandle_t ble_event_queue;
// 函数声明
esp_err_t ble_cfg_net_init(void);
void ble_send_ch2_data(const uint8_t *data, uint8_t len);
