#ifndef MAIN_BLE_H_
#define MAIN_BLE_H_
#include "esp_err.h"
#include <stdint.h>

extern uint8_t sv1_char1_value[20];
extern uint8_t sv1_char1_value_len;
extern uint8_t g_ble_recive_flag; // 标志位，表示是否收到数据
/**
 * 初始化并启动蓝牙BLE
 * @param 无
 * @return 是否成功
 */
esp_err_t ble_cfg_net_init(void);

void ble_send_ch2_data(const uint8_t *data, uint8_t len);


#endif // !MAIN_BLE_H_
