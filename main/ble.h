#ifndef _BLE_APP_H_
#define _BLE_APP_H_
#include "esp_err.h"

/**
 * 初始化并启动蓝牙BLE
 * @param 无
 * @return 是否成功
 */
esp_err_t ble_cfg_net_init(void);


void ble_send_ch2_data(const uint8_t *data, uint8_t len);

#endif
