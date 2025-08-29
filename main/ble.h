#ifndef _BLE_APP_H_
#define _BLE_APP_H_
#include "esp_err.h"

/**
 * 初始化并启动蓝牙BLE
 * @param 无
 * @return 是否成功
 */
esp_err_t ble_cfg_net_init(void);


/**
 * 设置特征2的值
 * @param value 值
 * @return 无
 */
void ble_set_ch2_value(uint16_t value);

#endif
