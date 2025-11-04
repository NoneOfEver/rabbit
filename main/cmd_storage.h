#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CMD_SIZE 10
#define MAX_CMDS 120   // 可根据需要调整最大命令数

typedef struct {
    uint8_t command[CMD_SIZE];
} cmd_t;

/**
 * @brief 初始化命令存储系统（必须先调用一次）
 */
esp_err_t cmd_storage_init(void);

/**
 * @brief 向存储中追加一批命令
 * @param data 原始命令数据，长度必须是 CMD_SIZE 的倍数
 * @param len  数据长度（字节）
 */
esp_err_t cmd_storage_add(const uint8_t *data, size_t len);

/**
 * @brief 获取所有命令
 * @param cmds 缓冲区
 * @param max_count 缓冲区可存放的最大命令数
 * @return 实际读取到的命令数量
 */
size_t cmd_storage_get(cmd_t *cmds, size_t max_count);

/**
 * @brief 弹出（删除）第一个命令
 */
esp_err_t cmd_storage_pop(void);

/**
 * @brief 清空所有命令
 */
esp_err_t cmd_storage_clear(void);

#ifdef __cplusplus
}
#endif
