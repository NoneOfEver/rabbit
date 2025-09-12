#include "cmd_storage.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

#define STORAGE_NAMESPACE "storage"

static const char *TAG = "cmd_storage";

/**
 * @brief 初始化 NVS 和命令存储
 */
esp_err_t cmd_storage_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) return err;

    // 确保 cmd_count 存在
    nvs_handle_t handle;
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    uint32_t count = 0;
    if (nvs_get_u32(handle, "cmd_count", &count) == ESP_ERR_NVS_NOT_FOUND) {
        nvs_set_u32(handle, "cmd_count", 0);
        nvs_commit(handle);
    }

    nvs_close(handle);
    return ESP_OK;
}

/**
 * @brief 保存命令数组和数量
 */
static esp_err_t save_cmds(cmd_t *cmds, size_t count)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(handle, "cmds", cmds, count * sizeof(cmd_t));
    if (err != ESP_OK) {
        nvs_close(handle);
        return err;
    }

    err = nvs_set_u32(handle, "cmd_count", count);
    if (err != ESP_OK) {
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

/**
 * @brief 加载命令数组
 */
static size_t load_cmds(cmd_t *cmds, size_t max_count)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) return 0;

    uint32_t count = 0;
    if (nvs_get_u32(handle, "cmd_count", &count) != ESP_OK) {
        nvs_close(handle);
        return 0;
    }
    if (count > max_count) count = max_count;

    size_t size = count * sizeof(cmd_t);
    err = nvs_get_blob(handle, "cmds", cmds, &size);
    if (err != ESP_OK) {
        nvs_close(handle);
        return 0;
    }

    nvs_close(handle);
    return count;
}

/**
 * @brief 添加一批命令
 */
esp_err_t cmd_storage_add(const uint8_t *data, size_t len)
{
    if (len % CMD_SIZE != 0) {
        ESP_LOGE(TAG, "Invalid command length");
        return ESP_ERR_INVALID_ARG;
    }

    cmd_t cmds[MAX_CMDS];
    size_t count = load_cmds(cmds, MAX_CMDS);

    size_t new_cmds = len / CMD_SIZE;
    if (count + new_cmds > MAX_CMDS) {
        ESP_LOGE(TAG, "Command buffer overflow");
        return ESP_ERR_NO_MEM;
    }

    for (size_t i = 0; i < new_cmds; i++) {
        memcpy(cmds[count + i].command, data + i * CMD_SIZE, CMD_SIZE);
    }

    return save_cmds(cmds, count + new_cmds);
}

/**
 * @brief 获取命令数组
 */
size_t cmd_storage_get(cmd_t *cmds, size_t max_count)
{
    return load_cmds(cmds, max_count);
}

/**
 * @brief 删除第一个命令
 */
esp_err_t cmd_storage_pop(void)
{
    cmd_t cmds[MAX_CMDS];
    size_t count = load_cmds(cmds, MAX_CMDS);

    if (count == 0) return ESP_OK;

    for (size_t i = 1; i < count; i++) {
        cmds[i - 1] = cmds[i];
    }
    count--;

    return save_cmds(cmds, count);
}

/**
 * @brief 清空所有命令
 */
esp_err_t cmd_storage_clear(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    nvs_erase_key(handle, "cmds");
    nvs_erase_key(handle, "cmd_count");

    err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}
