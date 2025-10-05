#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdio.h>

#include "esp_log.h"
#include <led_strip.h>
#include <led_strip_rmt.h>
#include "nvs_flash.h"
#include "ble.h"

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "motor.h"
#include "rgb_led.h"
#include "rtc.h"
#include "sd_card.h"
#include "pulse_counter.h"
#include "cmd_storage.h"
#include <math.h>

#define TAG "MAIN"
// 创建文件路径
char file_path[] = MOUNT_POINT "/LOG.txt";
// 十六进制打印函数
void hex_dump(const uint8_t *data, size_t data_len) {
    for (int i = 0; i < data_len; i++) {
        ESP_LOGD(TAG, "%02X ", data[i]);
    }
    ESP_LOGD(TAG, "\n");
}

// 电机参数（使用一套定义）
#define PPR                 7
#define GEAR_RATIO          298
#define SCREW_PITCH_MM      0.7f

// 计算每毫米脉冲数（宏观定义）
#define PULSES_PER_MM ((PPR * GEAR_RATIO) / SCREW_PITCH_MM)

// 编码器引脚定义
#define ENCODER_A_GPIO      16
#define ENCODER_B_GPIO      17

// ---------------- BLE ----------------
// ------------------ 命令结构定义 ------------------
typedef struct {
    uint16_t frame_header;   // 帧头 (2字节)
    uint16_t time_minutes;   // 时间 (2字节，分钟数)
    uint8_t  movement_direction; // 运动方向 (1字节, 0:正向, 1:反向)
    uint16_t movement_length;// 运动长度 (2字节，0.01mm为单位)
    uint8_t  movement_days;   // 运动天数 (1字节)
    uint16_t frame_footer;   // 帧尾 (2字节)
} BleCommand;

// ------------------ 链表节点定义 ------------------
typedef struct CommandNode {
    BleCommand command;          // 命令数据
    struct tm start_date;        // 命令接收的日期
    struct CommandNode* next;    // 指向下一个节点的指针
} CommandNode;

// ------------------ 全局变量 ------------------
static CommandNode* command_list_head = NULL;
static SemaphoreHandle_t command_list_mutex = NULL;

// ------------------ 链表操作函数 ------------------

// 初始化命令链表
void init_command_list() 
{
    command_list_mutex = xSemaphoreCreateMutex();
    command_list_head = NULL;
}

// 添加命令到链表尾部
void add_command_to_list(BleCommand new_command) 
{
    CommandNode* new_node = malloc(sizeof(CommandNode));
    if (!new_node) {
        ESP_LOGE(TAG, "内存分配失败");
        return;
    }
    
    // 设置命令接收日期
    rtc_get_time(&new_node->start_date);  // 存储接收命令时的日期
    
    new_node->command = new_command;
    new_node->next = NULL;
    
    // 获取互斥锁
    if (xSemaphoreTake(command_list_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (command_list_head == NULL) {
            command_list_head = new_node;
        } else {
            CommandNode* current = command_list_head;
            while (current->next != NULL) {
                current = current->next;
            }
            current->next = new_node;
        }
        xSemaphoreGive(command_list_mutex);
    }
}

// 从链表头部取出命令节点
CommandNode* get_next_command_node() 
{
    CommandNode* node = NULL;
    
    if (xSemaphoreTake(command_list_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (command_list_head != NULL) {
            node = command_list_head;
            command_list_head = node->next;
            node->next = NULL; // 从链表中移除
        }
        xSemaphoreGive(command_list_mutex);
    }
    
    return node;
}

// 清空命令链表
void clear_command_list() 
{
    if (xSemaphoreTake(command_list_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        CommandNode* current = command_list_head;
        while (current != NULL) {
            CommandNode* next = current->next;
            free(current);
            current = next;
        }
        command_list_head = NULL;
        xSemaphoreGive(command_list_mutex);
        ESP_LOGI(TAG, "命令列表已清空");
    }
}

// 检查链表是否为空
bool is_command_list_empty() 
{
    bool empty = true;
    if (xSemaphoreTake(command_list_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        empty = (command_list_head == NULL);
        xSemaphoreGive(command_list_mutex);
    }
    return empty;
}

// 将分钟数转换为"小时:分钟"格式的字符串
static void minutes_to_time_str(uint16_t minutes, char* buf, size_t buf_size) 
{
    uint16_t hours = minutes / 60;
    uint16_t mins = minutes % 60;
    snprintf(buf, buf_size, "%02d:%02d", hours, mins);
}

// 命令执行任务函数声明
void command_execution_task(void *param);

// 解析BLE命令
// 解析时间更新命令 (5字节: YY MM DD HH MM)
static void parse_time_command(uint8_t* data) 
{
    uint8_t year = data[0];
    uint8_t month = data[1];
    uint8_t day = data[2];
    uint8_t hour = data[3];
    uint8_t minute = data[4];
    
    rtc_set_time(year, month, day, hour, minute);
    ESP_LOGI(TAG, "RTC时间已更新: 20%02d-%02d-%02d %02d:%02d", 
             year, month, day, hour, minute);
}

void parse_ble_command(uint8_t* data, size_t data_len) 
{
    const size_t COMMAND_SIZE = 10;
    const size_t TIME_COMMAND_SIZE = 5;
    
    // 记录接收到的BLE数据
    ESP_LOGI(TAG, "解析BLE命令，数据长度: %d", data_len);
    hex_dump(data, data_len);
    
    // 检查是否为时间更新命令
    if (data_len == TIME_COMMAND_SIZE) {
        parse_time_command(data);
        return;
    }
    
    size_t index = 0;
    
    while (index < data_len) {
        
        // 检查剩余数据是否足够一个命令
        if (index + COMMAND_SIZE > data_len) {
            break;
        }
        
        uint8_t* cmd_data = data + index;
        index += COMMAND_SIZE;
        
        // 提取命令数据（大端序）
        BleCommand new_cmd = {
            .frame_header = (cmd_data[0] << 8) | cmd_data[1],  // 大端序
            .time_minutes = (cmd_data[2] << 8) | cmd_data[3],  // 大端序
            .movement_direction = cmd_data[4],
            .movement_length = (cmd_data[5] << 8) | cmd_data[6],  // 大端序
            .movement_days = cmd_data[7],
            .frame_footer = (cmd_data[8] << 8) | cmd_data[9]   // 大端序
        };
        
        // 验证帧头帧尾
        if (new_cmd.frame_header != 0xAA55 || new_cmd.frame_footer != 0x55AA) {
            ESP_LOGW(TAG, "无效帧标记: 头=0x%04X, 尾=0x%04X", 
                    new_cmd.frame_header, new_cmd.frame_footer);
            continue;
        }
        
        // 添加任务到链表
        add_command_to_list(new_cmd);
        float actual_length_mm = (float)new_cmd.movement_length * 0.01f;
        char time_str[16];
        minutes_to_time_str(new_cmd.time_minutes, time_str, sizeof(time_str));
        ESP_LOGI(TAG, "添加命令: 时间=%s(%u分钟), 方向=%s, 长度=%.2fmm, 天数=%u", 
                 time_str, new_cmd.time_minutes,
                 new_cmd.movement_direction ? "反向" : "正向",
                 actual_length_mm,
                 new_cmd.movement_days);
    }
}

// ble接收处理任务
void ble_task(void* param)
{
    static uint8_t nvs_has_ben_cleard = 0; // 加一个 清除过 标志位，防止反复清除nvs
    init_command_list(); // 初始化链表
    
    while(1)
    {
        // 接收任务
        if(g_ble_recive_flag == 1)
        {
            g_ble_recive_flag = 0;

            if(nvs_has_ben_cleard != 1){ // 只在第一次接收时清除nvs
                // 清空nvs中之前的任务组
                cmd_storage_clear();
                // 清空已有任务链表
                clear_command_list();
                nvs_has_ben_cleard = 1; // 表示已经清除过
            }
            if(sv1_char1_value_len == CMD_SIZE){
                // 添加任务数据到nvs
                cmd_storage_add(sv1_char1_value,sv1_char1_value_len);
            }
            
            // 获取BLE数据长度（在ble.h中定义了sv1_char1_value_len）
            parse_ble_command(sv1_char1_value, sv1_char1_value_len);
        }

        // 处理命令
        if (!is_command_list_empty()) {
            CommandNode *node = get_next_command_node();
            if (node == NULL) {
                continue;
            }
            float actual_length_mm = (float)node->command.movement_length * 0.01f;
            char time_str[16];
            minutes_to_time_str(node->command.time_minutes, time_str, sizeof(time_str));
            ESP_LOGI(TAG, "调度命令: 时间=%s(%u分钟), 运动长度=%.2fmm, 天数=%u", 
                     time_str, node->command.time_minutes, fabs(actual_length_mm), node->command.movement_days);
            
            // 创建任务执行命令，传递节点（节点将在任务中释放）
            if (xTaskCreate(command_execution_task, "cmd_exec", 3072, node, 5, NULL) != pdPASS) {
                ESP_LOGE(TAG, "创建任务失败");
                free(node);
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS); // 1秒延迟
    }
}

static void ble_shutdown_task(void *pvParameters) 
{
    ble_event_t evt;
    for (;;) {
        if (xQueueReceive(ble_event_queue, &evt, portMAX_DELAY)) {
            if (evt == BLE_EVENT_SHUTDOWN) {
                ESP_LOGI(TAG, "Shutting down BLE safely...");

                esp_ble_gap_stop_advertising();

                if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
                    esp_bluedroid_disable();
                    esp_bluedroid_deinit();
                }

                if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
                    esp_bt_controller_disable();
                    esp_bt_controller_deinit();
                }

                esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

                ESP_LOGI(TAG, "BLE has been completely shut down");
            }
        }
    }
}

// 命令执行任务
void command_execution_task(void *param) 
{
    // 保存上一次执行的年月日和分钟
    static int last_exec_year = -1, last_exec_mon = -1, last_exec_day = -1, last_exec_minute = -1;

    CommandNode *node = (CommandNode *)param;
    if (node == NULL) {
        vTaskDelete(NULL);
        return;
    }

    BleCommand *cmd = &node->command;

    // 计算任务结束日期（movement_days 表示任务持续天数）
    struct tm end_date = node->start_date;
    end_date.tm_mday += cmd->movement_days - 1; // movement_days=2 表示共执行两天
    mktime(&end_date);  // 规范化日期结构

    // 打印任务初始化信息
    char time_str[16];
    minutes_to_time_str(cmd->time_minutes, time_str, sizeof(time_str));
    ESP_LOGI(TAG, "开始命令任务: 每天执行时间=%s, 持续 %u 天", time_str, cmd->movement_days);

    while (1)
    {
        // 获取当前 RTC 时间
        struct tm current_time;
        rtc_get_time(&current_time);

        // 转换为 time_t 比较日期是否超期
        time_t current_time_t = mktime(&current_time);
        time_t end_time_t = mktime(&end_date);

        // --- 任务到期检测 ---
        if (current_time_t > end_time_t)
        {
            ESP_LOGI(TAG, "命令已到期，任务结束");
            break; // 跳出 while 循环，任务结束
        }

        // 计算当前时间的分钟数
        uint32_t current_minutes = current_time.tm_hour * 60 + current_time.tm_min;

        // --- 检查是否到达执行时间 ---
        if (current_minutes == cmd->time_minutes)
        {
            // 检查当前分钟是否已经执行过
            if (current_time.tm_year != last_exec_year ||
                current_time.tm_mon  != last_exec_mon  ||
                current_time.tm_mday != last_exec_day ||
                current_minutes      != last_exec_minute){

                // 计算运动距离（单位：mm）
                float actual_length_mm = (float)cmd->movement_length * 0.01f;
                if (cmd->movement_direction) // 若方向标志为1，表示反向运动
                    actual_length_mm = -actual_length_mm;

                // 执行电机动作
                set_motor_target(actual_length_mm);

                ESP_LOGI(TAG, "执行命令: 日期=%04d-%02d-%02d, 长度=%.2fmm, 方向=%s",
                        current_time.tm_year + 1900,
                        current_time.tm_mon + 1,
                        current_time.tm_mday,
                        actual_length_mm,
                        cmd->movement_direction ? "拉伸" : "收缩");

                // --- 记录执行日志 ---
                FILE *f = fopen(file_path, "a");
                if (f)
                {
                    fprintf(f, "[%04d-%02d-%02d %02d:%02d:%02d] 执行命令: 时间=%s, 长度=%.2fmm, 方向=%s\n",
                            current_time.tm_year + 1900, current_time.tm_mon + 1, current_time.tm_mday,
                            current_time.tm_hour, current_time.tm_min, current_time.tm_sec,
                            time_str, fabs(actual_length_mm),
                            cmd->movement_direction ? "拉伸" : "收缩");
                    fclose(f);
                    ESP_LOGI(TAG, "命令日志已写入SD卡");
                }
                else
                {
                    ESP_LOGE(TAG, "无法打开日志文件: %s", file_path);
                }
                // === 更新执行标记 ===
                last_exec_year   = current_time.tm_year;
                last_exec_mon    = current_time.tm_mon;
                last_exec_day    = current_time.tm_mday;
                last_exec_minute = current_minutes;
                // 减小判断频率
                vTaskDelay(pdMS_TO_TICKS(50000));
            }
        }
        else
        {
            // 每秒检查一次时间（可以根据功耗需求调整）
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // --- 清理资源并退出任务 ---
    free(node);
    vTaskDelete(NULL);
}

// ---------------- app_main ----------------
void app_main()
{
    // nvs
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    cmd_storage_init();
    // 上电读取nvs中的命令组
    cmd_t cmds[MAX_CMDS];
    size_t command_nums_in_nvs = cmd_storage_get(cmds,MAX_CMDS);
    ESP_LOGI(TAG,"%d",command_nums_in_nvs);
    if(command_nums_in_nvs != 0){
        // 初始化链表
        init_command_list(); 
        // 接收任务
        for(int i = 0; i<command_nums_in_nvs; i++){
            parse_ble_command(cmds[i].command, CMD_SIZE);
        }
        // 处理命令
        while(1)
        {
            if (!is_command_list_empty()) {
                CommandNode *node = get_next_command_node();
                if (node == NULL) {
                    continue;
                }
                float actual_length_mm = (float)node->command.movement_length * 0.01f;
                char time_str[16];
                minutes_to_time_str(node->command.time_minutes, time_str, sizeof(time_str));
                ESP_LOGI(TAG, "调度命令: 时间=%s(%u分钟), 运动长度=%.2fmm, 天数=%u", 
                        time_str, node->command.time_minutes, actual_length_mm, node->command.movement_days);
                
                // 创建任务执行命令，传递节点（节点将在任务中释放）
                if (xTaskCreate(command_execution_task, "cmd_exec", 3072, node, 5, NULL) != pdPASS) {
                    ESP_LOGE(TAG, "创建任务失败");
                    free(node);
                }
            }else{
                break;
            }
        }
    }

    // rtc
    my_rtc_init();

    // LED
    led_init();
    set_led_state(LED_GREEN); // 初始状态
    vTaskDelay(1000);
    set_led_state(LED_OFF);

    // 编码器
    pulse_counter_init(ENCODER_A_GPIO, ENCODER_B_GPIO);
    // 设置编码器参数
    encoder_config_t config = {
        .pulses_per_mm = PULSES_PER_MM
    };
    pulse_counter_set_config(config);

    // motor
    motor_init();

    // BLE
    ble_cfg_net_init();

    // SD Card
    sdcard_init();

    // 启动任务
    xTaskCreate(control_task, "control_task", 4096, NULL, 5, NULL);
    xTaskCreate(led_task, "led_task", 5219, NULL, 3, NULL);
    xTaskCreate(ble_shutdown_task, "ble_shutdown_task", 4096, NULL, 5, NULL);
    xTaskCreatePinnedToCore(ble_task,"ble_task",6144,NULL,3,NULL,1);
}
