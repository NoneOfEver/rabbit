#include "uart.h"


/**
 * @brief       初始化UART
 * @param       baudrate:波特率
 * @retval      无
 */
void uart0_init(uint32_t baudrate)
{
    uart_config_t uart0_config = {0};
    uart0_config.baud_rate = baudrate;                  /* 设置波特率 */
    uart0_config.data_bits = UART_DATA_8_BITS;          /* 数据位 */
    uart0_config.parity = UART_PARITY_DISABLE;          /* 无奇偶校验位 */
    uart0_config.stop_bits = UART_STOP_BITS_1;          /* 一位停止位 */
    uart0_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;  /* 无硬件流控 */
    uart0_config.source_clk = UART_SCLK_DEFAULT;        /* 选择时钟源 */

    ESP_ERROR_CHECK(uart_param_config(USART_UX, &uart0_config));    /* UART0配置 */
    /* 设置管脚 */
    ESP_ERROR_CHECK(uart_set_pin(USART_UX, USART_TX_GPIO_PIN, USART_RX_GPIO_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    /* 安装串口驱动 */
    ESP_ERROR_CHECK(uart_driver_install(USART_UX, RX_BUF_SIZE, RX_BUF_SIZE, 0, NULL, 0));
}
