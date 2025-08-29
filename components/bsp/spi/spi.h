#ifndef __SPI_H
#define __SPI_H

#include <unistd.h>
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_err.h"

/* SPI驱动管脚 */
#define SPI_SCLK_PIN        GPIO_NUM_12
#define SPI_MOSI_PIN        GPIO_NUM_11
#define SPI_MISO_PIN        GPIO_NUM_13
/* 总线设备引脚定义 */
#define SD_CS_PIN           GPIO_NUM_2
/* SPI端口 */
#define SPI_HOST            SPI2_HOST
/* 设备句柄 */
extern spi_device_handle_t MY_SD_Handle;   /* SD卡句柄 */

/* 函数声明 */
esp_err_t spi_init(void);    /* SPI初始化 */

#endif
