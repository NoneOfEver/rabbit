#include "sd_card.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"

// ---------------- SD Card ----------------
#define PIN_CS 13
#define PIN_MOSI 11
#define PIN_MISO 12
#define PIN_CLK 10
 


// ---------------- SD Card ----------------
void sdcard_init()
{
	sdmmc_host_t host=SDSPI_HOST_DEFAULT();
	host.slot=SPI2_HOST;
	spi_bus_config_t bus_cnf={
		.mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_CLK,
		.quadhd_io_num=-1,  //这行不能省略
		.quadwp_io_num=-1,  //这行不能省略
		.max_transfer_sz=400000,
	};
	spi_bus_initialize(host.slot,&bus_cnf,SPI_DMA_CH_AUTO);
	static sdspi_device_config_t slot_cnf={
		.gpio_cs=PIN_CS,
		.gpio_cd=SDSPI_SLOT_NO_CD, //这行不能省略
		.gpio_int=GPIO_NUM_NC,//这行不能省略
		.gpio_wp=GPIO_NUM_NC,//这行不能省略
		.host_id=SPI2_HOST,
    };
	sdmmc_card_t *card;
	esp_vfs_fat_sdmmc_mount_config_t mount_cnf={
		.format_if_mount_failed=true,
		.max_files=5,
		.allocation_unit_size=16*1024,
	};
	esp_vfs_fat_sdspi_mount(MOUNT_POINT,&host,&slot_cnf,&mount_cnf,&card);
}
