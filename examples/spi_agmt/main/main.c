#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_system.h"

#include "driver/spi_master.h"

#include "icm20948.h"
#include "icm20948_spi.h"

#define TAG "spi_agmt"

spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_SPI_MASTER_MISO,
        .mosi_io_num = CONFIG_SPI_MASTER_MOSI,
        .sclk_io_num = CONFIG_SPI_MASTER_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 512 * 8 /* 4095 bytes is the max size of data that can be sent because of hardware limitations */
};

spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 4000000,  			/* Clock out at 4 MHz */ 
        .mode = 0,                  			/* SPI mode 0 */
        .spics_io_num = CONFIG_SPI_MASTER_CS, 	/* This field is used to specify the GPIO pin that is to be used as CS' */
        .queue_size = 1             			/* We want to be able to queue 7 transactions at a time */
};


void print_agmt(icm20948_agmt_t agmt) {
  	ESP_LOGI(TAG, "Acc: [ %d, %d, %d ] Gyr: [ %d, %d, %d ] Mag: [ %d, %d, %d ] Tmp: [ %d ]", 
		agmt.acc.axes.x, agmt.acc.axes.y, agmt.acc.axes.z,
		agmt.gyr.axes.x, agmt.gyr.axes.y, agmt.gyr.axes.z,
		agmt.mag.axes.x, agmt.mag.axes.y, agmt.mag.axes.z,
		agmt.tmp.val
	);
}


void app_main(void)
{
	icm20948_device_t icm;
	spi_device_handle_t spi;

	/* setup SPI bus and add SPI device */
	ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &spi));
	icm20948_init_spi(&icm, &spi);

	/* check ID */
    while (icm20948_check_id(&icm) != ICM_20948_STAT_OK)
	{
		ESP_LOGE(TAG, "check id failed");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
	ESP_LOGI(TAG, "check id passed");

	/* check whoami */
	icm20948_status_e stat = ICM_20948_STAT_ERR;
	uint8_t whoami = 0x00;
	do {
		whoami = 0x00;
		stat = icm20948_get_who_am_i(&icm, &whoami);
		if ((stat != ICM_20948_STAT_OK) || (whoami != ICM_20948_WHOAMI)) {
			ESP_LOGE(TAG, "whoami does not match (0x%02X). Retrying...", whoami);
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	} while ((stat != ICM_20948_STAT_OK) || (whoami != ICM_20948_WHOAMI));

	/* Here we are doing a SW reset to make sure the device starts in a known state */
	icm20948_sw_reset(&icm);
	vTaskDelay(250 / portTICK_PERIOD_MS);

	icm20948_internal_sensor_id_bm sensors = (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);

	/* Set Gyro and Accelerometer to a particular sample mode */
	// optiona: SAMPLE_MODE_CONTINUOUS. SAMPLE_MODE_CYCLED
	icm20948_set_sample_mode(&icm, sensors, SAMPLE_MODE_CONTINUOUS); 

	/* Set full scale ranges for both acc and gyr */
	icm20948_fss_t myfss;
	myfss.a = GPM_2;   // (icm20948_accel_config_fs_sel_e)
	myfss.g = DPS_250; // (icm20948_gyro_config_1_fs_sel_e)
	icm20948_set_full_scale(&icm, sensors, myfss);

	/* Set up DLPF configuration */
	icm20948_dlpcfg_t myDLPcfg;
	myDLPcfg.a = ACC_D473BW_N499BW;
	myDLPcfg.g = GYR_D361BW4_N376BW5;
	icm20948_set_dlpf_cfg(&icm, sensors, myDLPcfg);

	/* Choose whether or not to use DLPF */
	icm20948_enable_dlpf(&icm, ICM_20948_INTERNAL_ACC, false);
	icm20948_enable_dlpf(&icm, ICM_20948_INTERNAL_GYR, false);

	/* Now wake the sensor up */
	icm20948_sleep(&icm, false);
	icm20948_low_power(&icm, false);
	if (icm20948_configure_magnetometer(&icm, AK09916_MODE_CONT_100_HZ) != ICM_20948_STAT_OK) {
		ESP_LOGE(TAG, "magnetometer configuration failed");
		return;
	}

    /* loop */
    while(1)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);

		icm20948_agmt_t agmt;
		if (icm20948_get_agmt(&icm, &agmt) == ICM_20948_STAT_OK) {
			print_agmt(agmt);
		} else {
			ESP_LOGE(TAG, "Uh oh");
		}        
    }
}
