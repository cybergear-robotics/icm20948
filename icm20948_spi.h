#ifndef _ICM_20948_SPI_H_
#define _ICM_20948_SPI_H_

#include "icm20948.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

void icm20948_init_spi(icm20948_device_t *icm_device, spi_device_handle_t *handle);

/* these functions are exposed in order to make a custom setup of a serif_t possible */
icm20948_status_e icm20948_internal_write_spi(uint8_t reg, uint8_t *data, uint32_t len, void *handle);
icm20948_status_e icm20948_internal_read_spi(uint8_t reg, uint8_t *buff, uint32_t len, void *handle);

#ifdef __cplusplus
}
#endif

#endif
