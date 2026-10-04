#include <string.h>
#include "driver/spi_master.h"

#include "icm20948.h"
#include "icm20948_spi.h"


icm20948_status_e icm20948_internal_write_spi(uint8_t reg, uint8_t *data, uint32_t len, void *handle)
{	
    uint8_t length = len + 1;
    uint8_t tx_buffer[length];
    /* setup data trasmit buffer */
    tx_buffer[0] = ((reg & 0x7F) | 0x00);
    memcpy(tx_buffer+1, data, len);

    spi_transaction_t trans_desc = {
        .flags = 0,
        .tx_buffer = tx_buffer,
        .length = length*8
    };
    if (spi_device_polling_transmit((spi_device_handle_t)handle, &trans_desc) != ESP_OK)
    {
        return ICM_20948_STAT_ERR;
    }
    return ICM_20948_STAT_OK;
}


icm20948_status_e icm20948_internal_read_spi(uint8_t reg, uint8_t *buff, uint32_t len, void *handle)
{
    uint8_t length = len + 1;
    uint8_t tx_buffer[length];    
    uint8_t rx_buffer[length];
    /* setup data buffers */
    memset(tx_buffer+1, 0x0, len);
    tx_buffer[0] = ((reg & 0x7F) | 0x80);
	
	spi_transaction_t trans_desc = {
            .flags = 0,
            .tx_buffer = tx_buffer,
            .rxlength = length*8,
			.rx_buffer = rx_buffer,
            .length = length*8
    };
    if (spi_device_polling_transmit((spi_device_handle_t)handle, &trans_desc) != ESP_OK)
    {
        return ICM_20948_STAT_ERR;
    }
    /* copy received data back to buff */
    memcpy(buff, rx_buffer+1, len);
  	return ICM_20948_STAT_OK;
}

/* setup a default SPI-serif for a single-device use. If someone wants to use mutliple ICM-20948, the
   a serif for each device has to be implemented. */
icm20948_serif_t default_serif = {
    icm20948_internal_write_spi,
    icm20948_internal_read_spi,
    NULL,
};


void icm20948_init_spi(icm20948_device_t *icm_device, spi_device_handle_t *handle)
{
    default_serif.user = *handle;
    icm20948_init_struct(icm_device);
    icm20948_link_serif(icm_device, &default_serif);

#if CONFIG_ICM_20948_USE_DMP
  icm_device->_dmp_firmware_available = true; // Initialize _dmp_firmware_available 
#else
  icm_device->_dmp_firmware_available = false; // Initialize _dmp_firmware_available
#endif

    icm_device->_firmware_loaded = false; // Initialize _firmware_loaded
    icm_device->_last_bank = 255;         // Initialize _last_bank. Make it invalid. It will be set by the first call of icm20948_set_bank.
    icm_device->_last_mems_bank = 255;    // Initialize _last_mems_bank. Make it invalid. It will be set by the first call of inv_icm20948_write_mems.
    icm_device->_gyroSF = 0;              // Use this to record the GyroSF, calculated by inv_icm20948_set_gyro_sf
    icm_device->_gyroSFpll = 0;
    icm_device->_enabled_Android_0 = 0;      // Keep track of which Android sensors are enabled: 0-31
    icm_device->_enabled_Android_1 = 0;      // Keep track of which Android sensors are enabled: 32-
    icm_device->_enabled_Android_intr_0 = 0; // Keep track of which Android sensor interrupts are enabled: 0-31
    icm_device->_enabled_Android_intr_1 = 0; // Keep track of which Android sensor interrupts are enabled: 32-
}


