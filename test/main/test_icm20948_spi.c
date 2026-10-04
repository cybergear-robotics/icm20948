#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "unity.h"

#include "icm20948.h"
#include "icm20948_spi.h"
#include "ak09916_enumerations.h"

static const spi_bus_config_t bus_config = {
    .miso_io_num = CONFIG_ICM20948_SPI_MISO_GPIO,
    .mosi_io_num = CONFIG_ICM20948_SPI_MOSI_GPIO,
    .sclk_io_num = CONFIG_ICM20948_SPI_SCK_GPIO,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 512,
};

static const spi_device_interface_config_t device_config = {
    .clock_speed_hz = 4000000,
    .mode = 0,
    .spics_io_num = CONFIG_ICM20948_SPI_CS_GPIO,
    .queue_size = 1,
};

static icm20948_device_t icm;
static spi_device_handle_t spi;
static bool initialized;

static void initialize_spi_testbed(void)
{
    if (initialized) {
        return;
    }

    TEST_ASSERT_EQUAL(ESP_OK, spi_bus_initialize(SPI3_HOST, &bus_config, SPI_DMA_CH_AUTO));
    TEST_ASSERT_EQUAL(ESP_OK, spi_bus_add_device(SPI3_HOST, &device_config, &spi));
    icm20948_init_spi(&icm, &spi);
    initialized = true;
}

TEST_CASE("ICM-20948 initializes driver state and rejects invalid parameters", "[icm20948][spi]")
{
    icm20948_device_t device;

    initialize_spi_testbed();
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_init_struct(&device));
    TEST_ASSERT_NULL(device._serif);
    TEST_ASSERT_EQUAL_UINT8(4, device._last_bank);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_PARAM_ERR, icm20948_link_serif(&device, NULL));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_PARAM_ERR, icm20948_set_bank(&device, 4));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_PARAM_ERR, icm20948_get_who_am_i(&icm, NULL));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_SENSOR_NOT_SUPPORTED,
                          icm20948_set_sample_mode(&icm, ICM_20948_INTERNAL_MAG, SAMPLE_MODE_CONTINUOUS));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_SENSOR_NOT_SUPPORTED,
                          icm20948_enable_dlpf(&icm, ICM_20948_INTERNAL_MAG, true));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_PARAM_ERR,
                          icm20948_configure_magnetometer(&icm, AK09916_MODE_SINGLE));
}

TEST_CASE("ICM-20948 SPI reads the expected WHO_AM_I value", "[icm20948][spi]")
{
    uint8_t who_am_i = 0;

    initialize_spi_testbed();
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_sw_reset(&icm));
    vTaskDelay(pdMS_TO_TICKS(250));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_sleep(&icm, false));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                          icm20948_internal_read_spi(AGB0_REG_WHO_AM_I, &who_am_i, 1, spi));
    TEST_ASSERT_EQUAL_HEX8(ICM_20948_WHOAMI, who_am_i);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_check_id(&icm));
}

TEST_CASE("ICM-20948 SPI reset restores communication", "[icm20948][spi]")
{
    uint8_t who_am_i = 0;

    initialize_spi_testbed();
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_sw_reset(&icm));
    vTaskDelay(pdMS_TO_TICKS(250));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_sleep(&icm, false));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_low_power(&icm, false));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_set_clock_source(&icm, CLOCK_AUTO));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_get_who_am_i(&icm, &who_am_i));
    TEST_ASSERT_EQUAL_HEX8(ICM_20948_WHOAMI, who_am_i);
}

TEST_CASE("ICM-20948 SPI configures accelerometer and gyroscope", "[icm20948][spi]")
{
    const icm20948_internal_sensor_id_bm sensors =
        ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR;
    const icm20948_fss_t full_scale = {.a = GPM_4, .g = DPS_500};
    const icm20948_dlpcfg_t dlpf = {.a = ACC_D246BW_N265BW, .g = GYR_D119BW5_B154BW3};
    const icm20948_smplrt_t sample_rate = {.a = 19, .g = 19};

    initialize_spi_testbed();
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_set_sample_mode(&icm, sensors, SAMPLE_MODE_CONTINUOUS));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_set_full_scale(&icm, sensors, full_scale));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_set_dlpf_cfg(&icm, sensors, dlpf));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_enable_dlpf(&icm, sensors, true));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_set_sample_rate(&icm, sensors, sample_rate));
}

TEST_CASE("ICM-20948 SPI controls and resets FIFO", "[icm20948][spi]")
{
    uint16_t count = 0;

    initialize_spi_testbed();
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_enable_fifo(&icm, true));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_set_fifo_mode(&icm, false));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_reset_fifo(&icm));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_get_fifo_count(&icm, &count));
    TEST_ASSERT_EQUAL_UINT16(0, count);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_enable_fifo(&icm, false));
}

TEST_CASE("ICM-20948 SPI reads aggregate sensor data", "[icm20948][spi]")
{
    icm20948_agmt_t agmt;

    initialize_spi_testbed();
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_get_agmt(&icm, &agmt));
}

TEST_CASE("ICM-20948 SPI configures and reads the integrated magnetometer", "[icm20948][spi][mag]")
{
    icm20948_agmt_t agmt = {0};
    bool data_ready = false;

    initialize_spi_testbed();
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                          icm20948_configure_magnetometer(&icm, AK09916_MODE_CONT_100_HZ));

    for (int attempt = 0; attempt < 20; ++attempt) {
        vTaskDelay(pdMS_TO_TICKS(20));
        TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_get_agmt(&icm, &agmt));
        if (agmt.magStat1 & 0x01) {
            data_ready = true;
            break;
        }
    }

    TEST_ASSERT_TRUE(data_ready);
    TEST_ASSERT_EQUAL_UINT8(0, agmt.magStat2 & 0x08);
    TEST_ASSERT_TRUE(agmt.mag.axes.x != 0 || agmt.mag.axes.y != 0 || agmt.mag.axes.z != 0);
}
