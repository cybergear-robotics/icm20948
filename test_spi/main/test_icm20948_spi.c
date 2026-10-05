#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "unity.h"

#include "icm20948.h"
#include "icm20948_spi.h"
#include "ak09916_enumerations.h"
#include "ak09916_registers.h"

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
static QueueHandle_t int_queue;

static void IRAM_ATTR int1_isr(void *arg)
{
    const gpio_num_t gpio_num = (gpio_num_t)arg;
    BaseType_t higher_priority_task_woken = pdFALSE;

    xQueueSendFromISR(int_queue, &gpio_num, &higher_priority_task_woken);
    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

static void initialize_int1_gpio(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << CONFIG_ICM20948_SPI_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };

    TEST_ASSERT_EQUAL(ESP_OK, gpio_config(&config));
    if (int_queue == NULL) {
        int_queue = xQueueCreate(4, sizeof(gpio_num_t));
        TEST_ASSERT_NOT_NULL(int_queue);
        const esp_err_t result = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        TEST_ASSERT_TRUE(result == ESP_OK || result == ESP_ERR_INVALID_STATE);
        TEST_ASSERT_EQUAL(ESP_OK, gpio_isr_handler_add(CONFIG_ICM20948_SPI_INT_GPIO, int1_isr,
                                                        (void *)CONFIG_ICM20948_SPI_INT_GPIO));
    }
}

static void clear_int_queue(void)
{
    gpio_num_t interrupt_gpio;

    while (xQueueReceive(int_queue, &interrupt_gpio, 0) == pdTRUE) {
    }
}

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

static void reset_spi_device(void)
{
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_sw_reset(&icm));
    vTaskDelay(pdMS_TO_TICKS(250));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_sleep(&icm, false));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_low_power(&icm, false));
}

static void initialize_dmp(void)
{
    reset_spi_device();
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_init_dmp_sensor_with_defaults(&icm));
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
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_SENSOR_NOT_SUPPORTED,
                          icm20948_set_full_scale(&icm, ICM_20948_INTERNAL_MAG, (icm20948_fss_t){0}));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_SENSOR_NOT_SUPPORTED,
                          icm20948_set_sample_rate(&icm, ICM_20948_INTERNAL_MAG, (icm20948_smplrt_t){0}));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_PARAM_ERR,
                           icm20948_configure_magnetometer(&icm, AK09916_MODE_SINGLE));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_PARAM_ERR,
                          icm20948_i2c_controller_configure_peripheral(&icm, 4, 0, 0, 0, false, false,
                                                                          false, false, false, 0));
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
    const icm20948_smplrt_t sample_rate = {.a = 19, .g = 19};
    const icm20948_fss_t full_scale = {.a = GPM_4, .g = DPS_500};
    const icm20948_dlpcfg_t dlpf = {.a = ACC_D246BW_N265BW, .g = GYR_D119BW5_B154BW3};
    uint8_t accel_config;
    uint8_t gyro_config;
    uint8_t accel_rate[2];
    uint8_t gyro_rate;
    uint8_t lp_config;

    initialize_spi_testbed();
    reset_spi_device();
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_set_sample_mode(&icm, sensors, SAMPLE_MODE_CYCLED));
    // Read LP_CONFIG directly to verify the ACCEL_CYCLE and GYRO_CYCLE bits written by the API.
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_execute_r(&icm, AGB0_REG_LP_CONFIG, &lp_config, 1));
    TEST_ASSERT_EQUAL_HEX8(0x30, lp_config & 0x30);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_set_full_scale(&icm, sensors, full_scale));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_set_dlpf_cfg(&icm, sensors, dlpf));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_enable_dlpf(&icm, sensors, true));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_set_sample_rate(&icm, sensors, sample_rate));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_execute_r(&icm, AGB2_REG_ACCEL_CONFIG, &accel_config, 1));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_execute_r(&icm, AGB2_REG_GYRO_CONFIG_1, &gyro_config, 1));
    TEST_ASSERT_EQUAL_UINT8(full_scale.a, (accel_config >> 1) & 0x03);
    TEST_ASSERT_EQUAL_UINT8(full_scale.g, (gyro_config >> 1) & 0x03);
    TEST_ASSERT_EQUAL_UINT8(dlpf.a, (accel_config >> 3) & 0x07);
    TEST_ASSERT_EQUAL_UINT8(dlpf.g, (gyro_config >> 3) & 0x07);
    TEST_ASSERT_EQUAL_UINT8(1, accel_config & 0x01);
    TEST_ASSERT_EQUAL_UINT8(1, gyro_config & 0x01);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                          icm20948_execute_r(&icm, AGB2_REG_ACCEL_SMPLRT_DIV_1, accel_rate, sizeof(accel_rate)));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_execute_r(&icm, AGB2_REG_GYRO_SMPLRT_DIV, &gyro_rate, 1));
    TEST_ASSERT_EQUAL_UINT8(sample_rate.a >> 8, accel_rate[0] & 0x0F);
    TEST_ASSERT_EQUAL_UINT8(sample_rate.a & 0xFF, accel_rate[1]);
    TEST_ASSERT_EQUAL_UINT8(sample_rate.g, gyro_rate);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_set_sample_mode(&icm, sensors, SAMPLE_MODE_CONTINUOUS));
    // A direct register read verifies that the disable path cleared both cycle bits.
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_execute_r(&icm, AGB0_REG_LP_CONFIG, &lp_config, 1));
    TEST_ASSERT_EQUAL_HEX8(0, lp_config & 0x30);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_enable_dlpf(&icm, sensors, false));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_execute_r(&icm, AGB2_REG_ACCEL_CONFIG, &accel_config, 1));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_execute_r(&icm, AGB2_REG_GYRO_CONFIG_1, &gyro_config, 1));
    TEST_ASSERT_EQUAL_UINT8(0, accel_config & 0x01);
    TEST_ASSERT_EQUAL_UINT8(0, gyro_config & 0x01);
}

TEST_CASE("ICM-20948 SPI configures interrupts and reports data ready", "[icm20948][spi]")
{
    icm20948_int_pin_cfg_t pin_config = {
        .INT_ANYRD_2CLEAR = 1,
        .INT1_LATCH_EN = 1,
        .INT1_ACTL = 0,
    };
    icm20948_int_enable_t int_enable = {
        .DMP_INT1_EN = 1,
        .RAW_DATA_0_RDY_EN = 1,
        .FIFO_OVERFLOW_EN_0 = 1,
        .FIFO_WM_EN_0 = 1,
    };
    icm20948_int_pin_cfg_t pin_read = {0};
    icm20948_int_enable_t int_read = {0};
    gpio_num_t interrupt_gpio;

    initialize_spi_testbed();
    reset_spi_device();
    initialize_int1_gpio();
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_int_pin_cfg(&icm, &pin_config, &pin_read));
    TEST_ASSERT_EQUAL_UINT8(pin_config.INT_ANYRD_2CLEAR, pin_read.INT_ANYRD_2CLEAR);
    TEST_ASSERT_EQUAL_UINT8(pin_config.INT1_LATCH_EN, pin_read.INT1_LATCH_EN);
    TEST_ASSERT_EQUAL_UINT8(pin_config.INT1_ACTL, pin_read.INT1_ACTL);
    clear_int_queue();
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_int_enable(&icm, &int_enable, &int_read));
    TEST_ASSERT_EQUAL_UINT8(int_enable.DMP_INT1_EN, int_read.DMP_INT1_EN);
    TEST_ASSERT_EQUAL_UINT8(int_enable.RAW_DATA_0_RDY_EN, int_read.RAW_DATA_0_RDY_EN);
    TEST_ASSERT_EQUAL_UINT8(int_enable.FIFO_OVERFLOW_EN_0, int_read.FIFO_OVERFLOW_EN_0);
    TEST_ASSERT_EQUAL_UINT8(int_enable.FIFO_WM_EN_0, int_read.FIFO_WM_EN_0);
    TEST_ASSERT_EQUAL(pdTRUE, xQueueReceive(int_queue, &interrupt_gpio, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(CONFIG_ICM20948_SPI_INT_GPIO, interrupt_gpio);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_data_ready(&icm));
}

TEST_CASE("ICM-20948 SPI configures wake-on-motion registers", "[icm20948][spi][wom]")
{
    icm20948_accel_intel_ctrl_t logic = {
        .ACCEL_INTEL_EN = 1,
        .ACCEL_INTEL_MODE_INT = 1,
    };
    icm20948_accel_wom_thr_t threshold = {.WOM_THRESHOLD = 42};
    icm20948_accel_intel_ctrl_t logic_read = {0};
    icm20948_accel_wom_thr_t threshold_read = {0};

    initialize_spi_testbed();
    reset_spi_device();
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_wom_logic(&icm, &logic, &logic_read));
    TEST_ASSERT_EQUAL_UINT8(logic.ACCEL_INTEL_EN, logic_read.ACCEL_INTEL_EN);
    TEST_ASSERT_EQUAL_UINT8(logic.ACCEL_INTEL_MODE_INT, logic_read.ACCEL_INTEL_MODE_INT);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                          icm20948_wom_threshold(&icm, &threshold, &threshold_read));
    TEST_ASSERT_EQUAL_UINT8(threshold.WOM_THRESHOLD, threshold_read.WOM_THRESHOLD);
}

TEST_CASE("ICM-20948 SPI enters and exits sleep and low-power modes", "[icm20948][spi]")
{
    uint8_t pwr_mgmt_1;

    initialize_spi_testbed();
    reset_spi_device();
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_sleep(&icm, true));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_execute_r(&icm, AGB0_REG_PWR_MGMT_1, &pwr_mgmt_1, 1));
    TEST_ASSERT_EQUAL_UINT8(1, (pwr_mgmt_1 >> 6) & 0x01);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_sleep(&icm, false));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_low_power(&icm, true));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_execute_r(&icm, AGB0_REG_PWR_MGMT_1, &pwr_mgmt_1, 1));
    TEST_ASSERT_EQUAL_UINT8(1, (pwr_mgmt_1 >> 5) & 0x01);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_low_power(&icm, false));
}

TEST_CASE("ICM-20948 SPI reads raw FIFO data", "[icm20948][spi][fifo]")
{
    uint8_t fifo_en_2 = 0x1F;
    uint8_t fifo_data[14];
    uint16_t count = 0;
    uint16_t count_before_read = 0;

    initialize_spi_testbed();
    reset_spi_device();
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                          icm20948_set_sample_mode(&icm, ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR,
                                                    SAMPLE_MODE_CONTINUOUS));
    // FIFO_EN_2 selects temperature, all gyro axes, and accelerometer for the raw FIFO stream.
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_execute_w(&icm, AGB0_REG_FIFO_EN_2, &fifo_en_2, 1));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_enable_fifo(&icm, true));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_set_fifo_mode(&icm, false));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_reset_fifo(&icm));
    for (int attempt = 0; attempt < 50; ++attempt) {
        TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_get_fifo_count(&icm, &count));
        if (count >= sizeof(fifo_data)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    TEST_ASSERT_GREATER_OR_EQUAL_UINT16(sizeof(fifo_data), count);
    // Stop producing samples before comparing the count around the FIFO_R_W read.
    fifo_en_2 = 0;
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_execute_w(&icm, AGB0_REG_FIFO_EN_2, &fifo_en_2, 1));
    vTaskDelay(pdMS_TO_TICKS(20));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_get_fifo_count(&icm, &count_before_read));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT16(sizeof(fifo_data), count_before_read);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_read_fifo(&icm, fifo_data, sizeof(fifo_data)));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_get_fifo_count(&icm, &count));
    TEST_ASSERT_EQUAL_UINT16(count_before_read - sizeof(fifo_data), count);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_enable_fifo(&icm, false));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_reset_fifo(&icm));
}

TEST_CASE("ICM-20948 SPI controls the auxiliary I2C master", "[icm20948][spi][aux-i2c]")
{
    uint8_t register_value;
    uint8_t magnetometer_id = 0;

    initialize_spi_testbed();
    reset_spi_device();
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_i2c_master_passthrough(&icm, true));
    // INT_PIN_CONFIG owns BYPASS_EN; read it directly because passthrough has no readback API.
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_execute_r(&icm, AGB0_REG_INT_PIN_CONFIG, &register_value, 1));
    TEST_ASSERT_BITS_HIGH(0x02, register_value);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_i2c_master_enable(&icm, true));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                          icm20948_i2c_master_single_r(&icm, MAG_AK09916_I2C_ADDR, AK09916_REG_WIA2,
                                                        &magnetometer_id));
    TEST_ASSERT_EQUAL_HEX8(0x09, magnetometer_id);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                          icm20948_i2c_controller_configure_peripheral(&icm, 0, MAG_AK09916_I2C_ADDR,
                                                                          AK09916_REG_WIA2, 1, true, true,
                                                                          false, false, false, 0));
    // Verify the persistent SLV0 setup: 0x8C is AK09916 address 0x0C with the read bit set.
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_execute_r(&icm, AGB3_REG_I2C_PERIPH0_ADDR, &register_value, 1));
    TEST_ASSERT_EQUAL_HEX8(0x8C, register_value);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_execute_r(&icm, AGB3_REG_I2C_PERIPH0_CTRL, &register_value, 1));
    TEST_ASSERT_EQUAL_HEX8(0x81, register_value);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_i2c_master_reset(&icm));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_i2c_master_enable(&icm, false));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_execute_r(&icm, AGB0_REG_USER_CTRL, &register_value, 1));
    TEST_ASSERT_BITS_LOW(0x20, register_value);
}

TEST_CASE("ICM-20948 SPI loads DMP firmware and controls DMP state", "[icm20948][spi][dmp]")
{
    uint8_t start_address[2];
    uint8_t user_ctrl;

    initialize_spi_testbed();
    reset_spi_device();
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_firmware_load(&icm));
    TEST_ASSERT_TRUE(icm._firmware_loaded);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_set_dmp_start_address(&icm, 0x1234));
    // The start address is stored as a big-endian pair in Bank 2 program-start registers.
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                          icm20948_execute_r(&icm, AGB2_REG_PRGM_START_ADDRH, start_address, sizeof(start_address)));
    TEST_ASSERT_EQUAL_HEX8(0x12, start_address[0]);
    TEST_ASSERT_EQUAL_HEX8(0x34, start_address[1]);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_set_dmp_start_address(&icm, DMP_START_ADDRESS));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_enable_dmp(&icm, true));
    // USER_CTRL owns DMP_EN; direct reads verify both enable and disable operations.
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_execute_r(&icm, AGB0_REG_USER_CTRL, &user_ctrl, 1));
    TEST_ASSERT_BITS_HIGH(0x80, user_ctrl);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_enable_dmp(&icm, false));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_execute_r(&icm, AGB0_REG_USER_CTRL, &user_ctrl, 1));
    TEST_ASSERT_BITS_LOW(0x80, user_ctrl);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_INVALID_DMP_REGISTER,
                          inv_icm20948_set_dmp_sensor_period(&icm, (enum DMP_ODR_Registers)-1, 0));
}

TEST_CASE("ICM-20948 SPI reads aggregate sensor data", "[icm20948][spi]")
{
    icm20948_agmt_t agmt;

    initialize_spi_testbed();
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_get_agmt(&icm, &agmt));
}

TEST_CASE("ICM-20948 SPI reports DMP FIFO data through INT1", "[icm20948][spi][dmp]")
{
    const unsigned short memory_address = 0x02F8;
    const unsigned char memory_pattern[] = {
        0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7,
        0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF,
        0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7,
    };
    unsigned char original_memory[sizeof(memory_pattern)];
    unsigned char read_memory[sizeof(memory_pattern)];
    icm_20948_DMP_data_t data = {0};
    bool quaternion_received = false;
    bool quat6_received = false;
    bool accel_received = false;
    bool gyro_received = false;
    bool fifo_data_available = false;
    icm20948_status_e memory_status;
    icm20948_int_enable_t int_enable = {.DMP_INT1_EN = 1};
    gpio_num_t interrupt_gpio;

    initialize_spi_testbed();
    initialize_dmp();
    // Preserve the DMP memory contents so this read/write verification leaves the loaded image unchanged.
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                          inv_icm20948_read_mems(&icm, memory_address, sizeof(original_memory), original_memory));
    memory_status = inv_icm20948_write_mems(&icm, memory_address, sizeof(memory_pattern), memory_pattern);
    if (memory_status == ICM_20948_STAT_OK) {
        memory_status = inv_icm20948_read_mems(&icm, memory_address, sizeof(read_memory), read_memory);
    }
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                          inv_icm20948_write_mems(&icm, memory_address, sizeof(original_memory), original_memory));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, memory_status);
    TEST_ASSERT_EQUAL_MEMORY(memory_pattern, read_memory, sizeof(memory_pattern));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                           inv_icm20948_enable_dmp_sensor(&icm, INV_ICM20948_SENSOR_ORIENTATION, 1));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                           inv_icm20948_enable_dmp_sensor(&icm, INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, 1));
    // DATA_INTR_CTL is DMP memory; the cached value records the exact interrupt mask written there.
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                          inv_icm20948_enable_dmp_sensor_int(&icm, INV_ICM20948_SENSOR_ORIENTATION, 1));
    TEST_ASSERT_BITS_HIGH(DMP_Data_Output_Control_1_Quat9, icm._dataIntrCtl);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                          inv_icm20948_enable_dmp_sensor(&icm, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, 1));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                          inv_icm20948_enable_dmp_sensor(&icm, INV_ICM20948_SENSOR_RAW_GYROSCOPE, 1));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                           inv_icm20948_set_dmp_sensor_period(&icm, DMP_ODR_Reg_Quat9, 0));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                          inv_icm20948_set_dmp_sensor_period(&icm, DMP_ODR_Reg_Quat6, 0));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                          inv_icm20948_set_dmp_sensor_period(&icm, DMP_ODR_Reg_Accel, 0));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                           inv_icm20948_set_dmp_sensor_period(&icm, DMP_ODR_Reg_Gyro, 0));
    initialize_int1_gpio();
    clear_int_queue();
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_int_enable(&icm, &int_enable, NULL));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_enable_fifo(&icm, true));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_enable_dmp(&icm, true));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_reset_dmp(&icm));
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_reset_fifo(&icm));
    TEST_ASSERT_EQUAL(pdTRUE, xQueueReceive(int_queue, &interrupt_gpio, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(CONFIG_ICM20948_SPI_INT_GPIO, interrupt_gpio);

    for (int attempt = 0; attempt < 300; ++attempt) {
        uint16_t fifo_count = 0;
        TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK, icm20948_get_fifo_count(&icm, &fifo_count));
        fifo_data_available |= fifo_count > 0;
        const icm20948_status_e status = inv_icm20948_read_dmp_data(&icm, &data);
        if (status == ICM_20948_STAT_OK || status == ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
            quaternion_received |= (data.header & DMP_header_bitmap_Quat9) != 0;
            quat6_received |= (data.header & DMP_header_bitmap_Quat6) != 0;
            accel_received |= (data.header & DMP_header_bitmap_Accel) != 0;
            gyro_received |= (data.header & DMP_header_bitmap_Gyro) != 0;
        }
        if (quaternion_received && quat6_received && accel_received && gyro_received) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Packet arrival is asynchronous, so assert observed DMP headers instead of fixed payload values.
    TEST_ASSERT_TRUE(fifo_data_available);
    TEST_ASSERT_TRUE(quaternion_received);
    TEST_ASSERT_TRUE(quat6_received);
    TEST_ASSERT_TRUE(accel_received);
    TEST_ASSERT_TRUE(gyro_received);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                           inv_icm20948_enable_dmp_sensor(&icm, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, 0));
    TEST_ASSERT_EQUAL_HEX16(0, icm._dataOutCtl1 & DMP_Data_Output_Control_1_Accel);
    TEST_ASSERT_EQUAL_INT(ICM_20948_STAT_OK,
                          inv_icm20948_enable_dmp_sensor_int(&icm, INV_ICM20948_SENSOR_ORIENTATION, 0));
    TEST_ASSERT_EQUAL_HEX16(0, icm._dataIntrCtl & DMP_Data_Output_Control_1_Quat9);
}
