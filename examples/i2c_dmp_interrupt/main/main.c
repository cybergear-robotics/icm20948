#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "icm20948.h"
#include "icm20948_i2c.h"

#define TAG "i2c_dmp_interrupt"
#define CHECK_ICM(call) ESP_ERROR_CHECK((call) == ICM_20948_STAT_OK ? ESP_OK : ESP_FAIL)

static TaskHandle_t dmp_task;

static void IRAM_ATTR int1_isr(void *arg)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    vTaskNotifyGiveFromISR(dmp_task, &higher_priority_task_woken);
    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

static void configure_dmp(icm20948_device_t *icm)
{
    icm20948_int_enable_t int_enable = {.DMP_INT1_EN = 1};

    CHECK_ICM(icm20948_init_dmp_sensor_with_defaults(icm));
    CHECK_ICM(inv_icm20948_enable_dmp_sensor(icm, INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, 1));
    CHECK_ICM(inv_icm20948_set_dmp_sensor_period(icm, DMP_ODR_Reg_Quat6, 0));
    CHECK_ICM(icm20948_enable_fifo(icm, true));
    CHECK_ICM(icm20948_enable_dmp(icm, true));
    CHECK_ICM(icm20948_reset_dmp(icm));
    CHECK_ICM(icm20948_reset_fifo(icm));
    CHECK_ICM(icm20948_int_enable(icm, &int_enable, NULL));
}

void app_main(void)
{
    const i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_ICM20948_I2C_DMP_INT_SDA_GPIO,
        .scl_io_num = CONFIG_ICM20948_I2C_DMP_INT_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM_20948_I2C_ADDR_AD1,
        .scl_speed_hz = 400000,
    };
    const gpio_config_t int_config = {
        .pin_bit_mask = 1ULL << CONFIG_ICM20948_I2C_DMP_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    icm0948_config_i2c_t icm_config = {0};
    icm20948_device_t icm;
    i2c_master_bus_handle_t bus;

    ESP_ERROR_CHECK(gpio_set_direction(CONFIG_ICM20948_I2C_DMP_INT_CS_GPIO, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(CONFIG_ICM20948_I2C_DMP_INT_CS_GPIO, 1));
    ESP_ERROR_CHECK(gpio_set_direction(CONFIG_ICM20948_I2C_DMP_INT_AD0_GPIO, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(CONFIG_ICM20948_I2C_DMP_INT_AD0_GPIO, 1));
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &device_config, &icm_config.dev_handle));
    icm20948_init_i2c(&icm, &icm_config);
    CHECK_ICM(icm20948_check_id(&icm));
    ESP_ERROR_CHECK(gpio_config(&int_config));
    dmp_task = xTaskGetCurrentTaskHandle();
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_ICM20948_I2C_DMP_INT_GPIO, int1_isr, NULL));
    configure_dmp(&icm);

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        do {
            icm_20948_DMP_data_t data = {0};
            const icm20948_status_e status = inv_icm20948_read_dmp_data(&icm, &data);

            if (status != ICM_20948_STAT_OK && status != ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
                break;
            }
            if ((data.header & DMP_header_bitmap_Quat6) != 0) {
                ESP_LOGI(TAG, "Quat6: Q1=%ld Q2=%ld Q3=%ld", (long)data.Quat6.Data.Q1,
                         (long)data.Quat6.Data.Q2, (long)data.Quat6.Data.Q3);
            }
            if (status != ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
                break;
            }
        } while (true);
    }
}
