#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "icm20948.h"
#include "icm20948_i2c.h"

#define TAG "i2c_interrupt"

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

void app_main(void)
{
    const i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_ICM20948_I2C_INT_SDA_GPIO,
        .scl_io_num = CONFIG_ICM20948_I2C_INT_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM_20948_I2C_ADDR_AD1,
        .scl_speed_hz = 400000,
    };
    const gpio_config_t int_config = {
        .pin_bit_mask = 1ULL << CONFIG_ICM20948_I2C_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    icm20948_int_enable_t int_enable = {.RAW_DATA_0_RDY_EN = 1};
    icm20948_int_pin_cfg_t pin_config = {0};
    icm0948_config_i2c_t icm_config = {0};
    icm20948_device_t icm;
    i2c_master_bus_handle_t bus;

    ESP_ERROR_CHECK(gpio_set_direction(CONFIG_ICM20948_I2C_INT_CS_GPIO, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(CONFIG_ICM20948_I2C_INT_CS_GPIO, 1));
    ESP_ERROR_CHECK(gpio_set_direction(CONFIG_ICM20948_I2C_INT_AD0_GPIO, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(CONFIG_ICM20948_I2C_INT_AD0_GPIO, 1));
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &device_config, &icm_config.dev_handle));
    icm20948_init_i2c(&icm, &icm_config);
    ESP_ERROR_CHECK(gpio_config(&int_config));
    int_queue = xQueueCreate(8, sizeof(gpio_num_t));
    configASSERT(int_queue != NULL);
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_ICM20948_I2C_INT_GPIO, int1_isr,
                                        (void *)CONFIG_ICM20948_I2C_INT_GPIO));

    ESP_ERROR_CHECK(icm20948_check_id(&icm) == ICM_20948_STAT_OK ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(icm20948_sleep(&icm, false) == ICM_20948_STAT_OK ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(icm20948_int_pin_cfg(&icm, &pin_config, NULL) ==
                            ICM_20948_STAT_OK ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(icm20948_int_enable(&icm, &int_enable, NULL) ==
                            ICM_20948_STAT_OK ? ESP_OK : ESP_FAIL);

    while (true) {
        gpio_num_t gpio_num;
        if (xQueueReceive(int_queue, &gpio_num, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "data-ready interrupt on GPIO %d (%s)", gpio_num,
                     icm20948_data_ready(&icm) == ICM_20948_STAT_OK ? "confirmed" : "not pending");
        }
    }
}
