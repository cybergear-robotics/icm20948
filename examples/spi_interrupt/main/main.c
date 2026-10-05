#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include "icm20948.h"
#include "icm20948_spi.h"

#define TAG "spi_interrupt"

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
    const spi_bus_config_t bus_config = {
        .miso_io_num = CONFIG_ICM20948_SPI_INT_MISO_GPIO,
        .mosi_io_num = CONFIG_ICM20948_SPI_INT_MOSI_GPIO,
        .sclk_io_num = CONFIG_ICM20948_SPI_INT_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    const spi_device_interface_config_t device_config = {
        .clock_speed_hz = 4000000,
        .mode = 0,
        .spics_io_num = CONFIG_ICM20948_SPI_INT_CS_GPIO,
        .queue_size = 1,
    };
    const gpio_config_t int_config = {
        .pin_bit_mask = 1ULL << CONFIG_ICM20948_SPI_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    icm20948_int_enable_t int_enable = {.RAW_DATA_0_RDY_EN = 1};
    icm20948_int_pin_cfg_t pin_config = {0};
    icm20948_device_t icm;
    spi_device_handle_t spi;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus_config, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &device_config, &spi));
    icm20948_init_spi(&icm, &spi);
    ESP_ERROR_CHECK(gpio_config(&int_config));
    int_queue = xQueueCreate(8, sizeof(gpio_num_t));
    configASSERT(int_queue != NULL);
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_ICM20948_SPI_INT_GPIO, int1_isr,
                                        (void *)CONFIG_ICM20948_SPI_INT_GPIO));

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
