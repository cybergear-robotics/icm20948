idf_component_register(SRCS "test_runner.c" "test_icm20948_spi.c"
                       REQUIRES driver esp_driver_gpio icm20948 unity
                       WHOLE_ARCHIVE)
