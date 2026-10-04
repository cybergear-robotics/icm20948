idf_component_register(SRCS "icm20948_dmp.c" "icm20948.c" "icm20948_i2c.c" "icm20948_spi.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_spi esp_driver_i2c
)
