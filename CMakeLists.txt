set(icm20948_requires driver)

if(IDF_VERSION_MAJOR GREATER_EQUAL 6)
    list(APPEND icm20948_requires esp_driver_spi esp_driver_i2c)
endif()

idf_component_register(SRCS "icm20948_dmp.c" "icm20948.c" "icm20948_i2c.c" "icm20948_spi.c"
    INCLUDE_DIRS "."
    REQUIRES ${icm20948_requires}
)
