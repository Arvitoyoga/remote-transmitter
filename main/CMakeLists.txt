if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.3")
    set(DRIVER_REQS driver esp_driver_gpio esp_driver_i2c esp_driver_uart)
else()
    set(DRIVER_REQS driver)
endif()

idf_component_register(SRCS "main.c" "modules/utils.c" "modules/filters.c" "modules/imu.c" "modules/elrs.c"
                    INCLUDE_DIRS "."
                    REQUIRES ${DRIVER_REQS} esp_timer button mpu6050)
