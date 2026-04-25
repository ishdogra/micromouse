idf_component_register(SRCS "main.c" "sm.c" "sensors.c"
                    REQUIRES esp_driver_gpio esp_timer esp_driver_i2c
                    INCLUDE_DIRS ".")
