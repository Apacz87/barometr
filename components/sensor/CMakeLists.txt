idf_component_register(SRCS "sensor.c" "bmp280/bmp280.c"
                    INCLUDE_DIRS "../../include"
                    PRIV_REQUIRES esp_driver_i2c)
