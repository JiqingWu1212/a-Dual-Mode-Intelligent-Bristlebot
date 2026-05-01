idf_component_register(SRCS "gatts_table_creat_demo.c"
                    PRIV_REQUIRES bt nvs_flash
                    PRIV_REQUIRES esp_driver_gpio
                    PRIV_REQUIRES esp_driver_ledc
                    PRIV_REQUIRES esp_driver_i2c
                    INCLUDE_DIRS ".")
