idf_component_register(SRCS "motor_control.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_ledc)