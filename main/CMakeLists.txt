idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES motor_control encoder_reader simulink_control trajectory_generator esp_driver_uart esp_driver_gpio)