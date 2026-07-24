idf_component_register(SRCS "main.c" "dshot_esc.c" "dshot_esc_encoder.c" "pwm_esc.c"
                    PRIV_REQUIRES esp_driver_rmt esp_driver_gpio esp_driver_ledc
                    INCLUDE_DIRS ".")
