idf_component_register(
    SRCS "main.c" "dshot600.c" "esc_telemetry.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_rmt esp_driver_uart freertos esp_timer log
)
