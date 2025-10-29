<<<<<<< HEAD
idf_component_register(
    SRCS
        "oneshot_read_main.c"
        "drv_rgb.c"
        "chromein.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_driver_ledc
        esp_adc
)
=======
idf_component_register(SRCS "main.c" "Driver_RGB.c" 
                    PRIV_REQUIRES esp_driver_ledc esp_driver_gpio
                    INCLUDE_DIRS ".")
>>>>>>> 99327688cbb496b9addba352fd53114f085097aa
