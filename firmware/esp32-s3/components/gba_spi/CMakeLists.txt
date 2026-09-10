idf_component_register(SRCS "gba_spi.c" "gba_wap.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_spi esp_driver_gpio esp_timer esp_rom soc esp_hal_gpspi)
