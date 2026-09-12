idf_component_register(SRCS "gba_spi.c" "gba_wap.c" "gba_core1.c" "gba_core1_boot.S"
                             "gba_spi_hwmaster.c" "gba_relay.c" "gba_relay_selftest.c" "gba_busprobe.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_spi esp_driver_gpio esp_timer esp_rom soc esp_hal_gpspi
                                esp_hw_support hal spi_flash heap esp_hal_gpio)
