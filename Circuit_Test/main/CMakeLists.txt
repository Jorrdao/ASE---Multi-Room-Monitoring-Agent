idf_component_register(SRCS "hello_world_main.c"
                        INCLUDE_DIRS "."
                        REQUIRES esp_driver_i2c esp_adc driver 
                                 spi_flash fatfs sdmmc st7735_driver)