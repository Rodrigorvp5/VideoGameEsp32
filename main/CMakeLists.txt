idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        led_strip
        ssd1306      
    PRIV_REQUIRES 
        esp_adc
        esp_driver_gpio
        spiffs
        esp_timer
        lvgl
        lvgl_esp32_drivers
)

spiffs_create_partition_image(Datos ../InternalDisk FLASH_IN_PROJECT)
