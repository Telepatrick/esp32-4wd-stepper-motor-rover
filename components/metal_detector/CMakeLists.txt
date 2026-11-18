idf_component_register(SRCS "metal_detector.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_gpio esp_driver_ledc esp_adc esp_timer esp_rom)