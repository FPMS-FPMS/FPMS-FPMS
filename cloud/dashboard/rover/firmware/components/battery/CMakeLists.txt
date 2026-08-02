idf_component_register(SRCS "battery.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_adc board_pins rover_config)
