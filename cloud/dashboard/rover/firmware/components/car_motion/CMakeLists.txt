idf_component_register(SRCS "car_motion.c"
                       INCLUDE_DIRS "include"
                       REQUIRES motor rover_config esp_timer)
