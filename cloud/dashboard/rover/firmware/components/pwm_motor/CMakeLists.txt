idf_component_register(SRCS "pwm_motor.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver board_pins rover_config)
