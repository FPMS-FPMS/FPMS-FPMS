idf_component_register(SRCS "motor.c"
                       INCLUDE_DIRS "include"
                       REQUIRES pwm_motor encoder rover_config)
