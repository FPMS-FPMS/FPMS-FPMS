/*
 * main.c -- bring-up and the 100 Hz control task.
 *
 * TASK LAYOUT, AND WHY
 * --------------------
 * core 1, prio 10 : control task. Wheel loops, odometry, /cmd_vel watchdog.
 * core 0, prio  5 : micro-ROS task. All ROS I/O.
 *
 * The split is a safety decision, not a performance one. The watchdog that
 * stops the motors must not live inside the middleware it is protecting
 * against: if the executor blocks, the agent dies, or the USB cable is
 * pulled, the control task keeps running on its own core and zeroes the
 * motors 500 ms later. A watchdog implemented as a micro-ROS timer would
 * stop ticking in exactly the situations it exists for.
 */
#include <stdbool.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "aux_io.h"
#include "battery.h"
#include "board_pins.h"
#include "car_motion.h"
#include "imu_icm42670.h"
#include "motor.h"
#include "odometry.h"
#include "rover_config.h"
#include "uros_node.h"

#define CONTROL_TASK_CORE      1
#define CONTROL_TASK_PRIO      10
#define CONTROL_TASK_STACK     4096

static void led_init(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_LED,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&cfg);
    gpio_set_level(PIN_LED, 0);
}

static void control_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t   tick = 0;

    for (;;) {
        car_motion_step();
        odometry_step();

        /* The only status indicator this board has that does not need the
         * serial link (which belongs to micro-ROS -- see uart_transport.h).
         *   solid       : being commanded
         *   slow blink  : watchdog holding the motors down, i.e. idle or
         *                 disconnected
         * If the LED is dark, the control task itself has died. */
        if (++tick >= 25) {                  /* 4 Hz update */
            tick = 0;
            static bool phase;
            phase = !phase;
            gpio_set_level(PIN_LED, car_motion_watchdog_tripped() ? (phase ? 1 : 0) : 1);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));
    }
}

void app_main(void)
{
    /* Config first: everything below reads it. */
    rover_config_init();

    led_init();
    aux_io_init();

    /* Motors before anything that can take time, so the bridges are parked
     * in coast within a few milliseconds of boot. */
    car_motion_init();
    odometry_init();

    icm42670_init();
    battery_init();

    /* Two short beeps: the only boot confirmation available, since the
     * console is disabled so that micro-ROS can own UART0. If the rover
     * does not beep twice on power-up, it did not reach app_main. */
    aux_beep_command(80);
    vTaskDelay(pdMS_TO_TICKS(200));
    aux_beep_command(80);

    xTaskCreatePinnedToCore(control_task, "control", CONTROL_TASK_STACK, NULL,
                            CONTROL_TASK_PRIO, NULL, CONTROL_TASK_CORE);

    uros_node_start();
}
