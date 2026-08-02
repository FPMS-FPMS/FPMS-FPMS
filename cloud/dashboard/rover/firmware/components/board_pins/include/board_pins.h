/*
 * board_pins.h -- Yahboom MicroROS Board V2.0 (ESP32-S3) hardware pin map.
 *
 * SOURCE OF THIS MAP
 * ------------------
 * Every pin below is taken from Yahboom's own "Introduction to microROS control
 * board" documentation page, which publishes a complete peripheral/GPIO table
 * for this board:
 *   http://www.yahboom.net/public/upload/upload-html/1716275153/3.%20Introduction%20to%20microROS%20control%20board.html
 *   http://www.yahboom.net/public/upload/upload-html/1713429128/Brief%20introduction%20of%20microROS%20control%20board.html
 *
 * !!! CONFLICT WITH THE THIRD-PARTY REFERENCE -- READ THIS !!!
 * The third-party ESP-IDF tree github.com/PrwTsrt/microros_esp32_diffdrive
 * (used only as a structural reference -- see README "Attribution") assigns
 * M1 and M2 the OTHER WAY ROUND from Yahboom's documentation:
 *
 *      Yahboom doc :  M1 = GPIO4/5    M2 = GPIO15/16
 *                     H1 = GPIO6/7    H2 = GPIO47/48
 *      PrwTsrt tree:  M1 = GPIO16/15  M2 = GPIO4/5
 *                     H1 = GPIO47/48  H2 = GPIO7/6
 *
 * i.e. PrwTsrt's "M1" is Yahboom's "M2". We follow the VENDOR documentation,
 * because the silkscreen on the board's motor connectors is what the operator
 * physically plugged the motors into.
 *
 * If the rover drives with the front and rear of one side swapped, this is the
 * first thing to check. It is harmless to swap -- see MOTOR_SIDE_OF_INDEX in
 * car_motion.h -- but it must be checked on hardware before odometry is trusted.
 *
 * CORROBORATION
 * -------------
 * The motor GPIOs (4/5, 15/16, 9/10, 13/14), the encoder GPIOs (6/7, 47/48,
 * 11/12, 1/2), buzzer 46, LED 45 and the UART1 header (TX 17 / RX 18) now
 * agree across THREE independent sources: Yahboom's documentation, the
 * PrwTsrt tree, and the operator's own working Arduino sketch for this
 * board recovered from its build cache. That is as close to verified as
 * this can get without a multimeter.
 *
 * The M1/M2 ORDERING remains the one open question (see the conflict note
 * above) and it is still worth confirming with the "one motor at a time"
 * bring-up procedure in the README before the rover touches the floor.
 */
#ifndef BOARD_PINS_H
#define BOARD_PINS_H

/* ------------------------------------------------------------------ *
 * Motor H-bridge PWM inputs (2 per motor, sign-magnitude drive)
 * ------------------------------------------------------------------ */
#define PIN_M1A   4
#define PIN_M1B   5
#define PIN_M2A   15
#define PIN_M2B   16
#define PIN_M3A   9
#define PIN_M3B   10
#define PIN_M4A   13
#define PIN_M4B   14

/* ------------------------------------------------------------------ *
 * Quadrature encoder inputs (A/B per motor)
 * ------------------------------------------------------------------ */
#define PIN_H1A   6
#define PIN_H1B   7
#define PIN_H2A   47
#define PIN_H2B   48
#define PIN_H3A   11
#define PIN_H3B   12
#define PIN_H4A   1
#define PIN_H4B   2

/* ------------------------------------------------------------------ *
 * On-board peripherals
 * ------------------------------------------------------------------ */
#define PIN_BEEP        46   /* ACTIVE buzzer: high = sound, low = silent.   */

/* !!! GPIO8 IS THE SERVO S1 HEADER, AND ON THIS ROVER IT MAY BE A SPRAYER !!!
 * Yahboom document GPIO8 as PWM servo header S1. The operator's own Arduino
 * sketch for this board refers to GPIO8 as "spray". They are not in conflict
 * -- a spray pump or relay wired into the S1 header is exactly how that
 * happens -- but it means a /servo_s1 message may actuate a SPRAYER rather
 * than move a servo horn. aux_io.c therefore emits NO pulses at all until
 * the first /servo_s1 arrives, so nothing fires on boot. Confirm what is
 * physically on this header before publishing to /servo_s1. */
#define PIN_SERVO_S1    8    /* PWM servo header S1 -- see warning above     */
#define PIN_SERVO_S2    21   /* PWM servo header S2                          */
#define PIN_BAT_ADC     3    /* Battery sense divider -> ADC1 channel 2      */
#define PIN_IMU_SCL     39
#define PIN_IMU_SDA     40
#define PIN_IMU_INT     41   /* not used by this firmware (polled instead)   */
#define PIN_LED         45   /* MCU indicator LED                            */
#define PIN_KEY1        42   /* user button (unused)                         */

/* Lidar header is on UART1 (TX=17, RX=18). This firmware does NOT touch it:
 * the LiDAR on this rover is wired straight to the host SBC, and the stock
 * firmware's /scan topic was measured DEAD (all ranges 0.0). See README. */
#define PIN_LIDAR_TX    17
#define PIN_LIDAR_RX    18

/* Type-C <-> CP2102 <-> UART0. THIS IS THE micro-ROS TRANSPORT.
 * Nothing else may print to it. See sdkconfig.defaults CONFIG_ESP_CONSOLE_NONE. */
#define PIN_UART0_TX    43
#define PIN_UART0_RX    44

#endif /* BOARD_PINS_H */
