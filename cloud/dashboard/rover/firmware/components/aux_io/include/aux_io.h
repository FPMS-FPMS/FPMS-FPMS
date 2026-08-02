/*
 * aux_io.h -- buzzer and the two PWM servo headers.
 *
 * These are cheap to keep and the host stack already uses them, so they
 * are kept bit-for-bit compatible with the firmware being replaced. The
 * third-party reference tree implements NONE of them; losing them was
 * listed as a real regression in research/R4_FIRMWARE.md, so they are
 * reimplemented here from the vendor's documented behaviour.
 *
 * /beep, std_msgs/UInt16, exactly as the host sends it today:
 *      0        -> off
 *      1        -> on, and stay on until told otherwise
 *      >= 10    -> beep for that many milliseconds, then stop
 *      2..9     -> treated as 10 ms (the host never sends these)
 *
 * /servo_s1, /servo_s2, std_msgs/Int32, degrees, 0..180, clamped.
 *
 * !!! S1 (GPIO8) MAY BE A SPRAYER ON THIS ROVER, NOT A SERVO. !!!
 * See board_pins.h. Both channels start with the PWM duty at zero -- no
 * pulse train at all -- so nothing is commanded anywhere until the first
 * /servo_sN message arrives. Do not "helpfully" centre them at boot.
 */
#ifndef AUX_IO_H
#define AUX_IO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_MIN_DEG    0
#define SERVO_MAX_DEG    180

void aux_io_init(void);

/* Apply the /beep convention above. */
void aux_beep_command(uint16_t value);

/* channel is 1 or 2 (S1 / S2). Degrees are clamped to [0, 180]. */
void aux_servo_set_deg(int channel, int32_t degrees);

#ifdef __cplusplus
}
#endif
#endif /* AUX_IO_H */
