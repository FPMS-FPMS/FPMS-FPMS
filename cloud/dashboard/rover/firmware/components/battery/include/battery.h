/*
 * battery.h -- pack voltage from the on-board divider on GPIO3 (ADC1 ch2).
 *
 * The host stack subscribes /battery as std_msgs/UInt16 in DECIVOLTS
 * (value / 10 = volts). That encoding is preserved exactly; see
 * uros_node.c.
 *
 * MEASURE-ME: the divider ratio on this board is not documented anywhere
 * this author could find. rover_config's bat_divider defaults to 5.0,
 * which is the smallest ratio that keeps a fully charged 12.6 V pack
 * inside the ADC's usable range -- a plausible design value, not a known
 * one. Calibrate it with a multimeter (README).
 */
#ifndef BATTERY_H
#define BATTERY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void battery_init(void);

/* Filtered pack voltage in volts. Returns 0.0 if the ADC never came up. */
float battery_volts(void);

/* Take one reading and fold it into the filter. Call at ~1 Hz. */
void battery_sample(void);

#ifdef __cplusplus
}
#endif
#endif /* BATTERY_H */
