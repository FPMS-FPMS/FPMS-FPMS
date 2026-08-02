/*
 * encoder.h -- four quadrature encoders on the ESP32-S3 PCNT peripheral.
 *
 * x4 decoding (both edges of both channels), which is what makes the
 * vendor's "13 lines x 20 reduction x 4 edge detection = 1040 counts per
 * wheel revolution" arithmetic come out. If you change the decoding you
 * must change rover_config's enc_counts_per_rev by the same factor.
 *
 * ESP32-S3 has exactly 4 PCNT units of 2 channels each: an exact fit for
 * 4 quadrature encoders, with nothing left over.
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_COUNT 4

void encoder_init(void);

/* Counts since the previous call, per motor, and accumulate into the
 * 64-bit totals. Call this exactly once per control period from exactly
 * one task -- it is a read-and-clear, so a second caller would silently
 * steal the first caller's counts.
 *
 * `delta` must point at ENCODER_COUNT int32_t. */
void encoder_sample(int32_t *delta);

/* Monotonic 64-bit accumulation of everything encoder_sample() has seen.
 * Never cleared. Exported over ROS parameters (enc1..enc4) so wheel
 * geometry can be calibrated by pushing the rover a measured distance and
 * reading the ticks -- see README. */
int64_t encoder_total(int idx);

#ifdef __cplusplus
}
#endif
#endif /* ENCODER_H */
