/*
 * imu_icm42670.h -- minimal driver for the board's 6-axis IMU.
 *
 * The MicroROS Board V2.0 carries a 6-axis IMU on I2C0 (SCL=39, SDA=40).
 * It is an InvenSense ICM-42670-P: the third-party reference tree for
 * this board drives one, and the operator's own working sketch for this
 * rover names the same part.
 *
 * The PART is corroborated; the REGISTER SEQUENCE below is not -- it was
 * written from the datasheet, not from a working build. If
 * icm42670_present() returns false after init, WHO_AM_I did not read
 * 0x67, and /imu will publish zeros with the covariance marked invalid
 * rather than publishing plausible-looking nonsense. Whatever WHO_AM_I
 * actually returned is kept in icm42670_whoami() so the real part can be
 * identified without a logic analyser -- see README.
 *
 * Orientation is NOT fused. The quaternion published on /imu is identity,
 * which is what the previous firmware did and what the host stack expects
 * (it integrates angular_velocity.z itself). Adding a fusion filter here
 * would silently change the meaning of a field the stack already relies
 * on being meaningless.
 */
#ifndef IMU_ICM42670_H
#define IMU_ICM42670_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float ax, ay, az;   /* m/s^2   */
    float gx, gy, gz;   /* rad/s   */
    float temp_c;
} imu_sample_t;

/* Brings up I2C0 and configures the part. Never blocks forever and never
 * aborts: a missing IMU must not stop the rover from driving. */
void icm42670_init(void);

bool    icm42670_present(void);
uint8_t icm42670_whoami(void);

/* Reads accel+gyro+temp. Returns false if the part is absent or the
 * transaction failed; *out is zeroed in that case. */
bool icm42670_read(imu_sample_t *out);

#ifdef __cplusplus
}
#endif
#endif /* IMU_ICM42670_H */
