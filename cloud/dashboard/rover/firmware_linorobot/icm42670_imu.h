#ifndef ICM42670_IMU_H
#define ICM42670_IMU_H

#include <Arduino.h>
#include <Wire.h>

#include "imu_interface.h"

/* TDK ICM42670-P on the Yahboom MicroROS Board V2.0, I2C0 (SCL 39 / SDA 40).
 *
 * Ported from the ESP-IDF driver in this project's own
 * rover/firmware/components/imu_icm42670/, which booted and published telemetry
 * on this exact board.
 *
 * This gyro is load-bearing. The vendor firmware publishes an IDENTITY
 * quaternion on /odom_raw (measured over 134 messages), so there is no
 * wheel-derived heading anywhere in the system, and the golden phase6 driver's
 * +/-1-4 degree turn accuracy came entirely from integrating this sensor's Z
 * rate.
 *
 * DIAGNOSTIC MODE
 * --------------
 * 2026-08-03: the chassis was observed turning a clean 180 degrees while
 * angular_velocity.z integrated to 0.0. The accelerometer reads |a| = 9.804,
 * so I2C, the address probe and the scaling constants are all correct -- the
 * fault is specific to the gyro data path.
 *
 * The IMU sits on the ESP32's I2C bus, so it cannot be probed from Linux, and
 * Serial is the micro-ROS transport so nothing may print to it. With
 * FPMS_IMU_DIAG defined, the raw gyro registers and a config readback are
 * exported through imu_msg.orientation, which is otherwise unused (this
 * firmware never fuses an orientation and the field is left identity).
 * IMU_TWEAK is upstream's documented hook for exactly this, injected verbatim
 * just before getData() returns.
 */

#define FPMS_IMU_DIAG 1

#define ICM_ADDR_PRIMARY   0x68   /* AP_AD0 tied low */
#define ICM_ADDR_SECONDARY 0x69

#define ICM_REG_ACCEL_DATA_X1 0x0B  /* accel XYZ then gyro XYZ, big-endian */
#define ICM_REG_GYRO_DATA_X1  0x11
#define ICM_REG_PWR_MGMT0     0x1F
#define ICM_REG_GYRO_CONFIG0  0x20
#define ICM_REG_ACCEL_CONFIG0 0x21
#define ICM_REG_WHO_AM_I      0x75
#define ICM_WHOAMI_VALUE      0x67

/* PWR_MGMT0: bits[1:0] ACCEL_MODE, bits[3:2] GYRO_MODE. 0x0F = both low-noise.
 * The datasheet requires ODR/FS to be configured BEFORE leaving standby, which
 * is why the writes below are ordered config-then-enable. */
#define ICM_PWR_MGMT0_LN_BOTH 0x0F
/* GYRO_CONFIG0:  bits[6:5] FS_SEL (0 = +/-2000 dps), bits[3:0] ODR
 * ACCEL_CONFIG0: bits[6:5] FS_SEL (0 = +/-16 g),     bits[3:0] ODR
 * ODR 0x06 = 800 Hz on this part. */
#define ICM_GYRO_CONFIG0_VAL  0x06
#define ICM_ACCEL_CONFIG0_VAL 0x06

#ifdef FPMS_IMU_DIAG
/* Declared in fpms_config.h (which imu_interface.h pulls in early enough for
 * IMU_TWEAK to exist when getData() is compiled). Defined here, in the one
 * translation unit that includes this driver. */
volatile int32_t fpms_diag_gyro_z = 0;
volatile int32_t fpms_diag_accel_z = 0;
volatile int32_t fpms_diag_cfg = 0;
volatile int32_t fpms_diag_who = 0;
#endif

class ICM42670IMU : public IMUInterface
{
    private:
        /* 16-bit signed over the full-scale range chosen above. Base class
         * supplies g_to_accel_ (9.81); DEG_TO_RAD is the Arduino core macro.
         * readAccelerometer() must return m/s^2 and readGyroscope() rad/s. */
        const float accel_scale_ = 16.0f / 32768.0f;     // g per LSB
        const float gyro_scale_ = 2000.0f / 32768.0f;    // dps per LSB

        uint8_t addr_ = ICM_ADDR_PRIMARY;

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

        bool regWrite(uint8_t reg, uint8_t val)
        {
            Wire.beginTransmission(addr_);
            Wire.write(reg);
            Wire.write(val);
            return Wire.endTransmission() == 0;
        }

        bool regRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
        {
            Wire.beginTransmission(addr);
            Wire.write(reg);
            if (Wire.endTransmission(false) != 0) return false;
            if (Wire.requestFrom((int)addr, (int)len) != (int)len) return false;
            for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
            return true;
        }

        /* Registers are big-endian, unlike most MEMS parts. */
        int16_t be16(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }

        /* Gyro is read from its OWN register address rather than as an offset
         * into the accel burst. A 12-byte burst from ACCEL_DATA_X1 assumes the
         * two blocks are contiguous; reading GYRO_DATA_X1 directly removes that
         * assumption, which is one of the two candidate causes of the zero
         * readings. */
        bool readGyroRaw(int16_t *out3)
        {
            uint8_t b[6];
            if (!regRead(addr_, ICM_REG_GYRO_DATA_X1, b, sizeof(b))) return false;
            for (int i = 0; i < 3; i++) out3[i] = be16(&b[i * 2]);
            return true;
        }

        bool readAccelRaw(int16_t *out3)
        {
            uint8_t b[6];
            if (!regRead(addr_, ICM_REG_ACCEL_DATA_X1, b, sizeof(b))) return false;
            for (int i = 0; i < 3; i++) out3[i] = be16(&b[i * 2]);
            return true;
        }

#ifdef FPMS_IMU_DIAG
        /* Re-read live rather than only at startup: a register that reads back
         * correct at boot but zero later would point at the part resetting or
         * dropping out of low-noise mode, which a one-shot check cannot see. */
        void refreshCfg()
        {
            uint8_t pwr = 0, gcfg = 0, acfg = 0;
            regRead(addr_, ICM_REG_PWR_MGMT0, &pwr, 1);
            regRead(addr_, ICM_REG_GYRO_CONFIG0, &gcfg, 1);
            regRead(addr_, ICM_REG_ACCEL_CONFIG0, &acfg, 1);
            fpms_diag_cfg = (int32_t)pwr | ((int32_t)gcfg << 8) |
                            ((int32_t)acfg << 16);
        }
        uint16_t diag_tick_ = 0;
#endif

    public:
        ICM42670IMU() {}   /* must not touch Wire: constructed before setup() */

        bool startSensor() override
        {
            /* Probe both plausible addresses -- a false return here is FATAL in
             * firmware.cpp (while(1) with a 3-flash LED code), so never assume
             * the strap. */
            const uint8_t candidates[2] = {ICM_ADDR_PRIMARY, ICM_ADDR_SECONDARY};
            bool found = false;
            for (int i = 0; i < 2 && !found; i++)
            {
                uint8_t who = 0;
                if (regRead(candidates[i], ICM_REG_WHO_AM_I, &who, 1) &&
                    who == ICM_WHOAMI_VALUE)
                {
                    addr_ = candidates[i];
                    found = true;
                }
            }
            if (!found) return false;

            if (!regWrite(ICM_REG_GYRO_CONFIG0, ICM_GYRO_CONFIG0_VAL)) return false;
            if (!regWrite(ICM_REG_ACCEL_CONFIG0, ICM_ACCEL_CONFIG0_VAL)) return false;
            if (!regWrite(ICM_REG_PWR_MGMT0, ICM_PWR_MGMT0_LN_BOTH)) return false;
            /* Gyro start-up from cold is the slowest transition on this part and
             * the base class immediately averages 40 samples for its bias
             * estimate, so a dirty first read would poison the bias for the
             * whole session. The old 50 ms was likely too short. */
            delay(200);

#ifdef FPMS_IMU_DIAG
            uint8_t who = 0;
            bool ok = regRead(addr_, ICM_REG_WHO_AM_I, &who, 1);
            fpms_diag_who = (int32_t)who | ((int32_t)addr_ << 8) |
                            ((int32_t)(ok ? 1 : 0) << 16);
            refreshCfg();
#endif

            int16_t raw[3];
            return readAccelRaw(raw);
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            int16_t raw[3];
            if (readAccelRaw(raw))
            {
#ifdef FPMS_IMU_DIAG
                fpms_diag_accel_z = raw[2];
#endif
                accel_.x = raw[0] * (double)accel_scale_ * g_to_accel_;
                accel_.y = raw[1] * (double)accel_scale_ * g_to_accel_;
                accel_.z = raw[2] * (double)accel_scale_ * g_to_accel_;
            }
            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            int16_t raw[3];
            if (readGyroRaw(raw))
            {
#ifdef FPMS_IMU_DIAG
                fpms_diag_gyro_z = raw[2];
                if (++diag_tick_ >= 100) { diag_tick_ = 0; refreshCfg(); }
#endif
                gyro_.x = raw[0] * (double)gyro_scale_ * DEG_TO_RAD;
                gyro_.y = raw[1] * (double)gyro_scale_ * DEG_TO_RAD;
                gyro_.z = raw[2] * (double)gyro_scale_ * DEG_TO_RAD;
            }
            return gyro_;
        }
};

#endif
