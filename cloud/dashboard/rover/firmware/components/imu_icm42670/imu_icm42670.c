#include "imu_icm42670.h"

#include <math.h>
#include <string.h>

#include "board_pins.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Legacy driver/i2c.h rather than the newer driver/i2c_master.h: the legacy
 * API is present and functional across every ESP-IDF from v5.0 to v6.0,
 * whereas i2c_master.h only exists from v5.2. It emits a deprecation
 * warning on newer IDFs; that is expected and harmless. */

#define I2C_PORT            I2C_NUM_0
#define I2C_FREQ_HZ         400000
#define I2C_TIMEOUT_MS      50

/* AP_AD0 tied low. 0x69 is the alternative if the board strapped it high. */
#define ICM_ADDR_PRIMARY    0x68
#define ICM_ADDR_SECONDARY  0x69

/* ICM-42670-P user bank 0 register map. */
#define REG_TEMP_DATA1      0x09
#define REG_ACCEL_DATA_X1   0x0B   /* 12 bytes: accel XYZ then gyro XYZ, big-endian */
#define REG_PWR_MGMT0       0x1F
#define REG_GYRO_CONFIG0    0x20
#define REG_ACCEL_CONFIG0   0x21
#define REG_WHO_AM_I        0x75
#define ICM42670_WHOAMI     0x67

/* PWR_MGMT0: gyro low-noise (0b11 << 2) | accel low-noise (0b11). */
#define PWR_MGMT0_LN_BOTH   0x0F

/* GYRO_CONFIG0:  FS_SEL = 0 (+/-2000 dps), ODR = 0x06 (200 Hz)
 * ACCEL_CONFIG0: FS_SEL = 0 (+/-16 g),     ODR = 0x06 (200 Hz)
 * 200 Hz is comfortably above the 25 Hz publish rate, so /imu never
 * repeats a sample. */
#define GYRO_CONFIG0_VAL    0x06
#define ACCEL_CONFIG0_VAL   0x06

#define GYRO_FS_DPS         2000.0f
#define ACCEL_FS_G          16.0f
#define GRAVITY_MPS2        9.80665f
#define DEG_TO_RAD          0.017453292519943295f

static uint8_t s_addr    = ICM_ADDR_PRIMARY;
static uint8_t s_whoami  = 0x00;
static bool    s_present = false;

static esp_err_t reg_read(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT, addr, &reg, 1, buf, len,
                                        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t reg_write(uint8_t addr, uint8_t reg, uint8_t val)
{
    const uint8_t tx[2] = { reg, val };
    return i2c_master_write_to_device(I2C_PORT, addr, tx, sizeof(tx),
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

void icm42670_init(void)
{
    const i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_IMU_SDA,
        .scl_io_num       = PIN_IMU_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    if (i2c_param_config(I2C_PORT, &cfg) != ESP_OK) {
        return;
    }
    if (i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK) {
        return;
    }

    /* Probe both plausible addresses and record whatever WHO_AM_I actually
     * says, so a mismatch is diagnosable from the host instead of just
     * being a dead topic. */
    const uint8_t candidates[2] = { ICM_ADDR_PRIMARY, ICM_ADDR_SECONDARY };
    for (int i = 0; i < 2; ++i) {
        uint8_t who = 0;
        if (reg_read(candidates[i], REG_WHO_AM_I, &who, 1) == ESP_OK && who != 0x00 && who != 0xFF) {
            s_addr   = candidates[i];
            s_whoami = who;
            if (who == ICM42670_WHOAMI) {
                s_present = true;
            }
            break;
        }
    }

    if (!s_present) {
        return;
    }

    (void)reg_write(s_addr, REG_PWR_MGMT0, PWR_MGMT0_LN_BOTH);
    /* The datasheet requires a settling gap after leaving off mode before
     * the config registers are written. 20 ms is generous. */
    vTaskDelay(pdMS_TO_TICKS(20));
    (void)reg_write(s_addr, REG_GYRO_CONFIG0,  GYRO_CONFIG0_VAL);
    (void)reg_write(s_addr, REG_ACCEL_CONFIG0, ACCEL_CONFIG0_VAL);
    vTaskDelay(pdMS_TO_TICKS(20));
}

bool    icm42670_present(void) { return s_present; }
uint8_t icm42670_whoami(void)  { return s_whoami;  }

static inline int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

bool icm42670_read(imu_sample_t *out)
{
    if (!out) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    if (!s_present) {
        return false;
    }

    /* One burst: temperature (2 bytes) then accel XYZ and gyro XYZ. */
    uint8_t raw[14];
    if (reg_read(s_addr, REG_TEMP_DATA1, raw, sizeof(raw)) != ESP_OK) {
        return false;
    }

    const int16_t t  = be16(&raw[0]);
    const int16_t ax = be16(&raw[2]);
    const int16_t ay = be16(&raw[4]);
    const int16_t az = be16(&raw[6]);
    const int16_t gx = be16(&raw[8]);
    const int16_t gy = be16(&raw[10]);
    const int16_t gz = be16(&raw[12]);

    const float a_lsb = (ACCEL_FS_G * GRAVITY_MPS2) / 32768.0f;
    const float g_lsb = (GYRO_FS_DPS * DEG_TO_RAD)  / 32768.0f;

    out->ax = (float)ax * a_lsb;
    out->ay = (float)ay * a_lsb;
    out->az = (float)az * a_lsb;
    out->gx = (float)gx * g_lsb;
    out->gy = (float)gy * g_lsb;
    out->gz = (float)gz * g_lsb;
    /* Datasheet transfer function for the on-die temperature sensor. */
    out->temp_c = ((float)t / 128.0f) + 25.0f;

    return true;
}
