/*
 * BH1750 driver for Mongoose OS.
 *
 */

#include <stdbool.h>
#include <stdint.h>

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

enum mgos_bh1750_mode {
  MGOS_BH1750_MODE_POWER_DOWN = 0x0,
  // Continuous sampling modes.
  MGOS_BH1750_MODE_CONT_HIGH_RES = 0x10,
  MGOS_BH1750_MODE_CONT_HIGH_RES_2 = 0x11,
  MGOS_BH1750_MODE_CONT_LOW_RES = 0x13,
  // One-shot measurement modes.
  MGOS_BH1750_MODE_ONCE_HIGH_RES = 0x20,
  MGOS_BH1750_MODE_ONCE_HIGH_RES_2 = 0x21,
  MGOS_BH1750_MODE_ONCE_LOW_RES = 0x23,
};

#define BH1750_ADDR_0 0x23  // ADDR pin = 0
#define BH1750_ADDR_1 0x5c  // ADDR pin = 1

struct mgos_i2c;
struct mgos_bh1750;

// Detect BH1750 sensor on the bus.
// Tries both addresses and returns first that responds or 0 if none.
uint8_t mgos_bh1750_detect(void);
uint8_t mgos_bh1750_detect_i2c(struct mgos_i2c *i2c);

// Create and initialize sensor.
// Will return NULL if unsable to communicate.
struct mgos_bh1750 *mgos_bh1750_create(uint8_t addr);
struct mgos_bh1750 *mgos_bh1750_create_i2c(struct mgos_i2c *i2c, uint8_t addr);

void mgos_bh1750_free(struct mgos_bh1750 *ds);

// Measurement time constants.
#define MGOS_BH1750_MTIME_MIN 31
#define MGOS_BH1750_MTIME_DEFAULT 69
#define MGOS_BH1750_MTIME_MAX 254

// Set configuration: mode and measurement time.
// This also initiates measurement (unless mode is power down).
// Measurement time must be between 31 and 254. Longer mtime increases
// sensitivity but slows down the process and may result in overflow in high
// illumination conditions. HIGH_RES mode and MTIME_DEFAULT is the recommended
// configuration.
bool mgos_bh1750_set_config(struct mgos_bh1750 *bh, enum mgos_bh1750_mode mode,
                            int mtime);

// Returns wait time, in milliseconds, until next sample is ready.
int mgos_bh1750_get_wait_time_ms(struct mgos_bh1750 *bh);

// Returns measurement interval, in milliseconds.
// In continuous mode, data can be sampled at this interval.
int mgos_bh1750_get_meas_time_ms(struct mgos_bh1750 *bh);

// Returns true if the required amount of time has passed after start of
// measurement and valid data can be sampled.
bool mgos_bh1750_data_valid(struct mgos_bh1750 *bh);

// Reads measurement result and returns value in lux, converted according to
// current mode and sensitivity settings.
// Returns value in lux or a negative value in case of error.
// If raw is non-NULL, raw sample value is stored there.
float mgos_bh1750_read_lux(struct mgos_bh1750 *bh, int *raw_value);

// Reads measurement result and returns raw register value with no conversion.
// Returns a negative value in case of error.
int mgos_bh1750_read_raw(struct mgos_bh1750 *bh);

#ifdef __cplusplus
}
#endif
