#include "mgos_bh1750.h"

#include "mgos.h"
#include "mgos_i2c.h"
#include "mgos_time.h"

#define MGOS_BH1750_CMD_POWER_DOWN 0x0
#define MGOS_BH1750_CMD_POWER_ON 0x1
#define MGOS_BH1750_CMD_RESET 0x7
#define MGOS_BH1750_CMD_CONT_HR 0x10
#define MGOS_BH1750_CMD_CONT_HR_2 0x11
#define MGOS_BH1750_CMD_CONT_LR 0x13
#define MGOS_BH1750_CMD_ONE_TIME_HR 0x20
#define MGOS_BH1750_CMD_ONE_TIME_HR_2 0x21
#define MGOS_BH1750_CMD_ONE_TIME_LR 0x23
#define MGOS_BH1750_CMD_SET_MT_H 0x40
#define MGOS_BH1750_CMD_SET_MT_L 0x60

struct mgos_bh1750 {
  struct mgos_i2c *i2c;
  uint8_t addr;
  enum mgos_bh1750_mode mode;
  uint8_t mtime;
  float lux_factor;
  int64_t meas_start, last_sample;
  uint16_t meas_time_ms;
};

struct mgos_bh1750 *mgos_bh1750_create(uint8_t addr) {
  struct mgos_i2c *i2c = mgos_i2c_get_bus(0);
  if (i2c == NULL) {
    LOG(LL_ERROR, ("Could not get global I2C instance. Is I2C enabled?"));
    return NULL;
  }
  return mgos_bh1750_create_i2c(i2c, addr);
}

// Probe for sensor. It has no identification register so we use reset behavior
// to detect.
bool mgos_bh1750_probe_addr(struct mgos_i2c *i2c, uint8_t addr) {
  uint8_t cmd, value[3];
  if (i2c == NULL) return false;
  cmd = MGOS_BH1750_CMD_POWER_DOWN;
  if (!mgos_i2c_write(i2c, addr, &cmd, 1, true)) return false;
  cmd = MGOS_BH1750_CMD_POWER_ON;
  if (!mgos_i2c_write(i2c, addr, &cmd, 1, true)) return false;
  cmd = MGOS_BH1750_CMD_RESET;
  if (!mgos_i2c_write(i2c, addr, &cmd, 1, true)) return false;
  // Read 3 bytes. First two should be 0 because we reset it,
  // the third does not exist and should read as 0xff.
  if (!mgos_i2c_read(i2c, addr, value, sizeof(value), true /* stop */)) {
    return false;
  }
  if (value[0] != 0 || value[1] != 0 || value[2] != 0xff) return false;
  cmd = MGOS_BH1750_CMD_POWER_DOWN;
  return mgos_i2c_write(i2c, addr, &cmd, 1, true);
}

uint8_t mgos_bh1750_detect_i2c(struct mgos_i2c *i2c) {
  if (mgos_bh1750_probe_addr(i2c, BH1750_ADDR_0)) {
    return BH1750_ADDR_0;
  }
  if (mgos_bh1750_probe_addr(i2c, BH1750_ADDR_1)) {
    return BH1750_ADDR_1;
  }
  return 0;
}

uint8_t mgos_bh1750_detect(void) {
  struct mgos_i2c *i2c = mgos_i2c_get_bus(0);
  if (i2c == NULL) {
    LOG(LL_ERROR, ("Could not get global I2C instance. Is I2C enabled?"));
    return 0;
  }
  return mgos_bh1750_detect_i2c(i2c);
}

bool mgos_bh1750_probe(struct mgos_bh1750 *bh) {
  return mgos_bh1750_probe_addr(bh->i2c, bh->addr);
}

struct mgos_bh1750 *mgos_bh1750_create_i2c(struct mgos_i2c *i2c, uint8_t addr) {
  if (addr != BH1750_ADDR_0 && addr != BH1750_ADDR_1) {
    LOG(LL_ERROR, ("Invalid BH1750 address %#x", addr));
    return NULL;
  }

  struct mgos_bh1750 *bh = (struct mgos_bh1750 *) calloc(1, sizeof(*bh));

  if (bh == NULL) {
    return NULL;
  }

  bh->i2c = i2c;
  bh->addr = addr;

  if (!mgos_bh1750_set_config(bh, MGOS_BH1750_MODE_POWER_DOWN,
                              MGOS_BH1750_MTIME_DEFAULT)) {
    LOG(LL_ERROR, ("Failed to init BH1750 @ %#x", bh->addr));
    free(bh);
    bh = NULL;
  }

  return bh;
}

void mgos_bh1750_free(struct mgos_bh1750 *bh) {
  if (bh == NULL) return;
  memset(bh, 0, sizeof(*bh));
  free(bh);
}

static void mgos_bh1750_calc_coeffs(enum mgos_bh1750_mode mode, int mtime,
                                    float *lux_factor_out,
                                    uint16_t *meas_time_ms) {
  float meas_time_base = 0;
  float mtime_factor = ((float) mtime / MGOS_BH1750_MTIME_DEFAULT);
  float lux_factor = (1.2f * mtime_factor);
  switch (mode) {
    case MGOS_BH1750_MODE_POWER_DOWN:
      break;
    case MGOS_BH1750_MODE_ONCE_HIGH_RES_2:
    case MGOS_BH1750_MODE_CONT_HIGH_RES_2:
      lux_factor *= 2;
      // fallthrough
    case MGOS_BH1750_MODE_ONCE_HIGH_RES:
    case MGOS_BH1750_MODE_CONT_HIGH_RES:
      meas_time_base = 120;
      break;
    case MGOS_BH1750_MODE_ONCE_LOW_RES:
    case MGOS_BH1750_MODE_CONT_LOW_RES:
      meas_time_base = 16;
      break;
  }
  *lux_factor_out = lux_factor;
  // For simplicity, we incorporate longer first sample time as meas_time_base /
  // 2 and apply it uniformly.
  *meas_time_ms =
      (uint16_t)(meas_time_base * mtime_factor + meas_time_base / 2);
}

static bool mgos_bh1750_set_mtime(struct mgos_bh1750 *bh, uint8_t mtime) {
  uint8_t cmd;
  if (mtime < MGOS_BH1750_MTIME_MIN || mtime > MGOS_BH1750_MTIME_MAX) {
    return false;
  }
  if (mtime == bh->mtime) return true;
  cmd = MGOS_BH1750_CMD_SET_MT_H | (mtime >> 5);
  if (!mgos_i2c_write(bh->i2c, bh->addr, &cmd, 1, true /* stop */)) {
    LOG(LL_ERROR, ("Could not set %s", "mtime"));
    return false;
  }
  cmd = MGOS_BH1750_CMD_SET_MT_L | (mtime & 0x1f);
  if (!mgos_i2c_write(bh->i2c, bh->addr, &cmd, 1, true /* stop */)) {
    LOG(LL_ERROR, ("Could not set %s", "mtime"));
    return false;
  }
  bh->mtime = mtime;
  return true;
}

bool mgos_bh1750_set_config(struct mgos_bh1750 *bh, enum mgos_bh1750_mode mode,
                            int mtime) {
  // Set mtime first so the measurement is restarted with new parameters.
  if (!mgos_bh1750_set_mtime(bh, mtime)) return false;
  uint8_t cmd = mode;
  if (!mgos_i2c_write(bh->i2c, bh->addr, &cmd, 1, true /* stop */)) {
    LOG(LL_ERROR, ("Could not set %s", "mode"));
    return false;
  }
  bh->mode = mode;
  if (mode != MGOS_BH1750_MODE_POWER_DOWN) {
    bh->meas_start = bh->last_sample = mgos_uptime_micros();
  }
  mgos_bh1750_calc_coeffs(bh->mode, bh->mtime, &bh->lux_factor,
                          &bh->meas_time_ms);
  return true;
}

int mgos_bh1750_read_raw(struct mgos_bh1750 *bh) {
  if (!mgos_bh1750_data_valid(bh)) {
    return -2;
  }
  uint8_t value[2];
  bool res = mgos_i2c_read(bh->i2c, bh->addr, value, 2, true /* stop */);
  if (!res) {
    LOG(LL_ERROR, ("Could not get sensor data from BH1750"));
    return -1;
  }
  bh->last_sample = mgos_uptime_micros();
  return ((value[0] << 8) | value[1]);
}

static int mgos_bh1750_calc_wait_time_ms(int64_t since, int meas_time_ms,
                                         enum mgos_bh1750_mode mode) {
  int64_t deadline = since + (meas_time_ms * 1000);
  int result = (deadline - mgos_uptime_micros()) / 1000;
  return (result >= 0 ? result : 0);
}

bool mgos_bh1750_data_valid(struct mgos_bh1750 *bh) {
  if (bh->mode == MGOS_BH1750_MODE_POWER_DOWN) return false;
  int wait_ms =
      mgos_bh1750_calc_wait_time_ms(bh->meas_start, bh->meas_time_ms, bh->mode);
  return (wait_ms <= 0);
}

float mgos_bh1750_read_lux(struct mgos_bh1750 *bh, int *raw_value_out) {
  int raw_value = mgos_bh1750_read_raw(bh);
  if (raw_value_out != NULL) *raw_value_out = raw_value;
  if (raw_value < 0) {
    return raw_value;
  }
  return raw_value / bh->lux_factor;
}

int mgos_bh1750_get_meas_time_ms(struct mgos_bh1750 *bh) {
  return bh->meas_time_ms;
}

int mgos_bh1750_get_wait_time_ms(struct mgos_bh1750 *bh) {
  return mgos_bh1750_calc_wait_time_ms(bh->last_sample, bh->meas_time_ms,
                                       bh->mode);
}

bool mgos_bh1750_init() {
  return true;
}
