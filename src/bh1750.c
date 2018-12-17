#include <mgos.h>
#include "bh1750.h"
#include "mgos_i2c.h"

struct sensor_bh1750 {
  uint8_t addr;
  struct mgos_i2c *i2c;
};

sensor_bh1750 *sensor_bh1750_create(uint8_t addr)
{
  // Is I2C enabled?
  if (!mgos_sys_config_get_i2c_enable()) 
  {
    LOG(LL_INFO, ("I2C is disabled."));
    return NULL;
  }

  struct mgos_i2c *i2c = mgos_i2c_get_global();
  if (NULL == i2c) 
  {
    LOG(LL_INFO, ("Could not get i2c global instance"));
    return NULL;
  }

  sensor_bh1750 *bh =
      (sensor_bh1750 *) calloc(1, sizeof(sensor_bh1750));

  if (NULL == bh) 
  {
    LOG(LL_INFO, ("Could not allocate sensor_bh1750 structure."));
    return NULL;
  }

  bh->addr = addr;
  bh->i2c = i2c;

  return bh;
}

void sensor_bh1750_free(sensor_bh1750 *bh, bool power_down)
{
  if (NULL != bh) 
  {
    if (power_down)
      sensor_bh1750_power_down(bh);

    free(bh);
    bh = NULL;
  }
}

bool sensor_bh1750_power_down(sensor_bh1750 *bh)
{
  uint8_t power_down_command = 0x0;
  bool res = mgos_i2c_write(bh->i2c, bh->addr, (void*)&power_down_command, 1, true);
  if (!res)
  {
    LOG(LL_INFO, ("Could not power down BH1750"));
    return -1;
  }

  return true;
}

bool sensor_bh1750_set_mode(sensor_bh1750 *bh, const sensor_bh1750_mode mode)
{
  bool res = mgos_i2c_write(bh->i2c, bh->addr, (void*)&mode, 1, true);
  if (!res)
  {
    LOG(LL_INFO, ("Could not init I2C communication with BH1750"));
    return -1;
  }

  return true;
}

float sensor_bh1750_read_data(sensor_bh1750 *bh, const sensor_bh1750_mode mode)
{
  uint16_t value = -1;
  bool res = mgos_i2c_read(bh->i2c, bh->addr, (void*)&value, 2, true);
  if (!res)
  {
    LOG(LL_INFO, ("Could not get sensor data from BH1750"));
    return -1;
  }

  value = value<<8 | value>>8;

  float finalValue = value / 1.2;
  if (mode == CONTINUOUS_HIGH_RES_MODE_2 || mode == ONE_TIME_HIGH_RES_MODE_2)
    finalValue /= 2;

  return finalValue;
}

float sensor_bh1750_get_lux(sensor_bh1750 *bh, const sensor_bh1750_mode mode)
{
  return sensor_bh1750_get_lux_time(bh, mode, SENSOR_TYPICAL_WAIT_TIME);
}

uint8_t sensor_bh1750_get_wait_time(const sensor_bh1750_mode mode, int16_t ms_wait_time)
{
  if ( ms_wait_time < 0)
  {
    bool waitMaxTime = ms_wait_time == SENSOR_MAX_WAIT_TIME;
    if (mode == ONE_TIME_LOW_RES_MODE || mode == CONTINUOUS_LOW_RES_MODE)
      ms_wait_time = waitMaxTime ? 24 : 16;
    else
      ms_wait_time = waitMaxTime ? 180 : 120;
  }

  return ms_wait_time;
}

float sensor_bh1750_get_lux_time(sensor_bh1750 *bh, const sensor_bh1750_mode mode, int16_t ms_wait_time)
{
  sensor_bh1750_set_mode(bh, mode);
  ms_wait_time = sensor_bh1750_get_wait_time(mode, ms_wait_time); 

  // TODO: Should use timer and callback instead?
  mgos_msleep(ms_wait_time);

  return sensor_bh1750_read_data( bh, mode);
}


bool mgos_bh1750_init() {
  return true;
}