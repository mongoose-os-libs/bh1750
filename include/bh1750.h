#include <stdint.h>

#ifndef _BH1750_H
#define _BH1750_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum sensor_bh1750_mode
{
    CONTINUOUS_HIGH_RES_MODE    = 0x10,
    CONTINUOUS_HIGH_RES_MODE_2  = 0x11,
    CONTINUOUS_LOW_RES_MODE     = 0x13,

    ONE_TIME_HIGH_RES_MODE      = 0x20,
    ONE_TIME_HIGH_RES_MODE_2    = 0x21,
    ONE_TIME_LOW_RES_MODE       = 0x23,
} sensor_bh1750_mode;

typedef struct sensor_bh1750 sensor_bh1750;

sensor_bh1750 *sensor_bh1750_create(uint8_t addr);
void sensor_bh1750_free(sensor_bh1750 *ds, bool power_down);

#define SENSOR_MAX_WAIT_TIME -1
#define SENSOR_TYPICAL_WAIT_TIME -2

bool sensor_bh1750_set_mode(sensor_bh1750 *bh, const sensor_bh1750_mode mode);
float sensor_bh1750_read_data(sensor_bh1750 *bh, const sensor_bh1750_mode mode);
uint8_t sensor_bh1750_get_wait_time(const sensor_bh1750_mode mode, int16_t ms_wait_time);

float sensor_bh1750_get_lux(sensor_bh1750 *bh, const sensor_bh1750_mode mode);
float sensor_bh1750_get_lux_time(sensor_bh1750 *bh, const sensor_bh1750_mode mode, int16_t ms_wait_time);

bool sensor_bh1750_power_down(sensor_bh1750 *bh);


#ifdef __cplusplus
}
#endif

#endif // _BH1750_H