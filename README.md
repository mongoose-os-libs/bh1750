# BH1750 ambient light sensor native library for Mongoose OS

## Overview

BH1750 ambient light sensor native C driver for Mongoose OS

## Sensor wiring

## Usage
```c
struct sensor_bh1750* bh = sensor_bh1750_create(0x23);
float lux = sensor_bh1750_get_lux(bh,ONE_TIME_HIGH_RES_MODE_2);
printf("Light: %f lux\n", lux);

sensor_bh1750_free(bh, false);
```