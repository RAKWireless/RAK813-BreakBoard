#ifndef __RAK_I2C_SHT31_H__
#define __RAK_I2C_SHT31_H__

#include "stdint.h"

bool Sht31_startMeasurementLowResolution();

bool Sht31_startMeasurementHighResolution();

bool Sht31_readMeasurement_uint16_t_scale100(int16_t* humi, int16_t* temp);

bool Sht31_readMeasurement_ft(float* humi, float* temp);

#endif