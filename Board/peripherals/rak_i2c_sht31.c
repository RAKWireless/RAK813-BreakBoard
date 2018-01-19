#include "sht31.h"
#include "rak_i2c_drv.h"
#include "rak_i2c_sht31.h"

#define DHT31_ADDR	SHT31_ADDRESS_ARD_PIN_FLOATING

bool Sht31_startMeasurementLowResolution()
{
    uint32_t error;
    
    uint8_t tx[2] = {(SHT31_CMD_MEAS_POLLING_L >> 8), SHT31_CMD_MEAS_POLLING_L & 0xFF};
    error = rak_i2c_simple_write(DHT31_ADDR, tx, 2);
    
    return error;
}

bool Sht31_startMeasurementHighResolution()
{
    uint32_t error;
    
    uint8_t tx[2] = {(SHT31_CMD_MEAS_POLLING_H >> 8), SHT31_CMD_MEAS_POLLING_H & 0xFF};
    error = rak_i2c_simple_write(DHT31_ADDR, tx, 2);
    //printf("%d\r\n",error);
    return error;
}

bool Sht31_readMeasurement_uint16_t_scale100(int16_t* humi, int16_t* temp)
{
    uint8_t rx[6];
    uint32_t error;
    uint32_t rawTemp;
    uint32_t rawHumi;
    
    //[TEMP_LSB][TEMP_MSB][TEMP_CRC][HUMI_LSB][HUMI_MSB][HUMI_CRC]
    error = rak_i2c_simple_read(DHT31_ADDR, rx, 6);
    
    if (0 != error)
    {
        return true;
    }
    //filter Out of Spec (<2.4V) Measurements
    if (0x00 == rx[0] && 0x00 == rx[1] && 0xFF == rx[3] && 0xFF == rx[4])
    {
        return true;
    }
    
    rawTemp = rx[1] | (rx[0] << 8);
    rawTemp = (100 * rawTemp * 175) / 65535 - 45 * 100;
    rawHumi = rx[4] | (rx[3] << 8);
    rawHumi = (100 * rawHumi * 100) / 65535;
    
    *temp = (int16_t)(rawTemp);
    *humi = (int16_t)(rawHumi);
    
    return false;
}

bool Sht31_readMeasurement_ft(float* humi, float* temp)
{
    uint32_t error;
    int16_t temp_int16_t;
    int16_t humi_int16_t;
    
    error = Sht31_readMeasurement_uint16_t_scale100(&humi_int16_t, &temp_int16_t);
    
    if (0 != error)
    {
        return true;
    }
    //printf("-->%d %d\r\n", humi_int16_t, temp_int16_t);
    *temp = (float)(temp_int16_t) / 100.0f;
    *humi = (float)(humi_int16_t) / 100.0f;
    
    return false;
}

