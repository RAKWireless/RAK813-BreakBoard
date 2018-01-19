
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "lis2mdl_reg.h"
#include "custom_board.h"
#include "rak_i2c_drv.h"

#define LIS2MDL_ADDR          0x1E

lis2mdl_ctx_t dev_ctx;

static uint32_t lis2mdl_twi_init(void)
{
    uint32_t err_code;
    
    const nrf_drv_twi_config_t twi_lis_config = {
        .scl                = LIS2MDL_TWI_SCL_PIN,
        .sda                = LIS2MDL_TWI_SDA_PIN,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGHEST
    };
    
    err_code = rak_i2c_init(&twi_lis_config);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    return NRF_SUCCESS;
}

static int32_t platform_write(void *handle, uint8_t Reg, uint8_t *Bufp,
                              uint16_t len)
{
    uint32_t err_code;
    err_code = rak_i2c_write(LIS2MDL_ADDR, Reg, Bufp, len);
    return err_code;
}


static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
                             uint16_t len)
{
    uint32_t err_code;
    err_code = rak_i2c_read(LIS2MDL_ADDR, Reg, Bufp, len);
    return err_code;
}


uint32_t lis2mdl_init(void)
{
    uint32_t err_code;
    
    //≥ı ºªØTWI
    err_code = lis2mdl_twi_init();
    if(err_code != NRF_SUCCESS) return err_code;
    
    /*
    *  Initialize mems driver interface
    */
    
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    
    return NRF_SUCCESS;
}


void lis2mdl_test()
{
    /* Private variables ---------------------------------------------------------*/
#define TX_BUF_DIM          1000
    static axis3bit16_t data_raw_magnetic;
    static axis1bit16_t data_raw_temperature;
    static float magnetic_mG[3];
    static float temperature_degC;
    static uint8_t whoamI, rst;
    static uint8_t tx_buffer[TX_BUF_DIM];
    /*
    *  Check device ID
    */
    whoamI = 0;
    lis2mdl_device_id_get(&dev_ctx, &whoamI);
    if ( whoamI != LIS2MDL_ID )
        while(1); /*manage here device not found */
    /*
    *  Restore default configuration
    */
    lis2mdl_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do {
        lis2mdl_reset_get(&dev_ctx, &rst);
    } while (rst);
    /*
    *  Enable Block Data Update
    */
    lis2mdl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    /*
    * Set Output Data Rate
    */
    lis2mdl_data_rate_set(&dev_ctx, LIS2MDL_ODR_10Hz);
    /*
    * Set / Reset sensor mode
    */  
    lis2mdl_set_rst_mode_set(&dev_ctx, LIS2MDL_SENS_OFF_CANC_EVERY_ODR);
    /*
    * Enable temperature compensation
    */  
    lis2mdl_offset_temp_comp_set(&dev_ctx, PROPERTY_ENABLE);
    /*
    * Set device in continuos mode
    */   
    lis2mdl_operating_mode_set(&dev_ctx, LIS2MDL_CONTINUOUS_MODE);
    
    /*
    * Read samples in polling mode (no int)
    */
    //	uint8_t cnt=30;
    uint8_t cnt2=0;
    //  while(cnt--)
    while(1)
    {
        /*
        * Read output only if new value is available
        */
        lis2mdl_reg_t reg;
        lis2mdl_status_get(&dev_ctx, &reg.status_reg);
        
        if (reg.status_reg.zyxda)
        {
            /* Read magnetic field data */
            memset(data_raw_magnetic.u8bit, 0x00, 3*sizeof(int16_t));
            lis2mdl_magnetic_raw_get(&dev_ctx, data_raw_magnetic.u8bit);
            magnetic_mG[0] = LIS2MDL_FROM_LSB_TO_mG( data_raw_magnetic.i16bit[0]);
            magnetic_mG[1] = LIS2MDL_FROM_LSB_TO_mG( data_raw_magnetic.i16bit[1]);
            magnetic_mG[2] = LIS2MDL_FROM_LSB_TO_mG( data_raw_magnetic.i16bit[2]);
            
            sprintf((char*)tx_buffer, "Magnetic field [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
                    magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);
            //tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
            DPRINTF("%s", tx_buffer);
            
            /* Read temperature data */
            memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
            lis2mdl_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
            temperature_degC = LIS2MDL_FROM_LSB_TO_degC( data_raw_temperature.i16bit );
            
            sprintf((char*)tx_buffer, "Temperature [degC]:%6.2f\r\n", temperature_degC );
            //tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
            DPRINTF(LOG_INFO, "%s", tx_buffer);
            cnt2++;
        }
        //		else
        //		{
        //				DPRINTF(LOG_INFO, "fail...\n");
        //		}
    }
    DPRINTF("ok cnt=%d\n", cnt2);
}


void get_lis2mdl_data(char *buf)
{
    axis3bit16_t data_raw_magnetic;
    axis1bit16_t data_raw_temperature;
    float magnetic_mG[3];
    float temperature_degC;
    uint8_t whoamI, rst;
    /*
    *  Check device ID
    */
    whoamI = 0;
    lis2mdl_device_id_get(&dev_ctx, &whoamI);
    if ( whoamI != LIS2MDL_ID )
        while(1); /*manage here device not found */
    /*
    *  Restore default configuration
    */
    lis2mdl_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do {
        lis2mdl_reset_get(&dev_ctx, &rst);
    } while (rst);
    /*
    *  Enable Block Data Update
    */
    lis2mdl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    /*
    * Set Output Data Rate
    */
    lis2mdl_data_rate_set(&dev_ctx, LIS2MDL_ODR_10Hz);
    /*
    * Set / Reset sensor mode
    */  
    lis2mdl_set_rst_mode_set(&dev_ctx, LIS2MDL_SENS_OFF_CANC_EVERY_ODR);
    /*
    * Enable temperature compensation
    */  
    lis2mdl_offset_temp_comp_set(&dev_ctx, PROPERTY_ENABLE);
    /*
    * Set device in continuos mode
    */   
    lis2mdl_operating_mode_set(&dev_ctx, LIS2MDL_CONTINUOUS_MODE);
    
    /*
    * Read samples in polling mode (no int)
    */
    uint8_t cnt=10;
    uint8_t valid=0;
    while(cnt-- || valid==0)
    {
        /*
        * Read output only if new value is available
        */
        lis2mdl_reg_t reg;
        lis2mdl_status_get(&dev_ctx, &reg.status_reg);
        
        if (reg.status_reg.zyxda)
        {
            /* Read magnetic field data */
            memset(data_raw_magnetic.u8bit, 0x00, 3*sizeof(int16_t));
            lis2mdl_magnetic_raw_get(&dev_ctx, data_raw_magnetic.u8bit);
            magnetic_mG[0] = LIS2MDL_FROM_LSB_TO_mG( data_raw_magnetic.i16bit[0]);
            magnetic_mG[1] = LIS2MDL_FROM_LSB_TO_mG( data_raw_magnetic.i16bit[1]);
            magnetic_mG[2] = LIS2MDL_FROM_LSB_TO_mG( data_raw_magnetic.i16bit[2]);
            
            sprintf((char*)buf, "%4.2f\t%4.2f\t%4.2f\r\n",
                    magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);
            DPRINTF("%s", buf);
            valid = 1;
            
        }
    }
}

void lis2mdl_deinit()
{
    rak_i2c_deinit();
}
