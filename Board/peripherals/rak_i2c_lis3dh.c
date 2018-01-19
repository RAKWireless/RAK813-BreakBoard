
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "custom_board.h"
#include "rak_i2c_drv.h"
#include "lis3dh_driver.h"

#define LIS3DH_ADDR          0x19

static uint32_t lis3dh_twi_init(void)
{
    uint32_t err_code;
    
    const nrf_drv_twi_config_t twi_lis_config = {
        .scl                = LIS3DH_TWI_SCL_PIN,
        .sda                = LIS3DH_TWI_SDA_PIN,
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

uint32_t nrf_drv_lis3dh_write_reg(uint8_t reg, uint8_t data)
{
    uint32_t err_code;
    err_code = rak_i2c_write(LIS3DH_ADDR, reg, &data, 1);
    return err_code;
}


uint8_t nrf_drv_lis3dh_read_reg(uint8_t reg, uint8_t * p_data)
{
    uint32_t err_code;
    err_code = rak_i2c_read(LIS3DH_ADDR, reg, p_data, 1);
    return err_code;
}

uint32_t lis3dh_init(void)
{
    uint32_t err_code;
    //uint8_t id =0;
    
    //≥ı ºªØTWI
    err_code = lis3dh_twi_init();
    if(err_code != NRF_SUCCESS) return err_code;
    
    return NRF_SUCCESS;
}


void lis3dh_test()
{
    uint8_t response;
    AxesRaw_t data;
    uint8_t position=0, old_position=0;
    //Inizialize MEMS Sensor
    //set ODR (turn ON device)
    response = LIS3DH_SetODR(LIS3DH_ODR_100Hz);
    if(response==1){
        NRF_LOG_INFO("%s", "\n\rSET_ODR_OK    \n\r\0");
    }
    //set PowerMode 
    response = LIS3DH_SetMode(LIS3DH_NORMAL);
    if(response==1){
        NRF_LOG_INFO("%s", "SET_MODE_OK     \n\r\0");
    }
    //set Fullscale
    response = LIS3DH_SetFullScale(LIS3DH_FULLSCALE_2);
    if(response==1){
        NRF_LOG_INFO("%s", "SET_FULLSCALE_OK\n\r\0");
    }
    //set axis Enable
    response = LIS3DH_SetAxis(LIS3DH_X_ENABLE | LIS3DH_Y_ENABLE | LIS3DH_Z_ENABLE);
    if(response==1){
        NRF_LOG_INFO("%s", "SET_AXIS_OK     \n\r\0");
    }
    
    /*********************/  
    /******Example 1******/ 
#define __EXAMPLE1__H
#ifdef __EXAMPLE1__H 
    uint8_t cnt=30;
    while(cnt--){
        //get Acceleration Raw data  
        response = LIS3DH_GetAccAxesRaw(&data);
        if(response==1){
            NRF_LOG_INFO("X=%6d Y=%6d Z=%6d \r\n", data.AXIS_X, data.AXIS_Y, data.AXIS_Z);
            old_position = position;
        }
    }
#endif /* __EXAMPLE1__H  */ 
    
    
    //  i8_t tempData;
    // LIS3DH_SetADCAux(MEMS_ENABLE);
    // LIS3DH_SetBDU(MEMS_ENABLE);
    // LIS3DH_SetTemperature(MEMS_ENABLE);
    // while(1)
    // {
    //		  response = LIS3DH_GetTempRaw(&tempData);
    //  if(response==1){
    //		SEGGER_RTT_printf(0,"%x\n", tempData);
    //  }
    // }
}

void lis3dh_setup()
{	
    uint8_t response;
    //Inizialize MEMS Sensor
    //set ODR (turn ON device)
    response = LIS3DH_SetODR(LIS3DH_ODR_100Hz);
    if(response==1){
        NRF_LOG_INFO("%s", "\n\rSET_ODR_OK    \n\r\0");
    }
    //set PowerMode 
    response = LIS3DH_SetMode(LIS3DH_NORMAL);
    if(response==1){
        NRF_LOG_INFO("%s", "SET_MODE_OK     \n\r\0");
    }
    //set Fullscale
    response = LIS3DH_SetFullScale(LIS3DH_FULLSCALE_2);
    if(response==1){
        NRF_LOG_INFO("%s", "SET_FULLSCALE_OK\n\r\0");
    }
    //set axis Enable
    response = LIS3DH_SetAxis(LIS3DH_X_ENABLE | LIS3DH_Y_ENABLE | LIS3DH_Z_ENABLE);
    if(response==1){
        NRF_LOG_INFO("%s", "SET_AXIS_OK     \n\r\0");
    }
}

void get_lis3dh_data(int *x)
{
    uint8_t response;
    AxesRaw_t data;
    uint8_t position=0, old_position=0;
    //Inizialize MEMS Sensor
    //set ODR (turn ON device)
    response = LIS3DH_SetODR(LIS3DH_ODR_100Hz);
    if(response==1){
        NRF_LOG_INFO("%s", "\n\rSET_ODR_OK    \n\r\0");
    }
    //set PowerMode 
    response = LIS3DH_SetMode(LIS3DH_NORMAL);
    if(response==1){
        NRF_LOG_INFO("%s", "SET_MODE_OK     \n\r\0");
    }
    //set Fullscale
    response = LIS3DH_SetFullScale(LIS3DH_FULLSCALE_2);
    if(response==1){
        NRF_LOG_INFO("%s", "SET_FULLSCALE_OK\n\r\0");
    }
    //set axis Enable
    response = LIS3DH_SetAxis(LIS3DH_X_ENABLE | LIS3DH_Y_ENABLE | LIS3DH_Z_ENABLE);
    if(response==1){
        NRF_LOG_INFO("%s", "SET_AXIS_OK     \n\r\0");
    }
    
    uint8_t cnt=10;
    while(cnt--)
    {
        //get Acceleration Raw data  
        response = LIS3DH_GetAccAxesRaw(&data);
        if(response==1)
        {
            NRF_LOG_INFO("X=%6d Y=%6d Z=%6d \r\n", data.AXIS_X, data.AXIS_Y, data.AXIS_Z);
            old_position = position;
        }
    }
    *x = data.AXIS_X;
    
}

void lis3dh_deinit()
{
    rak_i2c_deinit();
}

