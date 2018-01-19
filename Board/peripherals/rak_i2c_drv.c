#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "rak_i2c_drv.h"

#define TWI_BUFFER_SIZE     	  14 
#define TWI_TIMEOUT			10000

static const nrf_drv_twi_t m_twi_instance = NRF_DRV_TWI_INSTANCE(1);
volatile static bool twi_tx_done = false;
volatile static bool twi_rx_done = false;

static uint8_t twi_tx_buffer[TWI_BUFFER_SIZE];

static void nrf_drv_twi_event_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            switch(p_event->xfer_desc.type)
            {
              case NRF_DRV_TWI_XFER_TX:
                  twi_tx_done = true;
                  break;
              case NRF_DRV_TWI_XFER_TXTX:
                  twi_tx_done = true;
                  break;
              case NRF_DRV_TWI_XFER_RX:
                  twi_rx_done = true;
                  break;
              case NRF_DRV_TWI_XFER_TXRX:
                  twi_rx_done = true;
                  break;
              default:
                  break;
            }
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            break;
        default:
            break;
    }
}

uint32_t rak_i2c_init(const nrf_drv_twi_config_t *twi_config)
{
    uint32_t err_code;
    
    err_code = nrf_drv_twi_init(&m_twi_instance, twi_config, nrf_drv_twi_event_handler, NULL);
    if(err_code != NRF_SUCCESS)
    {
      return err_code;
    }

    nrf_drv_twi_enable(&m_twi_instance);
	
    return NRF_SUCCESS;
}

uint32_t rak_i2c_write(uint8_t twi_addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint32_t err_code;
    uint32_t timeout = TWI_TIMEOUT;
	
    twi_tx_buffer[0] = reg;
    memcpy(&twi_tx_buffer[1], data, len);
		
    twi_tx_done = false;
    err_code = nrf_drv_twi_tx(&m_twi_instance, twi_addr, twi_tx_buffer, len+1, false);
    if(err_code != NRF_SUCCESS)
    {
      twi_tx_done = false;
      return err_code;
    }
    while((!twi_tx_done) && --timeout);
    if(!timeout)
    {
      return NRF_ERROR_TIMEOUT;
    }
    twi_tx_done = false;

    return err_code;
}

uint32_t rak_i2c_simple_write(uint8_t twi_addr, uint8_t *data, uint16_t len)
{
    uint32_t err_code;
    uint32_t timeout = TWI_TIMEOUT;
		
    twi_tx_done = false;
    err_code = nrf_drv_twi_tx(&m_twi_instance, twi_addr, data, len, false);
    if(err_code != NRF_SUCCESS)
    {
      twi_tx_done = false;
      return err_code;
    }

    while((!twi_tx_done) && --timeout);
    if(!timeout) 
    {
      return NRF_ERROR_TIMEOUT;
    }
    twi_tx_done = false;

    return err_code;
}

uint32_t rak_i2c_simple_write_m(uint8_t twi_addr, uint8_t *data, uint16_t len, bool repeated)
{
    uint32_t err_code;
    uint32_t timeout = TWI_TIMEOUT;
		
    twi_tx_done = false;
    err_code = nrf_drv_twi_tx(&m_twi_instance, twi_addr, data, len, repeated);
    if(err_code != NRF_SUCCESS)
    {
      twi_tx_done = false;
      return err_code;
    }

    while((!twi_tx_done) && --timeout);
    if(!timeout) 
    {
      return NRF_ERROR_TIMEOUT;
    }
    twi_tx_done = false;

    return err_code;
}

uint8_t rak_i2c_read(uint8_t twi_addr, uint8_t reg, uint8_t * data, uint16_t len)
{
    uint32_t err_code;
    uint32_t timeout = TWI_TIMEOUT;

    twi_tx_done = false;
    err_code = nrf_drv_twi_tx(&m_twi_instance, twi_addr, &reg, 1, false);
    if(err_code != NRF_SUCCESS)
    {
      twi_tx_done = false;
      return err_code;
    }

    while((!twi_tx_done) && --timeout);
    if(!timeout) 
    {
      return NRF_ERROR_TIMEOUT;
    }
		
    twi_rx_done = false;
    err_code = nrf_drv_twi_rx(&m_twi_instance, twi_addr, data, len);
    if(err_code != NRF_SUCCESS)
    {
      twi_rx_done = false;
      return err_code;
    }

    timeout = TWI_TIMEOUT;
    while((!twi_rx_done) && --timeout);
    
    if(!timeout) 
    {
      return NRF_ERROR_TIMEOUT;
    }
    twi_rx_done = false;
		
    return err_code;
}

uint8_t rak_i2c_simple_read(uint8_t twi_addr, uint8_t * data, uint16_t len)
{
    uint32_t err_code;
    uint32_t timeout = TWI_TIMEOUT;
		
    twi_rx_done = false;
    err_code = nrf_drv_twi_rx(&m_twi_instance, twi_addr, data, len);
    if(err_code != NRF_SUCCESS)
    {
      twi_rx_done = false;
      return err_code;
    }

    timeout = TWI_TIMEOUT;
    while((!twi_rx_done) && --timeout);
    if(!timeout) 
    {
      return NRF_ERROR_TIMEOUT;
    }
    twi_rx_done = false;
		
    return err_code;
}

void rak_i2c_deinit()
{
    nrf_drv_twi_uninit(&m_twi_instance);
}

