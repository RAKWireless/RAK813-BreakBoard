#include "rak_spi_drv.h"

#define SPI_BUFSIZE 512
#define SPI_TIMEOUT			10000
#define SPI_INSTANCE  1 /**< SPI instance index. */

static uint8_t   SPI_Tx_Buf[SPI_BUFSIZE];
static uint8_t   SPI_Rx_Buf[SPI_BUFSIZE];

static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

/**
 * @brief SPI user event handler.
 * @param event
 */
static void* spi_event_handler(nrf_drv_spi_evt_t const * p_event, void *p_context)
{
    spi_xfer_done = true;
    return NULL;
}

uint32_t rak_spi_init(const nrf_drv_spi_config_t *spi_config)
{
    uint32_t err_code;
    
    err_code = nrf_drv_spi_init(&spi, spi_config, (nrf_drv_spi_evt_handler_t)spi_event_handler, NULL);
    if(err_code != NRF_SUCCESS)
	  {
		    return err_code;
	  }
	
	  return NRF_SUCCESS;
}

void rak_spi_deinit()
{
		nrf_drv_spi_uninit(&spi);
}

uint32_t rak_spi_write(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint32_t err_code;
		uint32_t timeout = SPI_TIMEOUT;
	
		spi_xfer_done = false;
    SPI_Tx_Buf[0] = reg;
		memcpy(&SPI_Tx_Buf[1], data, len);
	
		err_code = nrf_drv_spi_transfer(&spi, SPI_Tx_Buf, len+1, NULL, 0);
    if(err_code != NRF_SUCCESS)
	  {
		    return err_code;
	  }
		while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
	
    return err_code;
}

uint8_t rak_spi_read(uint8_t reg, uint8_t * data, uint16_t len)
{
    uint32_t err_code;
		uint32_t timeout = SPI_TIMEOUT;
	
    spi_xfer_done = false;
    SPI_Tx_Buf[0] = reg;	
	
		err_code = nrf_drv_spi_transfer(&spi, SPI_Tx_Buf, 1, SPI_Rx_Buf, len+1);
    if(err_code != NRF_SUCCESS)
	  {
		    return err_code;
	  }
		while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
		
    memcpy(data, &SPI_Rx_Buf[1], len);
    return err_code;
}
