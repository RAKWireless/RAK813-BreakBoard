
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "bme280_defs.h"
#include "bme280.h"
#include "custom_board.h"
#include "rak_spi_drv.h"

struct bme280_dev dev;
		
static uint32_t bme280_spi_init(void)
{
    uint32_t err_code;
    nrf_drv_spi_config_t spi_bme_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_bme_config.ss_pin   = BME280_SPI_CS_PIN;
    spi_bme_config.miso_pin = BME280_SPI_SDO_PIN;
    spi_bme_config.mosi_pin = BME280_SPI_SDI_PIN;
    spi_bme_config.sck_pin  = BME280_SPI_SCK_PIN;

    
    err_code = rak_spi_init(&spi_bme_config);
    if(err_code != NRF_SUCCESS)
	  {
		    return err_code;
	  }
	
	  return NRF_SUCCESS;
}

void user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
		nrf_delay_ms(period);
}

int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */

    /*
     * Data on the bus should be like
     * |----------------+---------------------+-------------|
     * | MOSI           | MISO                | Chip Select |
     * |----------------+---------------------|-------------|
     * | (don't care)   | (don't care)        | HIGH        |
     * | (reg_addr)     | (don't care)        | LOW         |
     * | (don't care)   | (reg_data[0])       | LOW         |
     * | (....)         | (....)              | LOW         |
     * | (don't care)   | (reg_data[len - 1]) | LOW         |
     * | (don't care)   | (don't care)        | HIGH        |
     * |----------------+---------------------|-------------|
     */
		nrf_gpio_pin_write ( BME280_SPI_CS_PIN, 0 );
		rslt = rak_spi_read(reg_addr, reg_data, len);
		nrf_gpio_pin_write ( BME280_SPI_CS_PIN, 1 );
		
    return rslt;
}

int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */

    /*
     * Data on the bus should be like
     * |---------------------+--------------+-------------|
     * | MOSI                | MISO         | Chip Select |
     * |---------------------+--------------|-------------|
     * | (don't care)        | (don't care) | HIGH        |
     * | (reg_addr)          | (don't care) | LOW         |
     * | (reg_data[0])       | (don't care) | LOW         |
     * | (....)              | (....)       | LOW         |
     * | (reg_data[len - 1]) | (don't care) | LOW         |
     * | (don't care)        | (don't care) | HIGH        |
     * |---------------------+--------------|-------------|
     */
		nrf_gpio_pin_write ( BME280_SPI_CS_PIN, 0 );
		rslt = rak_spi_write(reg_addr, reg_data, len);
		nrf_gpio_pin_write ( BME280_SPI_CS_PIN, 1 );
    
		return rslt;
}

void print_sensor_data(struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE
        NRF_LOG_INFO( "%0.2f, %0.2f, %0.2f\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
        NRF_LOG_INFO( "%ld, %ld, %ld\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}

int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t settings_sel;
	struct bme280_data comp_data;

	/* Recommended mode of operation: Indoor navigation */
	dev->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev->settings.filter = BME280_FILTER_COEFF_16;
	dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, dev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);

//	NRF_LOG_INFO(LOG_INFO, "Temperature, Pressure, Humidity\r\n");
//	while (1) {
//		/* Delay while the sensor completes a measurement */
//		dev->delay_ms(70);
//		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
//		print_sensor_data(&comp_data);
//	}

	return rslt;
}

int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t settings_sel;
    struct bme280_data comp_data;

    /* Recommended mode of operation: Indoor navigation */
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_16X;
    dev->settings.osr_t = BME280_OVERSAMPLING_2X;
    dev->settings.filter = BME280_FILTER_COEFF_16;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, dev);

    NRF_LOG_INFO( "Temperature, Pressure, Humidity\r\n");
    /* Continuously stream sensor data */
    while (1) {
        rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
        /* Wait for the measurement to complete and print data @25Hz */
        dev->delay_ms(40);
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
        print_sensor_data(&comp_data);
    }
    return rslt;
}

uint32_t _bme280_init(void)
{
		int8_t rslt = BME280_OK;

		bme280_spi_init();
		/* Sensor_0 interface over SPI with native chip select line */
		dev.dev_id = 0;
		dev.intf = BME280_SPI_INTF;
		dev.read = user_spi_read;
		dev.write = user_spi_write;
		dev.delay_ms = user_delay_ms;

		rslt = bme280_init(&dev);
		NRF_LOG_INFO( "bme280 init ret %d\n", rslt);
		rslt = stream_sensor_data_normal_mode(&dev);
		//rslt = stream_sensor_data_forced_mode(&dev);
    return rslt;
}

uint32_t get_bme280_data(int *temp, int *humidity, int *press)
{
		int8_t rslt = BME280_OK;
		int count=3;
		struct bme280_data comp_data;
		NRF_LOG_INFO( "Temperature, Pressure, Humidity\r\n");
		while (count--) 
		{
				/* Delay while the sensor completes a measurement */
				dev.delay_ms(70);
				rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
				print_sensor_data(&comp_data);
		}
		*temp = comp_data.temperature;
		*humidity = comp_data.humidity;
		*press = comp_data.pressure;
}


void bme280_deinit()
{
		rak_spi_deinit();
}
