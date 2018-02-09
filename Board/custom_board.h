#ifndef __BSP_H__
#define __BSP_H__

#include "nrf_gpio.h"
#include "app_uart.h"
#include "app_error.h"
#include "SEGGER_RTT.h"
#include "nrf_log.h"

#include "lis3dh_driver.h"
#include "rak_i2c_gps_max7.h"

/*
		UART PIN Assignment
		UART_TXD_PIN		--	P0.28
		UART_RXD_PIN		--	P0.29

*/

#define             UART_TXD_PIN                        29
#define             UART_RXD_PIN                        28
#define             UART_RTS_PIN                        0
#define             UART_CTS_PIN                        0


/*
		GPS PIN Assignment
		GPS_STANDBY		--	P0.07
		GPS_TXD			--	P0.08
		GPS_RXD		--	P0.09(nfc default)
		GPS_PWR_ON		--	P0.10
		GPS_RESET		--	P0.31

*/
#define             GPS_STANDBY_PIN                    7
#define             GPS_TXD_PIN                        8
#define             GPS_RXD_PIN                        9
#define 	    GPS_PWR_ON_PIN	        	31
#define             GPS_RESET_PIN                        31

#define             GPS_PWR_ON                     nrf_gpio_pin_write ( GPS_PWR_ON_PIN, 1 )
#define             GPS_PWR_OFF                      nrf_gpio_pin_write ( GPS_PWR_ON_PIN, 0 )

#define             GPS_RESET_HIGH                           nrf_gpio_pin_write ( GPS_RESET_PIN, 1 )
#define             GPS_RESET_LOW                            nrf_gpio_pin_write ( GPS_RESET_PIN, 0 )


/*
		I2C PIN Assignment
		
		lis3dh,sht31,oled,gps
		SCL		--	P0.16
		SDA		--	P0.15

*/
#define             DEV_TWI_SCL_PIN                        16
#define             DEV_TWI_SDA_PIN                        15

/*
		lis3dh PIN Assignment
		LIS3DH_SCL		--	P0.18
		LIS3DH_SDA		--	P0.19
		LIS3DH_INT1		--	P0.25
		LIS3DH_RES		--	P0.26
		LIS3DH_INT2		--	P0.27
		
*/
#define             LIS3DH_TWI_SCL_PIN                     16
#define             LIS3DH_TWI_SDA_PIN                     15
#define             LIS3DH_INT1_PIN                        25
#define 	    LIS3DH_RES_PIN			   26
#define             LIS3DH_INT2_PIN                        27

/*
		lis2mdl PIN Assignment
		LIS2MDL_SCL		--	P0.11
		LIS2MDL_SDA		--	P0.13
		LIS2MDL_INT		--	P0.16
		
*/
#define             LIS2MDL_TWI_SCL_PIN                    11
#define             LIS2MDL_TWI_SDA_PIN                    13
#define             LIS2MDL_INT_PIN                        16


/*
		bme280 PIN Assignment
		BME_CS		--	P0.02
		BME_SDI		--	P0.03
		BME_SCK		--	P0.04
		BME_SDO		--	P0.05
		
*/
//#define             BME280_SPI_CS_PIN                         2
//#define             BME280_SPI_SDI_PIN                        3
//#define             BME280_SPI_SCK_PIN                        4
//#define             BME280_SPI_SDO_PIN                        5


/*
		OPT3001 PIN Assignment
		OPT_SDA		--	P0.21
		OPT_INT		--	P0.22
		OPT_SCL		--	P0.23
		
*/
//#define             OPT3001_TWI_SDA_PIN                        21
//#define             OPT3001_INT_PIN                            22
//#define             OPT3001_TWI_SCL_PIN                        23


// LEDs BUTTONs definitions 
#define LEDS_NUMBER    1
#define LEDS_ACTIVE_STATE 0
#define LED_1                                       25
#define LED_2                                       26
#define LEDS_LIST { LED_1 }

#define BUTTONS_NUMBER 1
#define BUTTONS_ACTIVE_STATE 0
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP
#define BUTTON_1                                    27
#define BUTTON_2                                    24
#define BUTTONS_LIST { BUTTON_1 }
#define BSP_BUTTON_0   BUTTON_1

/*!
 * LED GPIO pins objects
 */

typedef struct{
        int temp_value;
        int humidity_value;
        int gas_value;
        int barometer_value;
        float light_value;
        float latitude;
        float longitude;
        int acceleration;
        int compass_value;
        int tilt;
        char ble_uuid[16];
        char sim_imei[16];
        char date[30];
        int status;
        int battery;
}tracker_data_t;

typedef struct {
        uint8_t sof;
        uint8_t dev_eui[8];
        uint8_t app_eui[8];
        uint8_t app_key[16];
        uint32_t dev_addr;
        uint8_t nwkskey[16];
        uint8_t appskey[16];
} lora_cfg_t;


typedef struct {
        uint8_t bleName[16];
        uint8_t bleMac[6];
        int8_t  bleRssi;
} Ble_scanRsp_t;


extern lora_cfg_t     g_lora_cfg;

typedef struct {
        double gps_longitude;
        double gps_latitude;
        uint16_t gps_altitude;
        float HT_temperature;
        float HT_humidity;
        int MEMS_X;
        int MEMS_Y;
        int MEMS_Z;
        uint8_t battery;
} peripherals_data;
/*
*********************************************************************************************************
*                                             LOG 
*********************************************************************************************************
*/
#define     LOG_NONE     (0x00UL)
#define     LOG_ERROR    (0x01UL)
#define     LOG_WARN     (0x02UL)
#define     LOG_INFO     (0x04UL)
#define     LOG_DEBUG    (0x08UL)
#define     LOG_TRACE    (0x10UL)

#define     G_DEBUG  (LOG_NONE | LOG_ERROR | LOG_WARN | LOG_INFO | LOG_DEBUG )     
//#define     G_DEBUG  (LOG_NONE)     
#define     LOG_LEVEL_CHECK(level)      (G_DEBUG & level)



static inline char* log_level_str(uint8_t level)
{
    char* string;
    switch(level) 
    {
      case LOG_ERROR:
              string ="ERROR";
          break;
      case LOG_WARN:
              string ="WARN";
          break;			
      case LOG_INFO:
              string ="INFO";
          break;		
      case LOG_DEBUG:
              string ="DEBUG";
          break;			
      case LOG_TRACE:
              string ="TRACE";
          break;		
      default:
          break;				
    }
    return string;
}

//#define DEBUG

#ifdef DEBUG
static const char* clean_filename(const char* path)
{
  const char* filename = path + strlen(path); 
  while(filename > path)
  {
    if(*filename == '/' || *filename == '\\')
    {
      return filename + 1;
    }
    filename--;
  }
  return path;
}
#endif

#endif