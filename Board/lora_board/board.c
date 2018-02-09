#include "board.h"
#include "nrf52.h"

#include "app_config.h"
#include "app_timer.h"

#include "nrf_drv_gpiote.h"
#include "nrf_drv_wdt.h"
#include "mem_manager.h"
#include "nrf_log_ctrl.h"

#include <string.h>

#define BATTERY_MAX_LEVEL				4150 // mV
#define BATTERY_MIN_LEVEL				3200 // mV
#define BATTERY_SHUTDOWN_LEVEL				3100 // mV

static uint16_t BatteryVoltage = BATTERY_MAX_LEVEL;

static bool IsExtPower = false;

/*!
* LED GPIO pins objects
*/
Gpio_t Led1;

uint8_t GetBoardPowerSource( void )
{
    if ( IsExtPower )
        return USB_POWER;
    return BATTERY_POWER;
}

uint16_t BoardGetBatteryVoltage( void )
{
    return BatteryVoltage;
}

uint16_t BoardBatteryMeasureVolage( void )
{
    return BatteryVoltage;
}

uint8_t BoardGetBatteryLevel( void )
{
    uint8_t batteryLevel = 0;
    
    BatteryVoltage = BoardBatteryMeasureVolage( );
    
    if ( GetBoardPowerSource( ) == USB_POWER )
    {
        batteryLevel = 0;
    }
    else
    {
        if ( BatteryVoltage >= BATTERY_MAX_LEVEL )
        {
            batteryLevel = 254;
        }
        else if ( ( BatteryVoltage > BATTERY_MIN_LEVEL ) && ( BatteryVoltage < BATTERY_MAX_LEVEL ) )
        {
            batteryLevel = ( ( 253 * ( BatteryVoltage - BATTERY_MIN_LEVEL ) ) / ( BATTERY_MAX_LEVEL - BATTERY_MIN_LEVEL ) ) + 1;
        }
        else if ( ( BatteryVoltage > BATTERY_SHUTDOWN_LEVEL ) && ( BatteryVoltage <= BATTERY_MIN_LEVEL ) )
        {
            batteryLevel = 1;
        }
        else // if ( BatteryVoltage <= BATTERY_SHUTDOWN_LEVEL )
        {
            batteryLevel = 255;
        }
    }
    return batteryLevel;
}

uint32_t BoardGetRandomSeed( void )
{
    uint32_t seed = NRF_FICR->DEVICEID[0];
    return seed;
}

void BoardGetUniqueId( uint8_t *id )
{
    uint32_t did[2];
    did[0] = NRF_FICR->DEVICEID[0];
    did[1] = NRF_FICR->DEVICEID[1];
    memcpy(id, did, 8);
}

/*!
* Nested interrupt counter.
*
* \remark Interrupt should only be fully disabled once the value is 0
*/
static int IrqNestLevel = 0;

void BoardDisableIrq( void )
{
    __disable_irq( );
    IrqNestLevel++;
}

void BoardEnableIrq( void )
{
    IrqNestLevel--;
    if( IrqNestLevel == 0 )
    {
        __enable_irq( );
    }
}

void BoardInitPeriph( void )
{
    //Gpio_t ioPin;
    
    // Init the GPIO pins
    //GpioInit( &ioPin, GPS_POWER_ON_PIN, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &Button1, BUTTON_1, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &Button2, BUTTON_2, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    
    // Switch LED 1, 2 OFF
    GpioWrite( &Led1, 1 );;
}

void BoardInitMcu( void )
{
    ret_code_t err_code;
    
    //	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    
    /* Initialize */
    //	err_code = app_timer_init();
    //	APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
    
    //	err_code = nrf_mem_init();
    //	APP_ERROR_CHECK(err_code);
    
    RtcInit();
    
    SX1276IoInit( );
}

void BoardProcess( void )
{
    while (NRF_LOG_PROCESS());
}

