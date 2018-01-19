#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"


#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                           /**< UART RX buffer size. */

#if 0
void uart_event_handle(app_uart_evt_t * p_event)
{
    //static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
//            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
//            index++;

//            if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
//            {
//                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
//                NRF_LOG_HEXDUMP_DEBUG(data_array, index);

//                do
//                {
//                    uint16_t length = (uint16_t)index;
//                    err_code = ble_nus_string_send(&m_nus, data_array, &length);
//                    if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) )
//                    {
//                        APP_ERROR_CHECK(err_code);
//                    }
//                } while (err_code == NRF_ERROR_BUSY);

//                index = 0;
//            }
						uint8_t rx_data;
						app_uart_get(&rx_data);
						Gsm_RingBuf(rx_data);
				
            break;

//        case APP_UART_COMMUNICATION_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_communication);
//            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
#endif

void rak_uart_init(const app_uart_comm_params_t *uart_config)
{
    uint32_t err_code;
	
//    APP_UART_FIFO_INIT(uart_config,
//                         UART_RX_BUF_SIZE,
//                         UART_TX_BUF_SIZE,
//                         uart_event_handle,
//                         APP_IRQ_PRIORITY_LOW,
//                         err_code);

    APP_ERROR_CHECK(err_code);
}


