/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_lbs.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_atfifo.h"
#include "nrf_delay.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "app_uart.h"


#define UART1_TX_BUF_SIZE      (256)  // uart buffer
#define UART1_RX_BUF_SIZE      (256)  // uart buffer
#define RX_DATA_FIFO_SIZE      (512)  // rx cache in a fifo

NRF_ATFIFO_DEF(m_rx_data_fifo, uint8_t, RX_DATA_FIFO_SIZE);

#define UART1_RXD     15  //port 15 is from the external processor to nRF MCU
#define UART1_TXD     17  //Port 17 is from the nRF MCU to external processor

static void uart1_event_handler(app_uart_evt_t *event)
{
    uint32_t err_code;

    switch (event->evt_type)
    {
        case APP_UART_DATA_READY:
        {
            nrf_atfifo_item_put_t fc;
            int a;
            uint8_t ok = 0;
            for (a = 0; a < 10; a++)
            {
                uint8_t *val = nrf_atfifo_item_alloc(m_rx_data_fifo, &fc);
                if (val != NULL)
                {
                    app_uart_get(val);
                    nrf_atfifo_item_put(m_rx_data_fifo, &fc);
                    ok = 1;
                    break;
                }
                else
                {
                  nrf_delay_us(3);
                }
            }

            if (ok == 0)
            {
                uint8_t read;
                app_uart_get(&read);
                NRF_LOG_ERROR("Can't allocate space in the rx_data_fifo. Dropped data: 0x%02X", (int) read);
            }
            break;
        }

        case APP_UART_COMMUNICATION_ERROR:
            //APP_ERROR_HANDLER(event->data.error_code);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(event->data.error_code);
            break;

        default:
            break;
    }
}

void main_uart1_init(void)
{
    uint32_t err_code;

    NRF_ATFIFO_INIT(m_rx_data_fifo);

    nrf_gpio_cfg(UART1_RXD, GPIO_PIN_CNF_DIR_Input, GPIO_PIN_CNF_INPUT_Connect, GPIO_PIN_CNF_PULL_Disabled, NRF_GPIO_PIN_H0H1, GPIO_PIN_CNF_SENSE_Disabled);
    nrf_gpio_cfg(UART1_TXD, GPIO_PIN_CNF_DIR_Output, GPIO_PIN_CNF_INPUT_Disconnect, GPIO_PIN_CNF_PULL_Disabled, NRF_GPIO_PIN_H0H1, GPIO_PIN_CNF_SENSE_Disabled);

    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = UART1_RXD,
        .tx_pin_no    = UART1_TXD,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = NRF_UART_BAUDRATE_115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART1_RX_BUF_SIZE,
                       UART1_TX_BUF_SIZE,
                       uart1_event_handler,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

void main_uart1_check_echo(void)
{
    uint8_t *buf = NULL;
    do
    {
        nrf_atfifo_item_get_t fc;
        buf = nrf_atfifo_item_get(m_rx_data_fifo, &fc);
        if (buf != NULL)
        {
            uint8_t val = *buf;
            nrf_atfifo_item_free(m_rx_data_fifo, &fc);
            // one byte from uart1
            uint32_t err_code;
            do
            {
                err_code = app_uart_put(val);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed sending app_uart_put, error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
    } while (buf != NULL);
}


volatile uint64_t g_ms_counter_64 = 0;

void TIMER1_IRQHandler(void)
{
    if ((NRF_TIMER1->EVENTS_COMPARE[0] != 0) && ((NRF_TIMER1->INTENSET & TIMER_INTENSET_COMPARE0_Msk) != 0))
    { 
        //NRF_TIMER2->TASKS_CAPTURE[1] = 1;
        //counter = NRF_TIMER2->CC[1];
        NRF_TIMER1->TASKS_CLEAR = 1;
        NRF_TIMER1->EVENTS_COMPARE[0] = 0;
        g_ms_counter_64++; 
    }
}

void start_timer_1(void)
{	
    NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;               // Set to Timer Mode
    NRF_TIMER1->TASKS_CLEAR = 1;
    NRF_TIMER1->PRESCALER = 0;                              //Prescaler = 0 gives 16MHz timer
    NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_16Bit;      //Set counter to 16 bit resolution
    NRF_TIMER1->CC[0] = 16000;                              //Set value for TIMER2 compare register 0
    NRF_TIMER1->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
    NVIC_EnableIRQ(TIMER1_IRQn);
    NRF_TIMER1->TASKS_START = 1;               
}

uint32_t ms_time_get(void)
{
    uint32_t v = (uint32_t)g_ms_counter_64;
    if (v == 0 && g_ms_counter_64 != (uint64_t)0 ) v = 1;
    return v;
}

