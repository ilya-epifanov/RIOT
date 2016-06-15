/*
 * Copyright (C) 2016 José Ignacio Alamos
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 *
 * @file
 * @author  José Ignacio Alamos <jialamos@uc.cl>
 */

#include <stdint.h>
#include <stdio.h>

#include <platform/uart.h>
#ifndef OPENTHREAD_UART_DEV
#include "uart_stdio.h"
#else
#include "periph/uart.h"
#include "ot.h"
#include "msg.h"
#include "tsrb.h"
#endif

#include "periph/gpio.h"

#ifndef OPENTHREAD_UART_BAUDRATE
#define OPENTHREAD_UART_BAUDRATE 115200
#endif

#ifndef OPENTHREAD_UART_RX_BUFSIZE
#define OPENTHREAD_UART_RX_BUFSIZE 256
#endif

static const gpio_t pin_led = GPIO_PIN(PA, 19);

#ifdef OPENTHREAD_UART_DEV
static char openthread_uart_rx_buffer_mem[OPENTHREAD_UART_RX_BUFSIZE];
tsrb_t openthread_uart_rx_buffer = TSRB_INIT(openthread_uart_rx_buffer_mem);

void openthread_uart_rx(void* context, uint8_t data)
{
  int res = tsrb_add_one(&openthread_uart_rx_buffer, (char)data);
  (void)res;
}
#endif

/* OpenThread will call this for enabling UART (required for OpenThread's CLI)*/
ThreadError otPlatUartEnable(void)
{
    gpio_init(pin_led, GPIO_OUT);
    gpio_set(pin_led);
    #ifdef OPENTHREAD_UART_DEV
    int ret = uart_init(UART_DEV(OPENTHREAD_UART_DEV),
                        OPENTHREAD_UART_BAUDRATE,
                        openthread_uart_rx,
                        NULL);

    switch (ret) {
        case 0:
            return kThreadError_None;
        case -1:
        case -2:
            return kThreadError_InvalidArgs;
        default:
            return kThreadError_Error;
    }
    #endif
    return kThreadError_None;
}

/* OpenThread will call this for disabling UART */
ThreadError otPlatUartDisable(void)
{
    #ifdef OPENTHREAD_UART_DEV
    uart_poweroff(UART_DEV(OPENTHREAD_UART_DEV));
    #endif
    return kThreadError_None;
}

/* OpenThread will call this for sending data through UART */
ThreadError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
    #ifndef OPENTHREAD_UART_DEV
    /* print UART data on screen */
    uart_stdio_write((const char*)aBuf, aBufLength);
    #else
    uart_write(UART_DEV(OPENTHREAD_UART_DEV), aBuf, aBufLength);
    #endif

    /* Tell OpenThread the sending of UART is done */
    otPlatUartSendDone();

    return kThreadError_None;
}

/** @} */
