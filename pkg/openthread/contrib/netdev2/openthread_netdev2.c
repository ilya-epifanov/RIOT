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

#include "ot.h"
#include "msg.h"
#include "openthread.h"
#include "platform/alarm.h"
#include "platform/uart.h"
#include "periph/gpio.h"
#include <cli/cli-uart.h>
#include <assert.h>
#include "openthread-tasklet.h"

#ifdef MODULE_OPENTHREAD_NCP
#include <ncp/ncp.h>
#endif

#if defined(MODULE_OPENTHREAD_NCP) || defined(MODULE_OPENTHREAD_CLI)
#ifdef OPENTHREAD_UART_DEV
#define _OPENTHREAD_DIRECT_UART
#endif
#endif

#ifdef _OPENTHREAD_DIRECT_UART
#include "xtimer.h"
#include "tsrb.h"
extern tsrb_t openthread_uart_rx_buffer;
#endif

#define ENABLE_DEBUG (0)
#include "debug.h"

#include <errno.h>
#include "random.h"

#define OPENTHREAD_QUEUE_LEN      (8)

static const gpio_t pin_led = GPIO_PIN(PA, 19);

static msg_t _queue[OPENTHREAD_QUEUE_LEN];
#if (defined(MODULE_OPENTHREAD_CLI) || defined(MODULE_OPENTHREAD_NCP)) && defined(OPENTHREAD_UART_DEV)
static msg_t uart_timer_msg = { KERNEL_PID_UNDEF, OPENTHREAD_SERIAL_MSG_TYPE_EVENT, {0} };
#endif

static kernel_pid_t _pid;
static otInstance *sInstance;

/* OpenThread will call this when switching state from empty tasklet to non-empty tasklet. */
void otSignalTaskletPending(otInstance *aInstance)
{
    //Unused
}

void *_openthread_event_loop(void *arg)
{
    _pid = thread_getpid();

    sInstance = otInstanceInit();
#if defined(MODULE_OPENTHREAD_CLI) && defined(MODULE_OPENTHREAD_NCP)
    otPlatUartEnable();
#endif

    msg_init_queue(_queue, OPENTHREAD_QUEUE_LEN);
    netdev2_t *dev;
    msg_t msg;

#ifdef MODULE_OPENTHREAD_CLI
    otCliUartInit(sInstance);
#else

#ifdef MODULE_OPENTHREAD_NCP
    otNcpInit(sInstance);
#endif

    /* It's necessary to call this after otEnable. Otherwise will freeze */
    otProcessQueuedTasklets(sInstance);
#endif

#ifdef _OPENTHREAD_DIRECT_UART
    static uint8_t uart_buf_length = 0;
    static uint8_t uart_buf[16];
#endif

    while (1) {
        /* Process OpenThread tasklets */
        begin_mutex();
        otProcessQueuedTasklets(sInstance);
        end_mutex();

#ifdef _OPENTHREAD_DIRECT_UART
        static xtimer_t uart_timer;
        xtimer_set_msg(&uart_timer, 10000, &uart_timer_msg, sched_active_pid);
#else
        msg_receive(&msg);
#endif
        switch (msg.type) {
            case OPENTHREAD_XTIMER_MSG_TYPE_EVENT:
                /* Tell OpenThread a time event was received */
                begin_mutex();
                otPlatAlarmFired(sInstance);
                end_mutex();
                break;
            case OPENTHREAD_NETDEV2_MSG_TYPE_EVENT:
                /* Received an event from driver */
                dev = (netdev2_t *) msg.content.ptr;
                dev->driver->isr(dev);
                break;
#if defined(MODULE_OPENTHREAD_CLI) || defined(MODULE_OPENTHREAD_NCP)
            case OPENTHREAD_SERIAL_MSG_TYPE_EVENT:
                /* Tell OpenThread about the receotion of a CLI command */
#ifdef OPENTHREAD_UART_DEV
                uart_buf_length = tsrb_get(&openthread_uart_rx_buffer, (char*)uart_buf, sizeof(uart_buf));
                if (uart_buf_length != 0) {
                    begin_mutex();
                    otPlatUartReceived((const uint8_t*)&uart_buf, uart_buf_length);
                    end_mutex();
                }
#else
                begin_mutex();
                otPlatUartReceived((const uint8_t*)&msg.content.value, 1);
                end_mutex();
#endif
                break;
#endif

        }
    }

    return NULL;
}

void _event_cb(netdev2_t *dev, netdev2_event_t event)
{
    if (event == NETDEV2_EVENT_ISR) {
        assert(_pid != KERNEL_PID_UNDEF);
        msg_t msg;

        msg.type = OPENTHREAD_NETDEV2_MSG_TYPE_EVENT;
        msg.content.ptr = dev;

        if (msg_send(&msg, _pid) <= 0) {
            DEBUG("openthread_netdev2: possibly lost interrupt.\n");
        }
    }
    else {
        switch (event) {
            case NETDEV2_EVENT_RX_COMPLETE:
                DEBUG("openthread_netdev2: Reception of a pcket\n");
                recv_pkt(dev, sInstance);
                break;
            case NETDEV2_EVENT_TX_COMPLETE:
            case NETDEV2_EVENT_TX_NOACK:
            case NETDEV2_EVENT_TX_MEDIUM_BUSY:
                DEBUG("openthread_netdev2: Transmission of a pcket\n");
                send_pkt(dev, event, sInstance);
                break;
            default:
                break;
        }
    }
}

/* get OpenThread thread pid */
kernel_pid_t openthread_get_pid(void)
{
    return _pid;
}

/* starts OpenThread thread */
int openthread_netdev2_init(char *stack, int stacksize, char priority,
                            const char *name, netdev2_t *netdev)
{
    netdev->driver->init(netdev);
    netdev->event_callback = _event_cb;

    netopt_enable_t enable = NETOPT_ENABLE;
    netdev->driver->set(netdev, NETOPT_TX_END_IRQ, &enable, sizeof(enable));

    _pid = thread_create(stack, stacksize,
                         priority, THREAD_CREATE_STACKTEST,
                         _openthread_event_loop, NULL, name);

    if (_pid <= 0) {
        return -EINVAL;
    }

    return _pid;
}
void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
    (void)aInstance;
    (void)aEnable;
}

ThreadError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    (void)aInstance;
    (void)aShortAddress;
    return kThreadError_None;
}

ThreadError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const uint8_t *aExtAddress)
{
    (void)aInstance;
    (void)aExtAddress;
    return kThreadError_None;
}

ThreadError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    (void)aInstance;
    (void)aShortAddress;
    return kThreadError_None;
}

ThreadError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const uint8_t *aExtAddress)
{
    (void)aInstance;
    (void)aExtAddress;
    return kThreadError_None;
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
    (void)aInstance;
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
    (void)aInstance;
}
int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
	(void)aInstance;
	return 0;
}
/** @} */
