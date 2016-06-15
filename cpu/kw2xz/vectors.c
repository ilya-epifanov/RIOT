/*
 * Copyright (C) 2014 Freie Universität Berlin
 * Copyright (C) 2015 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_kw2x
 * @{
 *
 * @file
 * @brief       Interrupt vector definition for MKW2XZXXX MCUs
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Johann Fischer <j.fischer@phytec.de>
 *
 * @}
 */

#include <stdint.h>
#include "vectors_cortexm.h"
#include "wdog.h"

/**
 * memory markers as defined in the linker script
 */
extern uint32_t _estack;

void pre_startup (void)
{
    /* disable the WDOG */
    wdog_disable();
}

void dummy_handler(void)
{
    dummy_handler_default();
}

/* Cortex-M specific interrupt vectors */
WEAK_DEFAULT void isr_svc(void);
WEAK_DEFAULT void isr_pendsv(void);
WEAK_DEFAULT void isr_systick(void);
/* MKW22D512 specific interrupt vector */
WEAK_DEFAULT void isr_dma0(void);
WEAK_DEFAULT void isr_dma1(void);
WEAK_DEFAULT void isr_dma2(void);
WEAK_DEFAULT void isr_dma3(void);
WEAK_DEFAULT void isr_ftfa(void);
WEAK_DEFAULT void isr_pmc(void);
WEAK_DEFAULT void isr_llwu(void);
WEAK_DEFAULT void isr_i2c0(void);
WEAK_DEFAULT void isr_i2c1(void);
WEAK_DEFAULT void isr_spi0(void);
WEAK_DEFAULT void isr_tsi1(void);
WEAK_DEFAULT void isr_uart0(void);
WEAK_DEFAULT void isr_rng(void);
WEAK_DEFAULT void isr_cmt(void);
WEAK_DEFAULT void isr_adc0(void);
WEAK_DEFAULT void isr_cmp0(void);
WEAK_DEFAULT void isr_ftm0(void);
WEAK_DEFAULT void isr_ftm1(void);
WEAK_DEFAULT void isr_ftm2(void);
WEAK_DEFAULT void isr_rtc(void);
WEAK_DEFAULT void isr_rtc_seconds(void);
WEAK_DEFAULT void isr_pit0(void);
WEAK_DEFAULT void isr_aesa0(void);
WEAK_DEFAULT void isr_radio0(void);
WEAK_DEFAULT void isr_dac0(void);
WEAK_DEFAULT void isr_radio1(void);
WEAK_DEFAULT void isr_mcg(void);
WEAK_DEFAULT void isr_lptmr0(void);
WEAK_DEFAULT void isr_spi1(void);
WEAK_DEFAULT void isr_porta(void);
WEAK_DEFAULT void isr_portbc(void);

/* interrupt vector table */
__attribute__((used, section(".vector_table")))
const void *interrupt_vector[] = {
    /* Stack pointer */
    (void *)(&_estack),             /* pointer to the top of the empty stack */
    /* Cortex-M4 handlers */
    (void*) reset_handler_default,  /* entry point of the program */
    (void*) nmi_default,            /* non maskable interrupt handler */
    (void*) hard_fault_default,     /* hard fault exception */
    (void*) (0UL),                  /* memory manage exception */
    (void*) (0UL),                  /* bus fault exception */
    (void*) (0UL),                  /* usage fault exception */
    (void*) (0UL),                  /* Reserved */
    (void*) (0UL),                  /* Reserved */
    (void*) (0UL),                  /* Reserved */
    (void*) (0UL),                  /* Reserved */
    (void*) isr_svc,                /* system call interrupt, in RIOT used for
                                     * switching into thread context on boot */
    (void*) (0UL),                  /* debug monitor exception */
    (void*) (0UL),                  /* Reserved */
    (void*) isr_pendsv,             /* pendSV interrupt, in RIOT the actual
                                     * context switching is happening here */
    (void*) isr_systick,            /* SysTick interrupt, not used in RIOT */
    /* MKW22Z512 specific peripheral handlers */
    (void *) isr_dma0,              /* DMA channel 0 transfer complete */
    (void *) isr_dma1,              /* DMA channel 1 transfer complete */
    (void *) isr_dma2,              /* DMA channel 2 transfer complete */
    (void *) isr_dma3,              /* DMA channel 3 transfer complete */
    (void *) (0UL),                 /* Reserved */
    (void *) isr_ftfa,              /* FTFL command complete / FTFL read collision */
    (void *) isr_pmc,               /* PMC controller low-voltage detect low-voltage warning */
    (void *) isr_llwu,              /* Low leakage wakeup */
    (void *) isr_i2c0,              /* Inter-integrated circuit 0 */
    (void *) isr_i2c1,              /* Inter-integrated circuit 1 */
    (void *) isr_spi0,              /* Serial peripheral Interface 0 */
    (void *) isr_tsi1,              /* Touch sense interface 0 */
    (void *) isr_uart0,             /* UART0 receive/transmit and error interrupts */
    (void *) isr_rng,               /* Randon number generator */
    (void *) isr_cmt,               /* Carrier modulator transmitter */
    (void *) isr_adc0,              /* Analog-to-digital converter 0 */
    (void *) isr_cmp0,              /* Comparator 0 */
    (void *) isr_ftm0,              /* FlexTimer module 0 fault overflow and channels interrupt */
    (void *) isr_ftm1,              /* FlexTimer module 1 fault overflow and channels interrupt */
    (void *) isr_ftm2,              /* FlexTimer module 2 fault overflow and channels interrupt */
    (void *) isr_rtc,               /* Real time clock */
    (void *) isr_rtc_seconds,       /* Real time clock seconds */
    (void *) isr_pit0,              /* Periodic interrupt timer channel 0 */
    (void *) isr_aesa0,             /* Advanced Encryption Standard Accelerator interrupt */
    (void *) isr_radio0,            /* Radio interrupt 0 */
    (void *) isr_dac0,              /* Digital-to-analog converter 0 */
    (void *) isr_radio1,            /* Radio interrupt 1 */
    (void *) isr_mcg,               /* Multipurpose clock generator */
    (void *) isr_lptmr0,            /* Low power timer interrupt */
    (void *) isr_spi1,              /* Serial peripheral Interface 1 */
    (void *) isr_porta,             /* Port A pin detect interrupt */
    (void *) isr_portbc,            /* Port B/C pin detect interrupt */
};
