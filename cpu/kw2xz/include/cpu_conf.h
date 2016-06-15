/*
 * Copyright (C) 2014 Freie Universität Berlin
 * Copyright (C) 2014 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup        cpu_kw2x KW2xZ SiP
 * @ingroup         cpu
 * @brief           CPU specific implementations for the Freescale KW2xZ SiP.
 *                  The SiP incorporates a low power 2.4 GHz transceiver and a
 *                  Kinetis Cortex-M4 MCU.
 * @{
 *
 * @file
 * @brief           Implementation specific CPU configuration options
 *
 * @author          Hauke Petersen <hauke.peterse@fu-berlin.de>
 * @author          Johann Fischer <j.fischer@phytec.de>
 */

#ifndef CPU_CONF_H
#define CPU_CONF_H

#include "cpu_conf_common.h"

#ifdef CPU_MODEL_KW21Z256
#include "MKW21Z4.h"
#elif CPU_MODEL_KW21Z512
#include "MKW21Z4.h"
#elif CPU_MODEL_KW31Z256
#include "MKW31Z4.h"
#elif CPU_MODEL_KW31Z512
#include "MKW31Z4.h"
#elif CPU_MODEL_KW41Z256
#include "MKW41Z4.h"
#elif CPU_MODEL_KW41Z512
#include "MKW41Z4.h"
#else
#error "undefined CPU_MODEL"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief   ARM Cortex-M specific CPU configuration
 * @{
 */
#define CPU_DEFAULT_IRQ_PRIO            (1U)
#define CPU_IRQ_NUMOF                   (65U)
#define CPU_FLASH_BASE                  (0x00000000)
/** @} */

/**
 * @name GPIO pin mux function numbers
 */
/** @{ */
#define PIN_MUX_FUNCTION_ANALOG 0
#define PIN_MUX_FUNCTION_GPIO 1
/** @} */
/**
 * @name GPIO interrupt flank settings
 */
/** @{ */
#define PIN_INTERRUPT_RISING 0b1001
#define PIN_INTERRUPT_FALLING 0b1010
#define PIN_INTERRUPT_EDGE 0b1011
/** @} */

/** @name PORT module clock gates */
/** @{ */
#define PORTA_CLOCK_GATE (BITBAND_REG32(SIM->SCGC5, SIM_SCGC5_PORTA_SHIFT))
#define PORTB_CLOCK_GATE (BITBAND_REG32(SIM->SCGC5, SIM_SCGC5_PORTB_SHIFT))
#define PORTC_CLOCK_GATE (BITBAND_REG32(SIM->SCGC5, SIM_SCGC5_PORTC_SHIFT))
#define PORTD_CLOCK_GATE (BITBAND_REG32(SIM->SCGC5, SIM_SCGC5_PORTD_SHIFT))
#define PORTE_CLOCK_GATE (BITBAND_REG32(SIM->SCGC5, SIM_SCGC5_PORTE_SHIFT))
/** @} */

/**
 * @brief MCU specific Low Power Timer settings.
 */
#define LPTIMER_CLKSRC                   LPTIMER_CLKSRC_LPO
#define LPTIMER_DEV                      (LPTMR0) /**< LPTIMER hardware module */
#define LPTIMER_CLKEN()                  (SIM->SCGC5 |= SIM_SCGC5_LPTMR_MASK) /**< Enable LPTMR0 clock gate */
#define LPTIMER_CLKDIS()                 (SIM->SCGC5 &= ~SIM_SCGC5_PTMR_MASK) /**< Disable LPTMR0 clock gate */
#define LPTIMER_CNR_NEEDS_LATCHING       1 /**< LPTMR.CNR register do not need latching */

/**
 * @name KW2XZ SiP internal interconnects between MCU and Modem.
 *
 * @{
 */
#define KW2XZRF_PORT_DEV        PORTB /**< MCU Port connected to Modem*/
#define KW2XZRF_PORT            PORT_B /**< MCU Port connected to Modem*/
#define KW2XZRF_GPIO            GPIOB /**< GPIO Device connected to Modem */
#define KW2XZRF_PORT_IRQn       PORTB_IRQn
/** Clock Enable for PORTB*/
#define KW2XZRF_PORT_CLKEN()    (PORTB_CLOCK_GATE = 1)
#define KW2XZRF_PIN_AF          2  /**< Pin Muxing Parameter for GPIO Device*/
#define KW2XZRF_PCS0_PIN        10 /**< SPI Slave Select Pin */
#define KW2XZRF_SCK_PIN         11 /**< SPI Clock Output Pin */
#define KW2XZRF_SOUT_PIN        16 /**< SPI Master Data Output Pin */
#define KW2XZRF_SIN_PIN         17 /**< SPI Master Data Input Pin */
#define KW2XZRF_RST_PIN         19 /**< Reset pin */
#define KW2XZRF_IRQ_PIN         3  /**< Modem's IRQ Output (activ low) */
#define KW2XZRF_CLK_CTRL_PORT   PORT_C /**< CLK_OUT control pin port */
#define KW2XZRF_CLK_CTRL_PORT_DEV PORTC /**< CLK_OUT control pin PORT device */
#define KW2XZRF_CLK_CTRL_GPIO   GPIOC /**< CLK_OUT control pin GPIO device */
#define KW2XZRF_CLK_CTRL_CLKEN() (PORTC_CLOCK_GATE = 1)
#define KW2XZRF_CLK_CTRL_PIN    0 /**< CLK_OUT control pin */


#define PORT_PCR_ODE_MASK PORT_PCR_DSE_MASK
#define PORT_PCR_ODE_SHIFT PORT_PCR_DSE_SHIFT
#define PORT_PCR_ODE PORT_PCR_DSE


/**
 * @brief Macro to calculate address of an aliased word in the peripheral
 *        bitband area for a peripheral register and bit (bit band region 0x40000000 to
 *        0x400FFFFF).
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return  Address of the aliased word in the peripheral bitband area.
 */
#define BITBAND_REGADDR(Reg,Bit) (0x42000000u + (32u*((uint32_t)&(Reg) - (uint32_t)0x40000000u)) + (4u*((uint32_t)(Bit))))
/**
 * @brief Macro to access a single bit of a peripheral register (bit band region
 *        0x40000000 to 0x400FFFFF) using the bit-band alias region access. Can
 *        be used for peripherals with 32bit access allowed.
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return Value of the targeted bit in the bit band region.
 */
#define BITBAND_REG32(Reg,Bit) (*((uint32_t volatile*)(BITBAND_REGADDR(Reg,Bit))))
#define BITBAND_REG(Reg,Bit) (BITBAND_REG32(Reg,Bit))
/**
 * @brief Macro to access a single bit of a peripheral register (bit band region
 *        0x40000000 to 0x400FFFFF) using the bit-band alias region access. Can
 *        be used for peripherals with 16bit access allowed.
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return Value of the targeted bit in the bit band region.
 */
#define BITBAND_REG16(Reg,Bit) (*((uint16_t volatile*)(BITBAND_REGADDR(Reg,Bit))))
/**
 * @brief Macro to access a single bit of a peripheral register (bit band region
 *        0x40000000 to 0x400FFFFF) using the bit-band alias region access. Can
 *        be used for peripherals with 8bit access allowed.
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return Value of the targeted bit in the bit band region.
 */
#define BITBAND_REG8(Reg,Bit) (*((uint8_t volatile*)(BITBAND_REGADDR(Reg,Bit))))

#define SIM_UIDH (SIM->UIDMH)
#define CPUID_LEN (24)

#define KINETIS_UART LPUART_Type


/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CPU_CONF_H */
/** @} */
