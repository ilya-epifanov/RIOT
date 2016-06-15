/*
 * Copyright (C) 2014 Freie Universität Berlin
 * Copyright (C) 2015 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    board_phywave_eval phyWAVE-KW22 Board
 * @ingroup     boards
 * @brief       Board specific implementations for the phyWAVE evaluation board
 * @{
 *
 * @file
 * @brief       Board specific definitions for the phyWAVE evaluation board
 *
 * @author      Johann Fischer <j.fischer@phytec.de>
 */

#ifndef BOARD_H_
#define BOARD_H_

#include "cpu.h"
#include "periph_conf.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief   LED pin definitions and handlers
 * @{
 */
#define LED0_PORT           PORT_C
#define LED1_PORT           PORT_A
#define LED2_PORT           PORT_A
#define LED3_PORT           PORT_B

#define LED0_GPIOR          GPIOC
#define LED1_GPIOR          GPIOA
#define LED2_GPIOR          GPIOA
#define LED3_GPIOR          GPIOB

#define LED0_BIT            1
#define LED1_BIT            19
#define LED2_BIT            18
#define LED3_BIT            0

#define LED0_PIN            GPIO_PIN(LED0_PORT, LED0_BIT)
#define LED1_PIN            GPIO_PIN(LED1_PORT, LED1_BIT)
#define LED2_PIN            GPIO_PIN(LED2_PORT, LED2_BIT)
#define LEDR_PIN            LED0_PIN
#define LEDG_PIN            LED1_PIN
#define LEDB_PIN            LED2_PIN
#define LED3_PIN            GPIO_PIN(LED3_PORT, LED3_BIT)

#define LED0_MASK           (1 << LED0_BIT)
#define LED1_MASK           (1 << LED1_BIT)
#define LED2_MASK           (1 << LED2_BIT)
#define LED3_MASK           (1 << LED3_BIT)

#define LED0_ON            (LED0_GPIOR->PCOR = LED0_MASK)
#define LED0_OFF           (LED0_GPIOR->PSOR = LED0_MASK)
#define LED0_TOGGLE        (LED0_GPIOR->PTOR = LED0_MASK)

#define LED1_ON            (LED1_GPIOR->PCOR = LED1_MASK)
#define LED1_OFF           (LED1_GPIOR->PSOR = LED1_MASK)
#define LED1_TOGGLE        (LED1_GPIOR->PTOR = LED1_MASK)

#define LED2_ON            (LED2_GPIOR->PCOR = LED2_MASK)
#define LED2_OFF           (LED2_GPIOR->PSOR = LED2_MASK)
#define LED2_TOGGLE        (LED2_GPIOR->PTOR = LED2_MASK)

#define LED3_ON            (LED3_GPIOR->PCOR = LED3_MASK)
#define LED3_OFF           (LED3_GPIOR->PSOR = LED3_MASK)
#define LED3_TOGGLE        (LED3_GPIOR->PTOR = LED3_MASK)
/** @} */

/**
 * @name Macro for button S1/S2.
 * @{
 */
#define BUTTON0_PORT         PORT_C
#define BUTTON0_BIT          5
#define BUTTON0_PIN          GPIO_PIN(BUTTON0_PORT, BUTTON0_BIT)

#define BUTTON1_PORT         PORT_C
#define BUTTON1_BIT          4
#define BUTTON1_PIN          GPIO_PIN(BUTTON1_PORT, BUTTON1_BIT)
/** @} */

/**
@name KW2XRF configuration
@{
*/
#define KW2XRF_SPI        (SPI_1)
#define KW2XRF_CS         (GPIO_PIN(KW2XDRF_PORT, KW2XDRF_PCS0_PIN))
#define KW2XRF_INT        (GPIO_PIN(KW2XDRF_PORT, KW2XDRF_IRQ_PIN))
#define KW2XRF_SPI_SPEED  (SPI_SPEED_10MHZ)
#define KW2XRF_SHARED_SPI 0
/** @}*/

/**
 * @name Define the interface for the HDC1000 humidity sensor
 * @{
 */
#define HDC1000_I2C         (I2C_0)
#define HDC1000_ADDR        (0x43)
/** @} */

/**
 * @name Define the interface for the MAG3110 magnetometer sensor
 * @{
 */
#define MAG3110_I2C         (I2C_0)
#define MAG3110_ADDR        (0x0E)
/** @} */

/**
 * @name Define the interface for the MMA8652 tri-axis accelerometer sensor
 * @{
 */
#define MMA8652_I2C         (I2C_0)
#define MMA8652_ADDR        (0x1D)
/** @} */

/**
 * @name Define the interface for the MPL3115A2 pressure sensor
 * @{
 */
#define MPL3115A2_I2C       (I2C_0)
#define MPL3115A2_ADDR      (0x60)
/** @} */

/**
 * @name Define the interface for the TCS3772 light sensor
 * @{
 */
#define TCS37727_I2C        (I2C_0)
#define TCS37727_ADDR       (0x29)
/** @} */

/**
 * @name Define the interface for the TMP006 IR-Termopile sensor
 * @{
 */
#define TMP006_I2C          (I2C_0)
#define TMP006_ADDR         (0x41)
/** @} */

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
/** @} */
