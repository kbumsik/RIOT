/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *               2015 Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_at86rf2xx
 *
 * @{
 * @file
 * @brief       Default configuration for the AT86RF2xx driver
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 */

#ifndef WINC1500_PARAMS_H
#define WINC1500_PARAMS_H

#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Set default configuration parameters for the WINC1500 driver
 * @{
 */
#define WINC1500_PARAMS_DEFAULT    {.spi = WINC1500_SPI, \
                                     .cs_pin = WINC1500_SPI_CS_PIN, \
                                     .int_pin = WINC1500_INTN_PIN, \
                                     .reset_pin = WINC1500_RESET_PIN, \
                                     .en_pin = WINC1500_CHIP_EN_PIN, \
                                     .wake_pin = WINC1500_WAKE_PIN, \
                                     .spi_clk = WINC1500_SPI_CLOCK}
/**@}*/

/**
 * @brief   WINC1500 SPI configuration
 * @{
 */
static const winc1500_params_t winc1500_params[] =
{
#ifdef WINC1500_PARAMS_BOARD
    WINC1500_PARAMS_BOARD,
#else
    WINC1500_PARAMS_DEFAULT,
#endif
};
/**@}*/

/**
 * @brief   WINC1500 SPI mode and clock configurations
 * @{
 */
#ifndef WINC1500_SPI_MODE
  #define WINC1500_SPI_MODE  SPI_MODE_0
#endif
#ifndef WINC1500_SPI_CLOCK
  #define WINC1500_SPI_CLOCK  SPI_CLK_10MHZ
#endif
/**@}*/

#ifdef __cplusplus
}
#endif

#endif /* WINC1500_PARAMS_H */
/** @} */
