/**
 *
 * \file
 *
 * \brief WINC1500 configuration.
 *
 * Copyright (c) 2016-2017 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef CONF_WINC_H_INCLUDED
#define CONF_WINC_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

/* This configuration files comes with empty settings! */
/* Default settings for SAMW25 Xplained Pro. */

/*
   ---------------------------------
   ---------- PIN settings ---------
   ---------------------------------
*/

#ifndef WINC1500_RESET_PIN
  #define WINC1500_RESET_PIN  (GPIO_PIN(PB, 6))
#endif
#ifndef WINC1500_CHIP_EN_PIN
  #define WINC1500_CHIP_EN_PIN (GPIO_PIN(PB, 5))
#endif
#ifndef WINC1500_WAKE_PIN
  #define WINC1500_WAKE_PIN (GPIO_PIN(PB, 7))
#endif

/*
   ---------------------------------
   ---------- SPI settings ---------
   ---------------------------------
*/

#define CONF_WINC_USE_SPI				(1)

/** SPI pin and instance settings. */
#if !defined(WINC1500_SPI)
  #define WINC1500_SPI SPI_DEV(0)
#endif
#if !defined(WINC1500_INTN_PIN)
  #define WINC1500_INTN_PIN   (GPIO_PIN(PB, 4))
#endif
#if !defined(WINC1500_SPI_CS_PIN)
  #define WINC1500_SPI_CS_PIN (GPIO_PIN(PA, 5))
#endif

/** SPI clock. */
#define CONF_WINC_SPI_CLOCK				(10000000)

/*
   ---------------------------------
   --------- Debug Options ---------
   ---------------------------------
*/
#include <stdio.h>

#define CONF_WINC_DEBUG					(1)
#define CONF_WINC_PRINTF				printf

#ifdef __cplusplus
}
#endif

#endif /* CONF_WINC_H_INCLUDED */
