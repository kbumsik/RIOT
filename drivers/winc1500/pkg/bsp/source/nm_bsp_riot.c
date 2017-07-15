/**
 *
 * \file
 *
 * \brief This module contains SAMD21 BSP APIs implementation.
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

#include "board.h"
#include "periph/gpio.h"
#include "xtimer.h"

#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"

#include "conf_winc.h"

static tpfNmBspIsr gpfIsr;

static void chip_isr(void *args)
{
	if (gpfIsr) {
		gpfIsr();
	}
}

/*
 *	@fn		init_chip_pins
 *	@brief	Initialize reset, chip enable and wake pin
 */
static void init_chip_pins(void)
{
	/* Configure INTN pins as input. */ // TODO: Delete?
	gpio_init(WINC1500_INTN_PIN, GPIO_IN);

	/* Configure RESETN pin as output. */
	gpio_init(WINC1500_RESET_PIN, GPIO_OUT);
	gpio_clear(WINC1500_RESET_PIN);

	/* Configure CHIP_EN as output */
	gpio_init(WINC1500_WAKE_PIN, GPIO_OUT);

	/* Configure CHIP_EN as output */
	if (WINC1500_CHIP_EN_PIN != GPIO_UNDEF) {
		gpio_init(WINC1500_CHIP_EN_PIN, GPIO_OUT); // TODO: pulled up input?
	}
}

/*
 *	@fn		nm_bsp_init
 *	@brief	Initialize BSP
 *	@return	0 in case of success and -1 in case of failure
 */
int8_t nm_bsp_init(void)
{
	gpfIsr = NULL;

	/* Initialize chip IOs. */
	init_chip_pins();

	/* Perform chip reset. */
	nm_bsp_reset();

	return M2M_SUCCESS;
}

/**
 *	@fn		nm_bsp_deinit
 *	@brief	De-iInitialize BSP
 *	@return	0 in case of success and -1 in case of failure
 */
int8_t nm_bsp_deinit(void)
{
	/* Configure control pins as input no pull up. */
	gpio_clear(WINC1500_RESET_PIN);
	gpio_init(WINC1500_RESET_PIN, GPIO_IN);
	if (WINC1500_CHIP_EN_PIN != GPIO_UNDEF) {
		gpio_clear(WINC1500_CHIP_EN_PIN);
		gpio_init(WINC1500_CHIP_EN_PIN, GPIO_IN);
	}

	return M2M_SUCCESS;
}

/**
 *	@fn		nm_bsp_reset
 *	@brief	Reset NMC1500 SoC by setting CHIP_EN and RESET_N signals low,
 *           CHIP_EN high then RESET_N high
 */
void nm_bsp_reset(void)
{
	if (WINC1500_CHIP_EN_PIN != GPIO_UNDEF) {
		gpio_clear(WINC1500_CHIP_EN_PIN);
	}
	gpio_clear(WINC1500_RESET_PIN);
	nm_bsp_sleep(100);
	if (WINC1500_CHIP_EN_PIN != GPIO_UNDEF) {
		gpio_set(WINC1500_CHIP_EN_PIN);
	}
	nm_bsp_sleep(100);
	gpio_set(WINC1500_RESET_PIN);
	nm_bsp_sleep(100);
}

/*
 *	@fn		nm_bsp_sleep
 *	@brief	Sleep in units of mSec
 *	@param[IN]	u32TimeMsec
 *				Time in milliseconds
 */
void nm_bsp_sleep(uint32_t u32TimeMsec)
{
	while (u32TimeMsec--) {
		xtimer_usleep(1);
	}
}

/*
 *	@fn		nm_bsp_register_isr
 *	@brief	Register interrupt service routine
 *	@param[IN]	pfIsr
 *				Pointer to ISR handler
 */
void nm_bsp_register_isr(tpfNmBspIsr pfIsr)
{
	gpfIsr = pfIsr;
	gpio_init_int(WINC1500_INTN_PIN, GPIO_IN, GPIO_FALLING, chip_isr, NULL);
}

/*
 *	@fn		nm_bsp_interrupt_ctrl
 *	@brief	Enable/Disable interrupts
 *	@param[IN]	u8Enable
 *				'0' disable interrupts. '1' enable interrupts
 */
void nm_bsp_interrupt_ctrl(uint8_t u8Enable)
{
	if (u8Enable) {
		gpio_init_int(WINC1500_INTN_PIN, GPIO_IN, GPIO_FALLING, chip_isr, NULL);
	} else {
		gpio_irq_disable(WINC1500_INTN_PIN);
	}
}
