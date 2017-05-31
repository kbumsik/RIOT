/**
 *
 * \file
 *
 * \brief This module contains NMC1000 bus wrapper APIs implementation.
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
#include "periph/spi.h"

#include <stdio.h>
#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"

#include "conf_winc.h"

#define NM_BUS_MAX_TRX_SZ	256

tstrNmBusCapabilities egstrNmBusCapabilities =
{
	NM_BUS_MAX_TRX_SZ
};

static int8_t spi_rw(uint8_t* pu8Mosi, uint8_t* pu8Miso, uint16_t u16Sz)
{
	uint8_t u8Dummy = 0;
	uint8_t u8SkipMosi = 0, u8SkipMiso = 0;

	/* Check parameters */
	if (((pu8Miso == NULL) && (pu8Mosi == NULL)) ||(u16Sz == 0)) {
		return M2M_ERR_INVALID_ARG;
	}

	/* Enable/Disable in/out operations */
	if (pu8Mosi == NULL) {
		pu8Mosi = &u8Dummy;
		u8SkipMosi = 1;
	}
	if(pu8Miso == NULL) {
		pu8Miso = &u8Dummy;
		u8SkipMiso = 1;
	}

	spi_acquire(WINC1500_SPI, SPI_CS_UNDEF, CONF_WINC_SPI_MODE, CONF_WINC_SPI_CLOCK);
	gpio_clear(WINC1500_SPI_CS_PIN);

	while (u16Sz) {
		*pu8Miso = spi_transfer_byte(WINC1500_SPI, SPI_CS_UNDEF,
										 true, *pu8Mosi);
		u16Sz--;
		if (!u8SkipMiso) {
			pu8Miso++;
		}
		if (!u8SkipMosi) {
			pu8Mosi++;
		}
	}

	gpio_set(WINC1500_SPI_CS_PIN);
	spi_release(WINC1500_SPI);

	return M2M_SUCCESS;
}

/*
*	@fn		nm_bus_init
*	@brief	Initialize the bus wrapper
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
int8_t nm_bus_init(void *pvinit)
{
	int8_t result = M2M_SUCCESS;
	
	/* Configure SPI peripheral. */
	spi_init(WINC1500_SPI);
	
	/* Configure CS PIN. */
	/* This step will set the CS high */
	gpio_init(WINC1500_SPI_CS_PIN, GPIO_OUT);
	gpio_set(WINC1500_SPI_CS_PIN);

	/* Reset WINC1500. */
	nm_bsp_reset();
	nm_bsp_sleep(1);
	return result;
}

/*
*	@fn		nm_bus_ioctl
*	@brief	send/receive from the bus
*	@param[IN]	u8Cmd
*					IOCTL command for the operation
*	@param[IN]	pvParameter
*					Arbitrary parameter depenging on IOCTL
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@note	For SPI only, it's important to be able to send/receive at the same time
*/
int8_t nm_bus_ioctl(uint8_t u8Cmd, void* pvParameter)
{
	int8_t s8Ret = 0;
	switch(u8Cmd)
	{
		case NM_BUS_IOCTL_RW: {
			tstrNmSpiRw *pstrParam = (tstrNmSpiRw *)pvParameter;
			s8Ret = spi_rw(pstrParam->pu8InBuf, pstrParam->pu8OutBuf, pstrParam->u16Sz);
		}
		break;
		default:
			s8Ret = -1;
			M2M_ERR("invalide ioclt cmd\n");
			break;
	}

	return s8Ret;
}

/*
*	@fn		nm_bus_deinit
*	@brief	De-initialize the bus wrapper
*/
int8_t nm_bus_deinit(void)
{
	int8_t result = M2M_SUCCESS;
	/* Code for Disabling SPI bus */
	return result;
}

/*
*	@fn			nm_bus_reinit
*	@brief		re-initialize the bus wrapper
*	@param [in]	void *config
*					re-init configuration data
*	@return		M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
int8_t nm_bus_reinit(void* config)
{
	return M2M_SUCCESS;
}

