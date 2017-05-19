/**
 *
 * \file
 *
 * \brief WINC1500 Chip Information Example.
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
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

/** \mainpage
 * \section intro Introduction
 * This example demonstrates the use of the WINC1500 with the SAMD21 Xplained Pro
 * board to retrieve the chip information of the Wi-Fi module.<br>
 * It uses the following hardware:
 * - the SAMD21 Xplained Pro.
 * - the WINC1500 on EXT1.
 *
 * \section files Main Files
 * - main.c : Initialize the WINC1500 and retrieve information.
 *
 * \section usage Usage
 * -# Build the program and download it into the board.
 * -# On the computer, open and configure a terminal application as the follows.
 * \code
 *    Baud Rate : 115200
 *    Data : 8bit
 *    Parity bit : none
 *    Stop bit : 1bit
 *    Flow control : none
 * \endcode
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 * \code
 *    -- WINC1500 chip information example --
 *    -- SAMD21_XPLAINED_PRO --
 *    -- Compiled: xxx xx xxxx xx:xx:xx --
 *    Chip ID :             xxxxxx
 *    RF Revision ID :      x
 *    Done.
 * \endcode
 *
 * \section compinfo Compilation Information
 * This software was written for the GNU GCC compiler using Atmel Studio 6.2
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com">Atmel</A>.\n
 */

#include "board.h"
#include "shell.h"
#include <string.h>

#include "driver/include/m2m_wifi.h"
#include "driver/source/nmasic.h"

#define STRING_HEADER "-- WINC1500 chip information example --\n" \
	"-- "BOARD_NAME " --"STRING_EOL	\
	"-- Compiled: "__DATE__ " "__TIME__ " --\n"

/** Wi-Fi Settings */
#define MAIN_WLAN_SSID        "KBUMSIK" /* < Destination SSID */
#define MAIN_WLAN_AUTH        M2M_WIFI_SEC_WPA_PSK /* < Security manner */
#define MAIN_WLAN_PSK         "stony2014" /* < Password for Destination SSID */

/* Gloabal flags */
static volatile uint8_t state_connected = 0;
static volatile uint8_t state_rssi_updated = 0;
static volatile int8_t *rssi;
/** Index of scan list to request scan result. */
static uint8_t scan_request_index = 0;
/** Number of APs found. */
static uint8_t num_founded_ap = 0;

/* Shell functions */
static int _init(int argc, char **argv);
static int _chipinfo(int argc, char **argv);
//static int _scan(int argc, char **argv);
static int _connect(int argc, char **argv);
static int _rssi(int argc, char **argv);

/* Functions used for WINC1500 internal */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg);

static const shell_command_t shell_commands[] = {
    { "init", "initializes WINC1500 module", _init },
    { "chipinfo", "Display WINC1500 module", _chipinfo },
    //{ "scan", "Scan for Access Points available", _scan },
    { "connect", "Connect to an Access Point", _connect },
    { "rssi", "Display RSSI for the connected AP", _rssi },
    { NULL, NULL, NULL }
};

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_DHCP_CONF](@ref M2M_WIFI_REQ_DHCP_CONF)
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type.
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_SCAN_DONE:
	{
		tstrM2mScanDone *pstrInfo = (tstrM2mScanDone *)pvMsg;
		scan_request_index = 0;
		if (pstrInfo->u8NumofCh >= 1) {
			m2m_wifi_req_scan_result(scan_request_index);
			scan_request_index++;
		} else {
			m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
		}

		break;
	}

	case M2M_WIFI_RESP_SCAN_RESULT:
	{
		tstrM2mWifiscanResult *pstrScanResult = (tstrM2mWifiscanResult *)pvMsg;
		uint16_t demo_ssid_len;
		uint16_t scan_ssid_len = strlen((const char *)pstrScanResult->au8SSID);

		/* display founded AP. */
		printf("[%d] SSID:%s\r\n", scan_request_index, pstrScanResult->au8SSID);

		num_founded_ap = m2m_wifi_get_num_ap_found();
		if (scan_ssid_len) {
			/* check same SSID. */
			demo_ssid_len = strlen((const char *)MAIN_WLAN_SSID);
			if
			(
				(demo_ssid_len == scan_ssid_len) &&
				(!memcmp(pstrScanResult->au8SSID, (uint8_t *)MAIN_WLAN_SSID, demo_ssid_len))
			) {
				/* A scan result matches an entry in the preferred AP List.
				 * Initiate a connection request.
				 */
				printf("Found %s \r\n", MAIN_WLAN_SSID);
				m2m_wifi_connect((char *)MAIN_WLAN_SSID,
						sizeof(MAIN_WLAN_SSID),
						MAIN_WLAN_AUTH,
						(void *)MAIN_WLAN_PSK,
						M2M_WIFI_CH_ALL);
				break;
			}
		}

		if (scan_request_index < num_founded_ap) {
			m2m_wifi_req_scan_result(scan_request_index);
			scan_request_index++;
		} else {
			printf("can not find AP %s\r\n", MAIN_WLAN_SSID);
			m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
		}

		break;
	}

	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			m2m_wifi_request_dhcp_client();
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			printf("Wi-Fi disconnected\n");
		}
		break;
	}

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		state_connected = 1;
		printf("Wi-Fi connected\r\n");
		printf("Wi-Fi IP is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		break;
	}

	case M2M_WIFI_RESP_CURRENT_RSSI:
	{
		/* This message type is triggered by "m2m_wifi_req_curr_rssi()" function. */
		rssi = (int8_t *)pvMsg;
		state_rssi_updated = 1;
		break;
	}

	default:
	{
		break;
	}
	}
}

static int _init(int argc, char **argv)
{
	int8_t ret;	
	/* This struct is from driver/include/m2m_wifi.h */
	tstrWifiInitParam param;
	
	/* Initialize the BSP. */
	nm_bsp_init();

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Register a callback */
	param.pfAppWifiCb = wifi_cb;
	/* Initialize Wi-Fi driver with data and status callbacks. */
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("m2m_wifi_init call error!(%d)\r\n", ret);
		return -1;
	}
    puts("[OK]");
    return 0;
}

static int _chipinfo(int argc, char **argv)
{
	uint8_t u8IsMacAddrValid;
	/** Mac address information. */
	static uint8_t mac_addr[M2M_MAC_ADDRES_LEN];

	/** User define MAC Address. */
	const static char main_user_define_mac_address[] = {0xf8, 0xf0, 0x05, 0x20, 0x0b, 0x09};

    puts("----------------------------------------");
	printf("Chip ID : \t%x\n", (unsigned int)nmi_get_chipid());
	printf("RF Revision ID : \t%x\n", (unsigned int)nmi_get_rfrevid());
    
	/* Get MAC Address from OTP. */
	m2m_wifi_get_otp_mac_address(mac_addr, &u8IsMacAddrValid);
	if (!u8IsMacAddrValid) {
		printf("USER MAC Address : ");

		/* Cannot found MAC Address from OTP. Set user define MAC address. */
		m2m_wifi_set_mac_address((uint8_t *)main_user_define_mac_address);
	} else {
		printf("OTP MAC Address : ");
	}

	/* Get MAC Address. */
	m2m_wifi_get_mac_address(mac_addr);

	printf("%02X:%02X:%02X:%02X:%02X:%02X\r\n",
			mac_addr[0], mac_addr[1], mac_addr[2],
			mac_addr[3], mac_addr[4], mac_addr[5]);

	puts("----------------------------------------");
    return 0;
}

static int _connect(int argc, char **argv)
{
	volatile uint8_t state = 0;
	
	/* Request scan. */
	m2m_wifi_request_scan(M2M_WIFI_CH_ALL);

	while (1) {
		/* Handle pending events from network controller. */
		while (m2m_wifi_handle_events(NULL) != M2M_SUCCESS) {
		}
		state = state_connected;
		if (state != 0) {
			break;
		}
	}
	if (state == 1) {
		return 0;
	} else {
		return -1;
	}
}

static int _rssi(int argc, char **argv)
{
	volatile uint8_t state = 0;
	
	/* Request RSSI for the connected AP. */
	state_rssi_updated = 0;
	m2m_wifi_req_curr_rssi();

	while (1) {
		/* Handle pending events from network controller. */
		while (m2m_wifi_handle_events(NULL) != M2M_SUCCESS) {
		}
		state = state_rssi_updated;
		if (state != 0) {
			break;
		}
	}
	if (state == 1) {
		printf("RSSI for the current connected AP (%d)\n", (int8_t)(*rssi));
		return 0;
	} else {
		return -1;
	}
}

int main(void)
{
    puts("WINC1500 WiFi module test application");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
	return 0;
}
