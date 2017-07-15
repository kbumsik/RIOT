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

#include "winc1500.h"
#include "winc1500_params.h"

/* Shell functions */
static int _init(int argc, char **argv);
// static int _chipinfo(int argc, char **argv);
static int _scan(int argc, char **argv);
static int _connect(int argc, char **argv);
static int _disconnect(int argc, char **argv);
static int _rssi(int argc, char **argv);

static const shell_command_t shell_commands[] = {
    { "init", "initializes WINC1500 module", _init },
    { "scan", "Scan for Access Points available", _scan },
    { "connect", "Connect to an Access Point", _connect },
    { "disconnect", "Connect from an Access Point", _disconnect },
    { "rssi", "Display RSSI for the connected AP", _rssi },
    { NULL, NULL, NULL }
};


static int _init(int argc, char **argv)
{
    // PARAMS
    if (WINC1500_OK == winc1500_init(&winc1500_params[0])) {
        puts("[OK]");
        return 0;
    }
    puts("[Error]");
    return 1;
}

static int _scan(int argc, char **argv)
{
    int result = winc1500_scan();
    if (result < 0) {
        puts("[Scanning error]");
        return 1;
    }
    printf("%d access points found\n", result);

    char ssid[WINC1500_MAX_SSID_LEN];
    winc1500_ap_t ap = {.ssid = ssid};
    for (int i = 0; i < result; i++) {
        winc1500_read_ap(&ap, i);
        printf("[%d] %s %d dBm ", i, ssid, ap.rssi);
        if (ap.sec & WINC1500_SEC_FLAGS_ENTERPRISE) {
            puts("WPA_Enterprise");
        }
        else if (ap.sec & WINC1500_SEC_FLAGS_WPA) {
            puts("WPA_PSK");
        }
        else if (ap.sec & WINC1500_SEC_FLAGS_OPEN) {
            puts("OPEN");
        }
        else if (ap.sec & WINC1500_SEC_FLAGS_WEP) {
            puts("WEP");
        }
        else {
            puts("Unknown");
        }
    }
    puts("[OK]");
    return result;
}

static int _connect(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: %s SSID [passphrase]\n", argv[0]);
        return 1;
    }

    winc1500_ap_t ap = {.ssid = NULL, .password = NULL};

    /* Get SSID */
    if (argc > 1) {
        ap.ssid = (char *)argv[1];
        ap.sec = WINC1500_SEC_FLAGS_OPEN;
    } else {
        puts("Please provide SSID to connect");
        return 1;
    }
    /* Get password if provided */
    if (argc > 2) {
        ap.password = (char *)argv[2];
        ap.sec = WINC1500_SEC_FLAGS_WPA2;
    }

    int result = winc1500_connect_single(&ap);
    if (result == WINC1500_OK) {
        puts("[OK]");
        return 0;
    } else {
        puts("Try using another SSID and passphrase, or just try connecting again.");
        puts("[Connecting failed]");
        return 1;
    }
}

static int _disconnect(int argc, char **argv)
{
    puts("Command in progress. Exiting...");
    return 1;

    int result = winc1500_disconnect();
    if (result == WINC1500_OK) {
        puts("[OK]");
        return 0;
    } else {
        puts("[Failed]");
        return 1;
    }
}

static int _rssi(int argc, char **argv)
{
    puts("Command in progress. Exiting...");
    return 1;

    int rssi;
    int result = winc1500_read_rssi(&rssi);
    if (result == WINC1500_OK) {
        printf("RSSI: %d", rssi);
        puts("[OK]");
        return 0;
    } else {
        puts("[Failed]");
        return 1;
    }
}

int main(void)
{
    puts("WINC1500 WiFi module test application");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
    return 0;
}
