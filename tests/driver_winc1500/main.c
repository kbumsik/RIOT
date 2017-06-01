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
#include "xtimer.h"
#include "thread.h"

#include "driver/include/m2m_wifi.h"
#include "driver/source/nmasic.h"
#include "socket/include/socket.h"
#define htons(port)	_htons(port)
#define inet_pton()	nmi_inet_addr()

/* Gloabal flags */
static volatile enum {
    SCAN_STOPPED = 0, SCAN_ONLY = 1, SCAN_CONNECTING = 2
    } state_scanning = SCAN_STOPPED;
static volatile enum {
    STATE_ERROR = -1, STATE_INIT = 0,
    STATE_CONNECTED = 1, STATE_DISCONNECTED = 2
    } state_wifi = STATE_ERROR;
static volatile uint8_t state_rssi_updated = 0;
static volatile int8_t *rssi;
/** Number of APs found. */
static uint8_t num_founded_ap = 0;
/* Socket address */
struct sockaddr_in addr;
int state_socket = 0;

/* Shell functions */
static int _init(int argc, char **argv);
static int _chipinfo(int argc, char **argv);
static int _scan(int argc, char **argv);
static int _connect(int argc, char **argv);
static int _disconnect(int argc, char **argv);
static int _rssi(int argc, char **argv);
static int _tcpmsg(int argc, char **argv);
static int _udpmsg(int argc, char **argv);
static int _tcplisten(int argc, char **argv);
static int _udplisten(int argc, char **argv);

/* WINC1500 Thread */
char winc1500_thread_stack[THREAD_STACKSIZE_MINIMUM + 512 + 128];
void *winc1500_thread(void *arg);

/* Functions used for WINC1500 internal */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg);
extern void winc1500_socket_cb(SOCKET sock, uint8_t msg_type, void *payload);
extern void winc1500_dns_resolve_cb(uint8_t* domain_name, uint32_t server_ip_addr);

static const shell_command_t shell_commands[] = {
    { "init", "initializes WINC1500 module", _init },
    { "chipinfo", "Display WINC1500 module", _chipinfo },
    { "scan", "Scan for Access Points available", _scan },
    { "connect", "Connect to an Access Point", _connect },
    { "disconnect", "Connect from an Access Point", _disconnect },
    { "rssi", "Display RSSI for the connected AP", _rssi },
    { "tcpmsg", "Send a TCP message", _tcpmsg },
    { "udpmsg", "Send a UDP message", _udpmsg },
    { "tcplisten", "Listen TCP messages", _tcplisten },
    { "udplisten", "Listen UDP messages", _udplisten },
    { NULL, NULL, NULL }
};

/**
 * \brief Callback function of IP address.
 *
 * \param[in] hostName Domain name.
 * \param[in] hostIp Server IP.
 *
 * eturn None.
 */
// static void resolve_cb(uint8_t *hostName, uint32_t hostIp)
// {
// 	gu32HostIp = hostIp;
// 	gbHostIpByName = true;
// 	printf("resolve_cb: %s IP address is %d.%d.%d.%d\n\n", hostName,
// 			(int)IPV4_BYTE(hostIp, 0), (int)IPV4_BYTE(hostIp, 1),
// 			(int)IPV4_BYTE(hostIp, 2), (int)IPV4_BYTE(hostIp, 3));
// }

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] msg_type type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
 *  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
 *  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
 *  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
 *  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
 *  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
 * \param[in] payload A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type. Existing types are:
 *  - tstrM2mWifiStateChanged
 *  - tstrM2MWPSInfo
 *  - tstrM2MP2pResp
 *  - tstrM2MAPResp
 *  - tstrM2mScanDone
 *  - tstrM2mWifiscanResult
 */
static void wifi_cb(uint8_t msg_type, void *payload)
{
    static uint8_t scan_request_index = 0; /* Index of scan list 
                                            to request scan result. */
    switch (msg_type) {
        case M2M_WIFI_RESP_CON_STATE_CHANGED: {
            tstrM2mWifiStateChanged *wifi_state = (tstrM2mWifiStateChanged *)payload;
            switch (wifi_state->u8CurrState) {
                case M2M_WIFI_DISCONNECTED:		/*!< WiFi is disconnected from AP */
                    state_wifi = STATE_DISCONNECTED;
                    state_scanning = SCAN_STOPPED;
                    printf("Wi-Fi disconnected\n");
                    break;
                case M2M_WIFI_CONNECTED: 		/*!< WiFi is to connected to AP */
                    state_scanning = SCAN_STOPPED;
                    break;
                case M2M_WIFI_UNDEF:			/*!< Undefined status */
                default:
                    /* TODO */
                    break;
            }
        }
            break;
        case M2M_WIFI_RESP_CONN_INFO: {
        }
            break;
        case M2M_WIFI_REQ_DHCP_CONF: { /* Called by m2m_wifi_connect() */
            uint8_t *ip_addr = (uint8_t *)payload;
            state_scanning = SCAN_STOPPED;
            state_wifi = STATE_CONNECTED;
            printf("Wi-Fi connected\n");
            printf("Wi-Fi IP is %u.%u.%u.%u\n",
                    ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3]);
            break;
        }
        case M2M_WIFI_REQ_WPS: {
        }
            break;
        case M2M_WIFI_RESP_IP_CONFLICT: {
        }
            break;
        case M2M_WIFI_RESP_SCAN_DONE: {
            tstrM2mScanDone *scaninfo = (tstrM2mScanDone *)payload;
            scan_request_index = 0;
            if (scaninfo->u8NumofCh >= 1) {
                m2m_wifi_req_scan_result(scan_request_index);
                scan_request_index++;
            } else {
                m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
            }
        }
            break;
        case M2M_WIFI_RESP_SCAN_RESULT: {
            tstrM2mWifiscanResult *scan_result = (tstrM2mWifiscanResult *)payload;

            /* display founded AP. */
            printf("[%d] SSID:%s\n", scan_request_index, scan_result->au8SSID);
            num_founded_ap = m2m_wifi_get_num_ap_found();
            if (scan_request_index < num_founded_ap) {
                m2m_wifi_req_scan_result(scan_request_index);
                scan_request_index++;
            } else {
                /* Scanning Finished */
                state_scanning = SCAN_STOPPED;
            }
        }
            break;
        case M2M_WIFI_RESP_CURRENT_RSSI: {  /* Called by m2m_wifi_req_curr_rssi() */
            rssi = (int8_t *)payload;
            state_rssi_updated = 1;
        }
            break;
        case M2M_WIFI_RESP_CLIENT_INFO: {
        }
            break;
        case M2M_WIFI_RESP_PROVISION_INFO: {
        }
            break;
        case M2M_WIFI_RESP_DEFAULT_CONNECT: {
        }
            break;
        case M2M_WIFI_RESP_GET_SYS_TIME: {
            /* This event should be disabled */
            puts("SNTP time event called");
        }
            break;
        default: {
            printf("Unknown WiFi message: %d\n", msg_type);
        }
            break;
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
    if (M2M_SUCCESS != ret && M2M_ERR_FW_VER_MISMATCH != ret) {
        printf("m2m_wifi_init call error!(%d)\n", ret);
        return -1;
    }
    /* Init and Register socket callback */
    socketInit();
    registerSocketCallback(winc1500_socket_cb, winc1500_dns_resolve_cb);
    /* Create WINC1500 thread */
    thread_create(winc1500_thread_stack, sizeof(winc1500_thread_stack),
                  THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                  winc1500_thread, NULL, "winc1500");

    state_wifi = STATE_INIT;
    puts("[OK]");
    return 0;
}

static int _chipinfo(int argc, char **argv)
{
    uint8_t u8IsMacAddrValid;
    /** Mac address information. */
    static uint8_t mac_addr[M2M_MAC_ADDRES_LEN];

    if (state_wifi == STATE_ERROR) {
        puts("Initialize the WINC1500 first");
        return -1;
    }

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

    printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
            mac_addr[0], mac_addr[1], mac_addr[2],
            mac_addr[3], mac_addr[4], mac_addr[5]);

    puts("----------------------------------------");
    return 0;
}

static int _scan(int argc, char **argv)
{
    volatile int8_t state = 0;

    if (state_wifi == STATE_ERROR) {
        puts("Initialize the WINC1500 first");
        return -1;
    }
    
    /* Wait for other scanning request */
    while (state_scanning != SCAN_STOPPED) {
    } 
    /* Request scan. */
    state_scanning = SCAN_ONLY;
    m2m_wifi_request_scan(M2M_WIFI_CH_ALL);

    while (1) {
        state = state_scanning;
        if (state != SCAN_ONLY) {
            break;
        }
    }
    if (state == SCAN_STOPPED) {
        puts("[OK]");
        return 0;
    } else {
        puts("[Scanning failed]");
        return -1;
    }

}

static int _connect(int argc, char **argv)
{
    volatile uint8_t state = 0;
    char *ssid = NULL;
    char *password = NULL;
    tenuM2mSecType sec_type;

    if (state_wifi == STATE_ERROR) {
        puts("Initialize the WINC1500 first");
        return -1;
    }
    /* Get SSID */
    if (argc > 1) {
        ssid = (char *)argv[1];
        sec_type = M2M_WIFI_SEC_OPEN;
    } else {
        puts("Please provide SSID to connect");
        return -1;
    }
    /* Get password if provided */
    if (argc > 2) {
        password = (char *)argv[2];
        sec_type = M2M_WIFI_SEC_WPA_PSK;
    }
    /* Wait for other scanning request */
    while (state_scanning != SCAN_STOPPED) {
    } 

    /* If connected disconnect first */
    if (state_wifi == STATE_CONNECTED) {
        puts("Disconnecting the current AP first...");
        _disconnect(0, NULL);
        puts("Connecting a new AP...");
    }

    /* Request scan and connecting. */
    state_scanning = SCAN_CONNECTING;
    state_wifi = STATE_INIT;
    
    m2m_wifi_connect(ssid, strlen(ssid), sec_type, (void *)password, M2M_WIFI_CH_ALL);
    while (1) {
        state = state_wifi;
        if (state != STATE_INIT) {
            break;
        }
    }
    if (state == STATE_CONNECTED) {
        puts("[OK]");
        return 0;
    } else {
        puts("[Connecting failed]");
        return -1;
    }
}

static int _disconnect(int argc, char **argv)
{
    volatile uint8_t state = 0;
    
    if (state_wifi == STATE_ERROR) {
        puts("Initialize the WINC1500 first");
        return -1;
    }

    m2m_wifi_disconnect();
    while (1) {
        state = state_wifi;
        if (state == STATE_DISCONNECTED) {
            break;
        }
    }
    if (state == STATE_DISCONNECTED) {
        puts("[OK]");
        return 0;
    } else {
        puts("[Unknown Error]");
        return -1;
    }
}

static int _rssi(int argc, char **argv)
{
    volatile uint8_t state = 0;
    
    if (state_wifi == STATE_ERROR) {
        puts("Initialize the WINC1500 first");
        return -1;
    }

    /* Request RSSI for the connected AP. */
    state_rssi_updated = 0;
    m2m_wifi_req_curr_rssi();

    while (1) {
        state = state_rssi_updated;
        if (state != 0) {
            break;
        }
    }
    if (state == 1) {
        printf("RSSI for the current connected AP :%ddBm\n", (int8_t)(*rssi));
        return 0;
    } else {
        return -1;
    }
}

static int _tcpmsg(int argc, char **argv)
{
    char buffer[128];
    if (state_wifi == STATE_ERROR) {
        puts("Initialize the WINC1500 first");
        return -1;
    }
    char *host;
    char *msg;
    /* Get host and message */
    if (argc > 2) {
        host = (char *)argv[1];
        msg = (char *)argv[2];
    } else {
        puts("Please hostname and message.");
        return -1;
    }

    /** create socket file descriptor */
    SOCKET s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) {
        return -1;
    }
    printf("Socket %d created\n", s);
    /** setting up the host address */
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(5555); /* Port number */
    
    if ( 0 == (addr.sin_addr.s_addr = nmi_inet_addr(host))) {
        state_socket = 0;
        if (SOCK_ERR_NO_ERROR != gethostbyname((uint8_t *)host)) {
            goto failed;
        }
        while (state_socket == 0) {
            /* Wait until it's connected */
            xtimer_usleep(1000);
        }
    }
    
    while (0 == addr.sin_addr.s_addr) {
        /* This will be updated in callback function */
    }
    state_socket = 0;
    uint8_t *ip = (uint8_t *)&addr.sin_addr.s_addr;
    printf("socket is connecting to: %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
    int ret = connect(s, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
    if (ret < 0) {
        printf("Socket function failed: %d\n", ret);
        goto failed;
    }
    while (state_socket == 0) {
        /* Wait until it's connected */
        xtimer_usleep(1000);
    }

    printf("Empty the buffer by calling recv\n");
    state_socket = 0;
    recv(s, buffer, 128, 100);
    while (0 == state_socket){
        xtimer_usleep(1000);
    }

    printf("Now sending message: %s\n", msg);
    state_socket = 0;
    int n = send(s, msg, strlen(msg), 0);
    if (n < 0) {
        printf("socket sending packet unsuccessful: %d\n", n);
        goto failed;
    }
    while (state_socket == 0) {
        /* Wait until it's being sent */
        xtimer_usleep(1000);
    }

    puts("[OK]");
    close(s);
    return 0;
failed:
    puts("[Failed]");
    close(s);
    return -1;
}


static int _udpmsg(int argc, char **argv)
{
    if (state_wifi == STATE_ERROR) {
        puts("Initialize the WINC1500 first");
        return -1;
    }
    char *host;
    char *msg;
    /* Get host and message */
    if (argc > 2) {
        host = (char *)argv[1];
        msg = (char *)argv[2];
    } else {
        puts("Please hostname and message.");
        return -1;
    }

    /** create socket file descriptor */
    SOCKET s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) {
        return -1;
    }
    printf("Socket %d created\n", s);
    /** setting up the host address */
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(5555); /* Port number */
    
    if ( 0 == (addr.sin_addr.s_addr = nmi_inet_addr(host))) {
        state_socket = 0;
        if (SOCK_ERR_NO_ERROR != gethostbyname((uint8_t *)host)) {
            goto failed;
        }
        while (state_socket == 0) {
            /* Wait until it's connected */
            xtimer_usleep(1000);
        }
    }
    
    while (0 == addr.sin_addr.s_addr) {
        /* This will be updated in callback function */
    }
    state_socket = 0;
    uint8_t *ip = (uint8_t *)&addr.sin_addr.s_addr;
    printf("Now sending message: %s\n", msg);
    printf("\tto: %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
    int n = sendto(s, msg, strlen(msg), 0, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
    if (n < 0) {
        printf("socket sending packet unsuccessful: %d\n", n);
        goto failed;
    }
    while (state_socket == 0) {
        /* Wait until it's being sent */
        xtimer_usleep(1000);
    }
    puts("[OK]");
    close(s);
    return 0;
failed:
    puts("[Failed]");
    close(s);
    return -1;
}

static int _tcplisten(int argc, char **argv)
{
    char buffer[256];

    if (state_wifi == STATE_ERROR) {
        puts("Initialize the WINC1500 first");
        return -1;
    }
    /** create socket file descriptor */
    SOCKET s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) {
        return -1;
    }
    printf("Socket %d created\n", s);
    /** setting up the host address */
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(5555); /* Port number */
    addr.sin_addr.s_addr = 0; /* From anywhere */
    
    /* bind */
    state_socket = 0;
    bind(s, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
    while (0 == state_socket){
        xtimer_usleep(1000);
    }
    /* listen */
    state_socket = 0;
    listen(s, 0);
    while (0 == state_socket){
        xtimer_usleep(1000);
    }

    /* Wait for accept first?? */
    state_socket = 0;
    while (0 == state_socket){
        xtimer_usleep(1000);
    }
    /* recv */
    // TODO: Implement accept function
    state_socket = 0;
    recv(1, buffer, M2M_BUFFER_MAX_SIZE, 0);
    while (0 == state_socket){
        xtimer_usleep(1000);
    }
    printf("Received: %s\n", buffer);
    
    puts("[OK]");
    close(s);
    return 0;
}

static int _udplisten(int argc, char **argv)
{
    char buffer[256];

    if (state_wifi == STATE_ERROR) {
        puts("Initialize the WINC1500 first");
        return -1;
    }
    /** create socket file descriptor */
    SOCKET s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) {
        return -1;
    }
    printf("Socket %d created\n", s);
    /** setting up the host address */
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(5555); /* Port number */
    addr.sin_addr.s_addr = 0; /* From anywhere */
    
    /* bind */
    state_socket = 0;
    bind(s, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
    while (0 == state_socket){
        xtimer_usleep(1000);
    }

    /* recv */
    state_socket = 0;
    recvfrom(s, buffer, 256, 0);
    while (0 == state_socket){
        xtimer_usleep(1000);
    }
    printf("Received: %s\n", buffer);
    
    puts("[OK]");
    close(s);
    return 0;
}

void *winc1500_thread(void *arg)
{
    while (1) {
        /* Handle pending events from network controller. */
        while (m2m_wifi_handle_events(NULL) != M2M_SUCCESS) {
            xtimer_usleep(1000);
        }
        xtimer_usleep(1000);
    }
    return NULL;
}

int main(void)
{
    puts("WINC1500 WiFi module test application");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
    return 0;
}
