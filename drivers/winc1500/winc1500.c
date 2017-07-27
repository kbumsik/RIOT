/*
 * Copyright (C) 2017 Bumsik Kim <kbumsik@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/* Required for strnlen in string.h, when building with -std=c99 */
#define _DEFAULT_SOURCE 1
#include <string.h>

#include "winc1500.h"
#include "pkg/driver/include/m2m_wifi.h"
#include "pkg/driver/source/m2m_hif.h"
#include "winc1500_internal.h"

#include "xtimer.h"
#include "log.h"

#define WINC1500_INTERNAL_START_HANDLER (1)
#define WINC1500_INTERNAL_STOP_HANDLER  (2)

kernel_pid_t _pid = KERNEL_PID_UNDEF;
char _stack[WINC1500_NETDEV_STACKSIZE];
msg_t _queue[WINC1500_NETDEV_QUEUE_LEN];
msg_t _mbox_msgs[WINC1500_NETDEV_MBOX_LEN];

winc1500_t *winc1500_dev = NULL;

char _ssid[WINC1500_MAX_SSID_LEN + 1];
winc1500_ap_t _ap;
uint8_t *_mac_addr;

static void *_event_handler(void *arg);
static int _wait_for_event(msg_t *response, uint16_t event, uint16_t error_event);
winc1500_sec_flags_t _sec_module2driver(tenuM2mSecType module_sec);
tenuM2mSecType _sec_driver2module(winc1500_sec_flags_t sec);


static void *_event_handler(void *arg)
{
    (void)arg;

    _pid = thread_getpid();
    
    msg_init_queue(_queue, WINC1500_NETDEV_QUEUE_LEN);
    while (1) {
        msg_t msg;
        msg_receive(&msg);
        if (msg.type == WINC1500_INTERNAL_START_HANDLER) {
            while (1) {
                /* Due to non-deterministic behavior of m2m_wifi_handle_events()
                 * e.g.) Wi-Fi disconnection event before reading RSSI.
                 * we should let this function called more than one time until
                 * it catches a desired event defined in _wait_for_event.
                 */
                m2m_wifi_handle_events(NULL);
                if (msg_try_receive(&msg) == 1) {
                    if (msg.type == WINC1500_INTERNAL_STOP_HANDLER) {
                        break;
                    }
                }
                xtimer_usleep(1000);
            }
        }
    }
    /* Shouldn't be reached */
    return NULL;
}


static int _wait_for_event(msg_t *response, uint16_t event, uint16_t error_event)
{
    int result = WINC1500_ERR;
    msg_t msg_req, msg_resp;
    /* Wake up handler */
    msg_req.content.value = 0;
    msg_req.type = WINC1500_INTERNAL_START_HANDLER;
    msg_send(&msg_req, _pid);
    /* wait for event message */
    while (true) {
        // TODO: Timeout condition
        mbox_get(&winc1500_dev->event_mbox, &msg_resp);
        if (msg_resp.type & event) {
            result = WINC1500_OK;
            goto done;
        }
        else if (msg_resp.type & error_event) {
            result = WINC1500_ERR;
            goto done;
        }
    }
done:
    /* Sleep handler */
    msg_req.content.value = 0;
    msg_req.type = WINC1500_INTERNAL_STOP_HANDLER;
    msg_send(&msg_req, _pid);
    /* Pass response */
    *response = msg_resp;
    return result;	
}


winc1500_sec_flags_t _sec_module2driver(tenuM2mSecType module_sec)
{
    winc1500_sec_flags_t sec = WINC1500_SEC_UNKNOWN;
    switch (module_sec) {
	    case M2M_WIFI_SEC_INVALID:
            sec = WINC1500_SEC_UNKNOWN;
            break;
	    case M2M_WIFI_SEC_OPEN:
            sec |= WINC1500_SEC_FLAGS_OPEN;
            break;
	    case M2M_WIFI_SEC_WPA_PSK:
            sec |= WINC1500_SEC_FLAGS_WPA;
            sec |= WINC1500_SEC_FLAGS_WPA2;
            break;
	    case M2M_WIFI_SEC_WEP:
            sec |= WINC1500_SEC_FLAGS_WEP;
            break;
	    case M2M_WIFI_SEC_802_1X:
            sec |= WINC1500_SEC_FLAGS_WPA;
            sec |= WINC1500_SEC_FLAGS_WPA2;
            sec |= WINC1500_SEC_FLAGS_ENTERPRISE;
            break;
    }
    return sec;
}


tenuM2mSecType _sec_driver2module(winc1500_sec_flags_t sec)
{
    tenuM2mSecType module_sec = M2M_WIFI_SEC_INVALID;
    if (sec & WINC1500_SEC_FLAGS_OPEN) {
        module_sec = M2M_WIFI_SEC_OPEN; 
    }
    else if ((sec & WINC1500_SEC_FLAGS_WPA) || (sec & WINC1500_SEC_FLAGS_WPA2)) {
        module_sec = M2M_WIFI_SEC_WPA_PSK;
    }
    else if (sec & WINC1500_SEC_FLAGS_WEP) {
        module_sec = M2M_WIFI_SEC_WEP;
    }

    if ((module_sec == M2M_WIFI_SEC_WPA_PSK) && (sec & WINC1500_SEC_FLAGS_ENTERPRISE)) {
        module_sec = M2M_WIFI_SEC_802_1X;
    }
    return module_sec;
}

void winc1500_setup(winc1500_t *dev, const winc1500_params_t *params)
{
#ifdef MODULE_GNRC_NETDEV
    dev->netdev.driver = &netdev_driver_winc1500;
#endif
    dev->params = *params;
}


int winc1500_init(winc1500_t *dev, const winc1500_params_t *params)
{
    /* Copy parameters to the global variable */
    if (winc1500_dev != NULL) {
        /* Currently multiple instances not allowed. */
        return WINC1500_ERR;
    }
    winc1500_dev = dev;
    dev->params = *params;

    spi_init(dev->params.spi);
    _lock(dev);

    /* Initialize the BSP. */
    nm_bsp_init();

    tstrWifiInitParam wifi_param;
    memset((uint8_t *)&wifi_param, 0, sizeof(tstrWifiInitParam));

    /* Register a callback. But replaced by the custom callback.
     *  See process_event() in winc1500_callback.c */
    wifi_param.pfAppWifiCb = NULL; 
    /* Setting Ethernet bypass mode. The ethernet buffer and callback will 
     *  be managed by netdev driver */
#ifdef MODULE_GNRC_NETDEV
    wifi_param.strEthInitParam.u8EthernetEnable = M2M_WIFI_MODE_ETHERNET;
    wifi_param.strEthInitParam.au8ethRcvBuf = NULL;
    wifi_param.strEthInitParam.u16ethRcvBufSize = 0;
    wifi_param.strEthInitParam.pfAppEthCb = NULL;
#endif
    if (M2M_SUCCESS != m2m_wifi_init(&wifi_param)) {
        return WINC1500_ERR;
    }
    /* Register WINC1500's internal wifi callback instead of the driver's callback
     *  for RIOT GNRC
     */
	hif_register_cb(M2M_REQ_GROUP_WIFI, _wifi_cb);

    /* Initialize winc1500 struct */
    mutex_init(&dev->mutex);
    mbox_init(&dev->event_mbox, _mbox_msgs, WINC1500_NETDEV_MBOX_LEN);
    //dev.ip_addr = 0;
    
    /* Create WINC1500 thread */
    kernel_pid_t pid = thread_create(_stack, sizeof(_stack),
                  THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                  _event_handler, NULL, "winc1500");
    if (pid <= 0) {
        return WINC1500_ERR;
    }

    dev->state = 0 | WINC1500_STATE_INIT;
    dev->state |= WINC1500_STATE_IDLE;

    _unlock(dev);
    return WINC1500_OK;
}


int winc1500_scan(winc1500_t *dev)
{
    int result = WINC1500_OK;
    msg_t msg_resp;
    if (!(dev->state & WINC1500_STATE_IDLE)) {
        return WINC1500_ERR;
    }
    _lock(dev);
    /* Request scan. */
    result = m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
    if (M2M_SUCCESS != result) {
        result = WINC1500_ERR;
        goto done;
    }

    result = _wait_for_event(&msg_resp, WINC1500_EVENT_SCAN_DONE, 
                                            WINC1500_EVENT_NOTHING);

done:
    _unlock(dev);
    if (WINC1500_OK == result) {
        return (int)msg_resp.content.value;
    }
    return result;
}


int winc1500_read_ap(winc1500_t *dev, winc1500_ap_t *ap_result, uint8_t ap_num)
{
    int result = WINC1500_OK;
    msg_t msg_resp;
    if (!(dev->state & WINC1500_STATE_IDLE)) {
        return WINC1500_ERR;
    }
    _lock(dev);
    /* Request scan. */
    result = m2m_wifi_req_scan_result(ap_num);
    if (M2M_SUCCESS != result) {
        result = WINC1500_ERR;
        goto done;
    }

    result = _wait_for_event(&msg_resp, WINC1500_EVENT_SCAN_RESULT, 
                                      WINC1500_EVENT_NOTHING);
done:
    _unlock(dev);
    if (WINC1500_OK == result) {
        winc1500_ap_t *ap_info = (winc1500_ap_t *)msg_resp.content.ptr;
        strncpy(ap_result->ssid, ap_info->ssid, WINC1500_MAX_SSID_LEN);
        ap_result->rssi = ap_info->rssi;
        ap_result->sec = _sec_module2driver(ap_info->sec);
    }
    return result;
}


int winc1500_get_mac_addr(winc1500_t *dev, uint8_t *addr)
{
    int result = WINC1500_OK;
    if (!(dev->state & WINC1500_STATE_INIT)) {
        return WINC1500_ERR;
    }
    _lock(dev);
    
    /* Get MAC Address. */
    m2m_wifi_get_mac_address(addr);
    LOG_DEBUG("MAC Address : ");
    LOG_DEBUG("%02X:%02X:%02X:%02X:%02X:%02X\n",
            addr[0], addr[1],
            addr[2], addr[3], 
            addr[4], addr[5]);
    _unlock(dev);
    return result;
}


int winc1500_connect(winc1500_t *dev, const winc1500_ap_t *ap_info)
{
    int result = WINC1500_OK;
    msg_t msg_resp;
    if (!(dev->state & WINC1500_STATE_IDLE)) {
        return WINC1500_ERR;
    }
    _lock(dev);

    /* Request scan. */
    result = m2m_wifi_connect(ap_info->ssid,
                        strnlen(ap_info->ssid, WINC1500_MAX_SSID_LEN),
                        _sec_driver2module(ap_info->sec),
                        (void *)ap_info->password, M2M_WIFI_CH_ALL);
    if (M2M_SUCCESS != result) {
        result = WINC1500_ERR;
        goto done;
    }

    // TODO: wait until connected or DHCP?
    result = _wait_for_event(&msg_resp, WINC1500_EVENT_CON_STATE_CONNECTED, 
                WINC1500_EVENT_NOTHING | WINC1500_EVENT_CON_STATE_DISCONNECTED);
done:
    _unlock(dev);
    return result;
}


int winc1500_connect_list(winc1500_t *dev, const winc1500_ap_t ap_info[])
{
    // TODO: Implement
    return -1;
}


int winc1500_connection_info(winc1500_t *dev, winc1500_ap_t *ap_result, uint8_t *mac_addr)
{
    int result = WINC1500_OK;
    msg_t msg_resp;
    if (!(dev->state & WINC1500_STATE_CONNECTED)) {
        return WINC1500_ERR;
    }
    _lock(dev);

    _mac_addr = mac_addr;
    /* Request scan. */
    result = m2m_wifi_get_connection_info();
    if (M2M_SUCCESS != result) {
        result = WINC1500_ERR;
        goto done;
    }

    result = _wait_for_event(&msg_resp, WINC1500_EVENT_CONN_INFO, 
                                        WINC1500_EVENT_NOTHING);
done:
    _unlock(dev);
    if (WINC1500_OK == result) {
        winc1500_ap_t *ap_info = (winc1500_ap_t *)msg_resp.content.ptr;
        strncpy(ap_result->ssid, ap_info->ssid, WINC1500_MAX_SSID_LEN);
        ap_result->rssi = ap_info->rssi;
        ap_result->sec = _sec_module2driver(ap_info->sec);
    }
    return result;
}


int winc1500_disconnect(winc1500_t *dev)
{
    int result = WINC1500_OK;
    msg_t msg_resp;
    if (!(dev->state & WINC1500_STATE_CONNECTED)) {
        return WINC1500_ERR;
    }
    _lock(dev);

    /* Request scan. */
    result = m2m_wifi_disconnect();
    if (M2M_SUCCESS != result) {
        result = WINC1500_ERR;
        goto done;
    }

    // TODO: wait until connected or DHCP?
    result = _wait_for_event(&msg_resp, WINC1500_EVENT_CON_STATE_DISCONNECTED, 
                WINC1500_EVENT_NOTHING);
done:
    _unlock(dev);
    return result;
}
