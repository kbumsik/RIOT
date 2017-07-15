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
#include "winc1500_internal.h"
#include "pkg/driver/include/m2m_wifi.h"

#include "xtimer.h"
#include "log.h"

#define WINC1500_INTERNAL_START_HANDLER (1)
#define WINC1500_INTERNAL_STOP_HANDLER  (2)

static kernel_pid_t _pid = KERNEL_PID_UNDEF;
static char _stack[WINC1500_NETDEV_STACKSIZE];
static msg_t _queue[WINC1500_NETDEV_QUEUE_LEN];
msg_t _mbox_msgs[WINC1500_NETDEV_MBOX_LEN]; 

winc1500_t winc1500_dev;
winc1500_params_t winc1500_param; /** Global device parameters. This will be
                                used in WINC1500 internall device driver
                                (e.g. nm_bsp_riot.c, nm_bus_wrapper_riot.c) */

char _ssid[WINC1500_MAX_SSID_LEN + 1];
winc1500_ap_t _ap;

static void *_event_handler(void *arg);
static void _wifi_callback(uint8_t msg_type, void *payload);
static int _wait_for_event(msg_t *response, uint16_t event, uint16_t error_event);
winc1500_sec_flags_t _convert_sec_from(tenuM2mSecType module_sec);
tenuM2mSecType _convert_sec_to(winc1500_sec_flags_t sec);

inline static void _acquire(winc1500_t *dev)
{
    mutex_lock(&dev->mutex);
}

inline static void _release(winc1500_t *dev)
{
    mutex_unlock(&dev->mutex);
}


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
static void _wifi_callback(uint8_t msg_type, void *payload)
{
    msg_t msg;
    msg.type = WINC1500_EVENT_WIFI_NOTHING;
    msg.content.value = 0;
    switch (msg_type) {
        case M2M_WIFI_RESP_CON_STATE_CHANGED: {
            LOG_DEBUG("M2M_WIFI_RESP_CON_STATE_CHANGED");
            tstrM2mWifiStateChanged *wifi_state = (tstrM2mWifiStateChanged *)payload;
            switch (wifi_state->u8CurrState) {
                case M2M_WIFI_DISCONNECTED:
                    /*!< WiFi is disconnected from AP */
                    msg.type = WINC1500_EVENT_WIFI_CON_STATE_DISCONNECTED;
                    LOG_DEBUG("Wi-Fi disconnected\n");
                    winc1500_dev.state = WINC1500_STATE_INIT;
                    break;
                case M2M_WIFI_CONNECTED:
                    /*!< WiFi is to connected to AP */
                    msg.type = WINC1500_EVENT_WIFI_CON_STATE_CONNECTED;
                    LOG_DEBUG("Wi-Fi connected\n");
                    winc1500_dev.state &= ~WINC1500_STATE_INIT;
                    winc1500_dev.state |= WINC1500_STATE_STA;
                    winc1500_dev.state |= WINC1500_STATE_CONNECTED;
                    break;
                case M2M_WIFI_UNDEF:
                default:
                    /*!< Undefined status */
                    break;
            }
        }
            break;
        case M2M_WIFI_RESP_CONN_INFO: {
            LOG_DEBUG("M2M_WIFI_RESP_CONN_INFO");
        }
            break;
        case M2M_WIFI_REQ_DHCP_CONF: {
            LOG_DEBUG("M2M_WIFI_REQ_DHCP_CONF");
            /* Called by m2m_wifi_connect() */
            uint8_t *ip_addr = (uint8_t *)payload;
            LOG_DEBUG("IP obtained from DHCP server\n");
            LOG_DEBUG("IP is %u.%u.%u.%u\n",
                    ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3]);
            winc1500_dev.ip_addr = *(uint32_t *)payload;
            winc1500_dev.state |= WINC1500_STATE_IP_OBTAINED;
            break;
        }
        case M2M_WIFI_REQ_WPS: {
            LOG_DEBUG("M2M_WIFI_REQ_WPS");
        }
            break;
        case M2M_WIFI_RESP_IP_CONFLICT: {
            LOG_DEBUG("M2M_WIFI_RESP_IP_CONFLICT");
        }
            break;
        case M2M_WIFI_RESP_SCAN_DONE: {
            LOG_DEBUG("M2M_WIFI_RESP_SCAN_DONE");
            tstrM2mScanDone *scaninfo = (tstrM2mScanDone *)payload;
            msg.type = WINC1500_EVENT_WIFI_SCAN_DONE;
            msg.content.value = scaninfo->u8NumofCh;
        }
            break;
        case M2M_WIFI_RESP_SCAN_RESULT: {
            LOG_DEBUG("M2M_WIFI_RESP_SCAN_RESULT");
            /* Called by m2m_wifi_req_scan_result() */
            tstrM2mWifiscanResult *scan_result = (tstrM2mWifiscanResult *)payload;
            /* Copy result */
            _ap.rssi = scan_result->s8rssi;
            _ap.sec = scan_result->u8AuthType;
            strncpy(_ssid, (char *)scan_result->au8SSID, WINC1500_MAX_SSID_LEN);
            _ap.ssid = _ssid;
            /* set message */
            msg.type = WINC1500_EVENT_WIFI_SCAN_RESULT;
            msg.content.ptr = &_ap;
            
            /* display founded AP. */
            LOG_DEBUG("SSID:%s\n", scan_result->au8SSID);
        }
            break;
        case M2M_WIFI_RESP_CURRENT_RSSI: {
            LOG_DEBUG("M2M_WIFI_RESP_CURRENT_RSSI");
            /* Called by m2m_wifi_req_curr_rssi() */
            int8_t rssi = *(int8_t *)payload;
            msg.type = WINC1500_EVENT_WIFI_CURRENT_RSSI;
            LOG_DEBUG("RSSI Read: %d\n", rssi);
            msg.content.value = rssi * -1;
        }
            break;
        case M2M_WIFI_RESP_CLIENT_INFO: {
            LOG_DEBUG("M2M_WIFI_RESP_CLIENT_INFO");
        }
            break;
        case M2M_WIFI_RESP_PROVISION_INFO: {
            LOG_DEBUG("M2M_WIFI_RESP_PROVISION_INFO");
        }
            break;
        case M2M_WIFI_RESP_DEFAULT_CONNECT: {
            LOG_DEBUG("M2M_WIFI_RESP_DEFAULT_CONNECT");
        }
            break;
        case M2M_WIFI_RESP_GET_SYS_TIME: {
            LOG_DEBUG("M2M_WIFI_RESP_GET_SYS_TIME");
            /* This event is internally used by module firmware 
             * on this event the module will sync time using SNTP */
            msg.type = WINC1500_EVENT_WIFI_OTHERS;
            LOG_DEBUG("SNTP time event called");
        }
            break;
        default: {
            LOG_DEBUG("Unknown WiFi message: %d\n", msg_type);
        }
            break;
    }
    mbox_put(&winc1500_dev.event_mbox, &msg);
}


static void *_event_handler(void *arg)
{
    (void)arg;
    msg_init_queue(_queue, WINC1500_NETDEV_QUEUE_LEN);
    while (1) {
        msg_t msg;
        msg_receive(&msg);
        if (msg.type == WINC1500_INTERNAL_START_HANDLER) {
            while (1) {
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
        mbox_get(&winc1500_dev.event_mbox, &msg_resp);
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


winc1500_sec_flags_t _convert_sec_from(tenuM2mSecType module_sec)
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


tenuM2mSecType _convert_sec_to(winc1500_sec_flags_t sec)
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


int winc1500_init(const winc1500_params_t *params)
{
    /* Copy parameters to the global variable */
    winc1500_param = *params;

    /* Initialize the BSP. */
    nm_bsp_init();

    /* Initialize Wi-Fi parameters structure. */
    tstrWifiInitParam wifi_param;
    memset((uint8_t *)&wifi_param, 0, sizeof(tstrWifiInitParam));

    /* Register a callback */
    wifi_param.pfAppWifiCb = _wifi_callback;
    /* Initialize Wi-Fi driver with data and status callbacks. */
    if (M2M_SUCCESS != m2m_wifi_init(&wifi_param)) {
        return WINC1500_ERR;
    }

    /* Initialize winc1500 struct */
    mutex_init(&winc1500_dev.mutex);
    mbox_init(&winc1500_dev.event_mbox, _mbox_msgs, WINC1500_NETDEV_MBOX_LEN);
    //winc1500_dev.ip_addr = 0;
    winc1500_dev.state = 0 | WINC1500_STATE_INIT;

    /* Create WINC1500 thread */
    _pid = thread_create(_stack, sizeof(_stack),
                  THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                  _event_handler, NULL, "winc1500");

    /* Get MAC Address. */
    m2m_wifi_get_mac_address(winc1500_dev.mac_addr);
    LOG_DEBUG("MAC Address : ");
    LOG_DEBUG("%02X:%02X:%02X:%02X:%02X:%02X\n",
            winc1500_dev.mac_addr[0], winc1500_dev.mac_addr[1],
            winc1500_dev.mac_addr[2], winc1500_dev.mac_addr[3], 
            winc1500_dev.mac_addr[4], winc1500_dev.mac_addr[5]);
    return WINC1500_OK;
}


int winc1500_scan(void)
{
    int result = WINC1500_OK;
    msg_t msg_resp;
    if (!( (winc1500_dev.state & WINC1500_STATE_INIT) || 
           (winc1500_dev.state & WINC1500_STATE_CONNECTED) )) {
        return WINC1500_ERR;
    }
    _acquire(&winc1500_dev);
    /* Request scan. */
    result = m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
    if (M2M_SUCCESS != result) {
        result = WINC1500_ERR;
        goto done;
    }

    result = _wait_for_event(&msg_resp, WINC1500_EVENT_WIFI_SCAN_DONE, 
                                            WINC1500_EVENT_WIFI_NOTHING);

done:
    _release(&winc1500_dev);
    if (WINC1500_OK == result) {
        return (int)msg_resp.content.value;
    }
    return result;
}


int winc1500_read_ap(winc1500_ap_t *ap_result, uint8_t ap_num)
{
    int result = WINC1500_OK;
    msg_t msg_resp;
    if (!(winc1500_dev.state & WINC1500_STATE_INIT)) {
        return WINC1500_ERR;
    }
    _acquire(&winc1500_dev);
    /* Request scan. */
    result = m2m_wifi_req_scan_result(ap_num);
    if (M2M_SUCCESS != result) {
        result = WINC1500_ERR;
        goto done;
    }

    result = _wait_for_event(&msg_resp, WINC1500_EVENT_WIFI_SCAN_RESULT, 
                                      WINC1500_EVENT_WIFI_NOTHING);
done:
    _release(&winc1500_dev);
    if (WINC1500_OK == result) {
        winc1500_ap_t *ap_info = (winc1500_ap_t *)msg_resp.content.ptr;
        strncpy(ap_result->ssid, ap_info->ssid, WINC1500_MAX_SSID_LEN);
        ap_result->rssi = ap_info->rssi;
        ap_result->sec = _convert_sec_from(ap_info->sec);
    }
    return result;
}


int winc1500_connect_single(winc1500_ap_t *ap_info)
{
    int result = WINC1500_OK;
    msg_t msg_resp;
    if (!(winc1500_dev.state & WINC1500_STATE_INIT)) {
        return WINC1500_ERR;
    }
    _acquire(&winc1500_dev);

    /* Request scan. */
    result = m2m_wifi_connect(ap_info->ssid,
                        strnlen(ap_info->ssid, WINC1500_MAX_SSID_LEN),
                        _convert_sec_to(ap_info->sec),
                        (void *)ap_info->password, M2M_WIFI_CH_ALL);
    if (M2M_SUCCESS != result) {
        result = WINC1500_ERR;
        goto done;
    }

    // TODO: wait until connected or DHCP?
    result = _wait_for_event(&msg_resp, WINC1500_EVENT_WIFI_CON_STATE_CONNECTED, 
                WINC1500_EVENT_WIFI_NOTHING | WINC1500_EVENT_WIFI_CON_STATE_DISCONNECTED);
done:
    _release(&winc1500_dev);
    return result;
}

int winc1500_read_rssi(int *output)
{
    int result = WINC1500_OK;
    msg_t msg_resp;
    if (!(winc1500_dev.state & WINC1500_STATE_CONNECTED)) {
        return WINC1500_ERR;
    }
    _acquire(&winc1500_dev);
    /* Request scan. */
    result = m2m_wifi_req_curr_rssi();
    if (M2M_SUCCESS != result) {
        result = WINC1500_ERR;
        goto done;
    }

    result = _wait_for_event(&msg_resp, WINC1500_EVENT_WIFI_CURRENT_RSSI, 
                                        WINC1500_EVENT_WIFI_NOTHING);
done:
    _release(&winc1500_dev);
    if (WINC1500_OK == result) {
        *output = msg_resp.content.value * -1; 
    }
    return result;
}


int winc1500_connect_list(winc1500_ap_t ap_info[])
{
    // TODO: Implement
    return -1;
}


int winc1500_disconnect(void)
{
    //m2m_wifi_disconnect();
    
    return -1;
}
