#include "pkg/driver/include/m2m_wifi.h"
#include "pkg/driver/source/m2m_hif.h"
#include "winc1500.h"
#include "winc1500_internal.h"
#include <string.h>

static void _process_event(uint8_t msg_type, void *payload);


/**
*	@fn			m2m_wifi_cb(uint8 u8OpCode, uint16 sizeu16DataSize, uint32 u32Addr, uint8 grp)
*	@brief		WiFi call back function
*	@param [in]	u8OpCode
*					HIF Opcode type.
*	@param [in]	sizeu16DataSize
*					HIF data length.
*	@param [in]	u32Addr
*					HIF address.
*	@param [in]	grp
*					HIF group type.
*	@author
*	@date
*	@version	1.0
*/
void _wifi_cb(uint8_t opcode, uint16_t size, uint32_t addr)
{
    /* The code is originally from m2m_wifi_cb() in m2m_wifi.c. The code is modified
     *  for GNRC.
     */
    (void) size; /* This one is originally used when CONF_MGMT is set */
    winc1500_event_info_t event_info;
    uint16_t recv_size = 0;
    uint8_t is_done = 0;

    /* Prepare */
    switch (opcode) {
        case M2M_WIFI_RESP_CON_STATE_CHANGED:
            recv_size = sizeof (event_info.state_change);
            break;
        case M2M_WIFI_RESP_GET_SYS_TIME:
            recv_size = sizeof (event_info.sys_time);
            break;
        case M2M_WIFI_RESP_CONN_INFO:
            recv_size = sizeof (event_info.conn_info);
            is_done = 1;
            break;
        case M2M_WIFI_REQ_DHCP_CONF:
            recv_size = sizeof (event_info.ip_config);
            break;
        case M2M_WIFI_REQ_WPS:
            recv_size = sizeof (event_info.wps_info);
            memset((uint8_t *) &event_info, 0, recv_size);
            break;
        case M2M_WIFI_RESP_IP_CONFLICT:
            recv_size = sizeof (event_info.ip_conflicted);
            break;
        case M2M_WIFI_RESP_SCAN_DONE: {
            extern uint8_t gu8scanInProgress;
            gu8scanInProgress = 0;
            recv_size = sizeof (event_info.scan_done);
            break;
        }
        case M2M_WIFI_RESP_SCAN_RESULT:
            recv_size = sizeof (event_info.scan_result);
            break;
        case M2M_WIFI_RESP_CURRENT_RSSI:
            recv_size = 4;
            break;
        case  M2M_WIFI_RESP_CLIENT_INFO:
            recv_size = 4;
            break;
        case M2M_WIFI_RESP_PROVISION_INFO:
            recv_size = sizeof (event_info.prov_info);
            is_done = 1;
            break;
        case M2M_WIFI_RESP_DEFAULT_CONNECT:
            recv_size = sizeof (event_info.default_conn_resp);
            is_done = 1;
            break;
        case M2M_WIFI_RESP_GET_PRNG:
            recv_size = sizeof (event_info.prng_result);
            break;
#ifdef MODULE_GNRC_NETDEV
        case M2M_WIFI_RESP_ETHERNET_RX_PACKET: {
            _unlock(dev);
            dev->rx_addr = addr;
            netdev->event_callback(netdev, NETDEV_EVENT_RX_COMPLETE);
            _lock(dev);
            // Return to callback
            return;
        }
#endif
        default:
            M2M_ERR("REQ Not defined %d\n",opcode);
            return;
    }

    /* Get data from the module */
    if (hif_receive(addr, (uint8_t *) &event_info, recv_size, is_done) == M2M_SUCCESS) {
        if (M2M_WIFI_RESP_GET_PRNG != opcode) {
            _process_event(opcode, &event_info);
        }
        else {
            addr += sizeof(event_info.prng_result);
            recv_size = event_info.prng_result.u16PrngSize;
            is_done = 1;
            if(hif_receive(addr, event_info.prng_result.pu8RngBuff, recv_size, is_done) == M2M_SUCCESS) {
                _process_event(opcode, &event_info);
            }
        }
    }
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
static void _process_event(uint8_t msg_type, void *payload)
{
    msg_t msg;
    msg.type = WINC1500_EVENT_NOTHING;
    msg.content.value = 0;
    winc1500_event_info_t *event = (winc1500_event_info_t *)payload;
    switch (msg_type) {
        case M2M_WIFI_RESP_CON_STATE_CHANGED: {
            LOG_DEBUG("M2M_WIFI_RESP_CON_STATE_CHANGED");
            switch (event->state_change.u8CurrState) {
                case M2M_WIFI_CONNECTED:
                    /*!< WiFi is to connected to AP */
                    msg.type = WINC1500_EVENT_CON_STATE_CONNECTED;
                    LOG_DEBUG("Wi-Fi connected\n");
                    winc1500_dev->state &= ~WINC1500_STATE_IDLE;
                    winc1500_dev->state |= WINC1500_STATE_STA;
                    winc1500_dev->state |= WINC1500_STATE_CONNECTED;
                    break;
                case M2M_WIFI_DISCONNECTED:
                    /*!< WiFi is disconnected from AP */
                    msg.type = WINC1500_EVENT_CON_STATE_DISCONNECTED;
                    LOG_DEBUG("Wi-Fi disconnected\n");
                    winc1500_dev->state |= WINC1500_STATE_IDLE;
                    winc1500_dev->state &= ~WINC1500_STATE_STA;
                    winc1500_dev->state &= ~WINC1500_STATE_CONNECTED;
                    winc1500_dev->state &= ~WINC1500_STATE_IP_OBTAINED;
                    break;
                case M2M_WIFI_UNDEF:
                default:
                    /*!< Undefined status */
                    break;
            }
#ifdef MODULE_GNRC_NETDEV
            /* Update link state to netdev */
            netdev_event_t event_netdev =
                (event->state_change.u8CurrState == M2M_WIFI_CONNECTED) ?
                    NETDEV_EVENT_LINK_UP:
                    NETDEV_EVENT_LINK_DOWN;

            netdev->event_callback(netdev, event_netdev);
#endif
            break;
        }
        case M2M_WIFI_RESP_CONN_INFO:
            LOG_DEBUG("M2M_WIFI_RESP_CONN_INFO");
            /* Copy result */
            _ap.rssi = event->conn_info.s8RSSI;
            _ap.sec = event->conn_info.u8SecType;
            strncpy(_ssid, (char *)event->conn_info.acSSID, WINC1500_MAX_SSID_LEN);
            _ap.ssid = _ssid;
            memcpy(_mac_addr, event->conn_info.au8MACAddress, 6);
            /* set message */
            msg.type = WINC1500_EVENT_CONN_INFO;
            msg.content.ptr = &_ap;
            break;
        case M2M_WIFI_REQ_DHCP_CONF: {
            LOG_DEBUG("M2M_WIFI_REQ_DHCP_CONF");
            /* Called by m2m_wifi_connect() */
            uint8_t *ip_addr = (uint8_t *)payload;
            LOG_DEBUG("IP obtained from DHCP server\n");
            LOG_DEBUG("IP is %u.%u.%u.%u\n",
                    ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3]);
            winc1500_dev->ip_addr = *(uint32_t *)payload;
            winc1500_dev->state |= WINC1500_STATE_IP_OBTAINED;
            break;
        }
        case M2M_WIFI_REQ_WPS:
            LOG_DEBUG("M2M_WIFI_REQ_WPS");
            break;
        case M2M_WIFI_RESP_IP_CONFLICT:
            LOG_DEBUG("M2M_WIFI_RESP_IP_CONFLICT");
            break;
        case M2M_WIFI_RESP_SCAN_DONE:
            LOG_DEBUG("M2M_WIFI_RESP_SCAN_DONE");
            msg.type = WINC1500_EVENT_SCAN_DONE;
            msg.content.value = event->scan_done.u8NumofCh;
            break;
        case M2M_WIFI_RESP_SCAN_RESULT:
            /* Called by m2m_wifi_req_scan_result() */
            LOG_DEBUG("M2M_WIFI_RESP_SCAN_RESULT");
            /* Copy result */
            _ap.rssi = event->scan_result.s8rssi;
            _ap.sec = event->scan_result.u8AuthType;
            strncpy(_ssid, (char *)event->scan_result.au8SSID, WINC1500_MAX_SSID_LEN);
            _ap.ssid = _ssid;
            /* set message */
            msg.type = WINC1500_EVENT_SCAN_RESULT;
            msg.content.ptr = &_ap;
            
            /* display founded AP. */
            LOG_DEBUG("SSID:%s\n", event->scan_result.au8SSID);
            break;
        case M2M_WIFI_RESP_CURRENT_RSSI:
            /* Called by m2m_wifi_req_curr_rssi() */
            LOG_DEBUG("M2M_WIFI_RESP_CURRENT_RSSI");
            int8_t rssi = *(int8_t *)payload;
            msg.type = WINC1500_EVENT_CURRENT_RSSI;
            LOG_DEBUG("RSSI Read: %d\n", rssi);
            msg.content.value = rssi * -1;
            break;
        case M2M_WIFI_RESP_CLIENT_INFO:
            LOG_DEBUG("M2M_WIFI_RESP_CLIENT_INFO");
            break;
        case M2M_WIFI_RESP_PROVISION_INFO:
            LOG_DEBUG("M2M_WIFI_RESP_PROVISION_INFO");
            break;
        case M2M_WIFI_RESP_DEFAULT_CONNECT:
            LOG_DEBUG("M2M_WIFI_RESP_DEFAULT_CONNECT");
            break;
        case M2M_WIFI_RESP_GET_SYS_TIME:
            LOG_DEBUG("M2M_WIFI_RESP_GET_SYS_TIME");
            /* This event is internally used by module firmware 
             * on this event the module will sync time using SNTP */
            msg.type = WINC1500_EVENT_OTHERS;
            LOG_DEBUG("SNTP time event called");
            break;
        default:
            LOG_DEBUG("Unknown WiFi message: %d\n", msg_type);
            break;
    }
    mbox_put(&winc1500_dev->event_mbox, &msg);
}