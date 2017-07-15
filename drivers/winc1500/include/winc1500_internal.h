/*
 * Copyright (C) 2013 Alaeddine Weslati <k.bumsik@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_winc1500
 * @{
 *
 * @file
 * @brief       Internal interfaces for WINC1500 drivers
 *
 * @author      Bumsik Kim <k.bumsik@gmail.com>
 */

#ifndef WINC1500_INTERNAL_H
#define WINC1500_INTERNAL_H


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name WINC1500 Socket API definitions
 * @{
 */
#define WINC1500_MAX_TCP_SOCKET         (7)
#define WINC1500_MAX_UDP_SOCKET         (4)
/** @} */

/**
 * @name WINC1500 driver configuration
 * @{
 */
#define WINC1500_NETDEV_MSG_TYPE_EVENT  (0x1236)
#define WINC1500_NETDEV_STACKSIZE       (256 + 512)
#define WINC1500_NETDEV_QUEUE_LEN       (1)
#define WINC1500_NETDEV_MBOX_LEN        (1)
/** @} */

/**
 * @brief   Flags for device internal states (see datasheet)
 * @{
 */
#define WINC1500_STATE_INIT          (1 << 0) /**< Driver is initialized */
#define WINC1500_STATE_IDLE          (1 << 1) /**< idle state after init */
#define WINC1500_STATE_STA           (1 << 2) /**< connected to an AP */
#define WINC1500_STATE_CONNECTED     (1 << 3) /**< connected to an AP */
#define WINC1500_STATE_IP_OBTAINED   (1 << 4) /**< IP is obtained from DHCP server */
/** Others are not used yet */
#define WINC1500_STATE_AP            (1 << 5) /**< Reserved. Wi-Fi AP mode */
#define WINC1500_STATE_SNIFFER       (1 << 6) /**< Reserved. WINC1500 sniffer mode */
#define WINC1500_STATE_P2P           (1 << 7) /**< Reserved. P2P mode (Wi-Fi Direct) */
/** @} */

/**
 * @brief   Flags for device internal states (see datasheet)
 * @{
 */
typedef enum {
    WINC1500_EVENT_WIFI_NOTHING =                   (0),
    WINC1500_EVENT_WIFI_OTHERS =                    (1<<0),
    /* From M2M_WIFI_RESP_CON_STATE_CHANGED */
    WINC1500_EVENT_WIFI_CON_STATE_CONNECTED =       (1<<1),
    WINC1500_EVENT_WIFI_CON_STATE_DISCONNECTED =    (1<<2),
    /* From M2M_WIFI_RESP_SCAN_DONE */
    WINC1500_EVENT_WIFI_SCAN_DONE =                 (1<<3),
    /* From M2M_WIFI_RESP_SCAN_RESULT */
    WINC1500_EVENT_WIFI_SCAN_RESULT =               (1<<4),
    /* From M2M_WIFI_RESP_CURRENT_RSSI */
    WINC1500_EVENT_WIFI_CURRENT_RSSI =              (1<<5)
} winc1500_wifi_cb_msg_t;
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* WINC1500_INTERNAL_H */
/** @} */
