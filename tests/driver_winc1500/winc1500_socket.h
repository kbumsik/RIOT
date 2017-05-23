#ifndef DRIVER_WINC1500_H
#define DRIVER_WINC1500_H

#define WINC1500_MAX_TCP_SOCKET 7
#define WINC1500_MAX_UDP_SOCKET 4
typedef enum {
    WIFI_SEC_OPEN =         (1<<0),
    WIFI_SEC_WPA =          (1<<1),
    WIFI_SEC_WPA2 =         (1<<2),
    WIFI_SEC_WEP =          (1<<3),
    WIFI_SEC_ENTERPRISE =   (1<<4),
    WIFI_SEC_WPS =          (1<<5)
} winc1500_wifi_security_t;

// typedef enum {
// 	M2M_WIFI_SEC_INVALID = 0,
// 	/*!< Invalid security type.
// 	*/
// 	M2M_WIFI_SEC_OPEN,
// 	/*!< Wi-Fi network is not secured.
// 	*/
// 	M2M_WIFI_SEC_WPA_PSK,
// 	/*!< Wi-Fi network is secured with WPA/WPA2 personal(PSK).
// 	*/
// 	M2M_WIFI_SEC_WEP,
// 	/*!< Security type WEP (40 or 104) OPEN OR SHARED.
// 	*/
// 	M2M_WIFI_SEC_802_1X
// 	/*!< Wi-Fi network is secured with WPA/WPA2 Enterprise.IEEE802.1x user-name/password authentication.
// 	 */
// }tenuM2mSecType;


#endif /* DRIVER_WINC1500_H */
