/*
 * Copyright (C) 2017 Bumsik Kim <k.bumsik@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    drivers_winc1500 WINC1500
 * @ingroup     drivers_netdev
 * @brief       Driver for the WINC1500 WiFi module
 * @{
 *
 * @file
 * @brief       Device drvier definition for the WINC1500 WiFi moudle.
 *
 * @details     
 *
 * @author      Bumsik Kim <k.bumsik@gmail.com>
 */

#ifndef WINC1500_H
#define WINC1500_H

#include "mutex.h"
#include "mbox.h"
#include "periph/spi.h"
#include "periph/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name String length definitions from Wi-Fi standard specification.
 * @{
 */
#define WINC1500_MAX_SSID_LEN   (32) /**< Maximum length of SSID without a null charater */
#define WINC1500_MAX_PSK_LEN    (63) /**< Maximum length of PSK without a null charater */
#define WINC1500_MIN_PSK_LEN    (8)  /**< Minimum length of PSK without a null charater */
#define WINC1500_MAC_ADDRES_LEN (6)  /**< MAC address length */
/** @} */

/**
 * @brief   Status and error return codes
 * @{
 */
typedef enum {
    WINC1500_SEC_UNKNOWN =            (0),
    WINC1500_SEC_FLAGS_OPEN =         (1<<0),
    WINC1500_SEC_FLAGS_WPA =          (1<<1),
    WINC1500_SEC_FLAGS_WPA2 =         (1<<2),
    WINC1500_SEC_FLAGS_WEP =          (1<<3),
    WINC1500_SEC_FLAGS_ENTERPRISE =   (1<<4),
    WINC1500_SEC_FLAGS_WPS =          (1<<5)
} winc1500_sec_flags_t;
/** @} */

/**
 * @brief   Status and error return codes
 * @{
 */
enum {
    WINC1500_OK              = (0),      /**< everything was fine */
    WINC1500_ERR             = (-1),     /**< generic winc1500 module error */
    WINC1500_ERR_SPI         = (-2),     /**< error initializing the SPI bus */
    WINC1500_ERR_NODEV       = (-3),     /**< did not detect WINC1500 */
    WINC1500_ERR_FW_VER_MISMATCH = (-4), /**< did not detect WINC1500 */
} ;
/** @} */

/**
 * @brief   Struct containing the needed peripheral configuration
 *
 * These parameters are needed to configure the device at startup.
 * @{
 */
typedef struct {
    char *ssid;
    char *password;
    winc1500_sec_flags_t sec;
    int8_t rssi;
} winc1500_ap_t;
/** @} */

/**
 * @brief   Struct containing the needed peripheral configuration
 *
 * These parameters are needed to configure the device at startup.
 * @{
 */
typedef struct {
    spi_t     spi;          /**< SPI device the module is connected to */
    spi_cs_t  cs_pin;       /**< SPI chip select pin */
    gpio_t    int_pin;      /**< SPI interrupt pin */
    gpio_t    reset_pin;    /**< Module RESET pin */
    gpio_t    en_pin;       /**< Module EN pin */
    gpio_t    wake_pin;     /**< Module WAKE pin */
    spi_clk_t spi_clk;      /**< SPI clock speed to use */
} winc1500_params_t;
/** @} */

/**
 * @brief Device descriptor for the WINC1500 WiFi module
* @{
 */
typedef struct {
#ifdef MODULE_GNRC_NETDEV
    netdev_t netdev;        /**< extended netdev structure */
    uint32_t rx_addr;
#endif
    winc1500_params_t params;   /**< Configuration parameters */
    mutex_t  mutex;             /**< Mutex used for locking concurrent sends */
    //kernel_pid_t   handler_pid; /**< pid of the event handler thread */ 
    mbox_t event_mbox;     /**< Message from the event handler */
    uint32_t ip_addr;           /**< Device's local IPv4 address */     /**< Device's MAC address */
    uint8_t  state;             /**< current state of the radio */
} winc1500_t;
/** @} */

void winc1500_setup(winc1500_t *dev, const winc1500_params_t *params);

/**
 * @brief Initialize a WINC1500 device
 *
 * This function sets SPI pins, initializes the device state structure.
 * It does not initialize the device itself.
 *
 * @param[out] dev          Initialized device descriptor of BME280 device
 * @param[in]  params       The parameters for the BME280 device (sampling rate, etc)
 *
 * @return                  BME280_OK on success
 * @return                  BME280_ERR_I2C
 * @return                  BME280_ERR_NODEV
 * @return                  BME280_ERR_NOCAL
 */
int winc1500_init(winc1500_t *dev, const winc1500_params_t *params);

int winc1500_scan(winc1500_t *dev);

int winc1500_read_ap(winc1500_t *dev, winc1500_ap_t *ap_result, uint8_t ap_num);

int winc1500_read_rssi(winc1500_t *dev, int *output);

int winc1500_get_mac_addr(winc1500_t *dev, uint8_t *addr);

int winc1500_connect(winc1500_t *dev, const winc1500_ap_t *ap_info);

int winc1500_connect_list(winc1500_t *dev, const winc1500_ap_t ap_info[]);

int winc1500_disconnect(winc1500_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* WINC1500_H */
/** @} */
