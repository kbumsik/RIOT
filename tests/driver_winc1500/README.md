# ATWINC1500 Wifi Module Driver

## WINC1500 Xplained Pro Pinmap

### ATWINC1500 Connection

| Pin | Function  |
|:---:|-----------|
| 5   | RESET_N   |
| 6   | WAKE      |
| 9   | IRQ_N     |
| 10  | CHIP_EN   |
| 15  | SPI_SSN   |
| 16  | SPI_MOSI  |
| 17  | SPI_MISO  |
| 18  | SPI_SCK   |
| 19  | GND       |
| 20  | VCC       |

### ATECC508A Crypto device Connection

| Pin | Function  |
|:---:|-----------|
| 11  | I2C_SDA   |
| 12  | I2C_SCL   |


## Porting note

- MKR1000 is not responding.
    - I suspect it's interrupt problem
    - Need to look at intruppt counter
    ```
    static void isr(void)
    {
    	gu8Interrupt++;
    #ifdef NM_LEVEL_INTERRUPT
    	nm_bsp_interrupt_ctrl(0);
    #endif
    }
    ```
    in m2m_hif.c
- moving m2m_wifi_connect outside