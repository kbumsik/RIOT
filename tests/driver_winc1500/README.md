# ATWINC1500 Wifi Module Driver

## How to use the test program

### Debugging support

To get more debugging messages while developing you can enable debugging mode
by adding `CFLAGS += -DLOG_LEVEL=LOG_DEBUG` in `Makefile`. THis line is commented
out by default.

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

for ping?
NMI_API sint8 m2m_ping_req(uint32 u32DstIP, uint8 u8TTL, tpfPingCb fpPingCb);?

- Arduino Bootloader
    - https://github.com/AtmelUniversityFrance/atmel-samd21-xpro-boardmanagermodule/wiki

- Unstable UDP

- Ping callback
- WPS
- useful things:
    - If it's over TCP_SOCK_MAX it's UDP socket 
- Partial packet calls
    - http://www.avrfreaks.net/forum/winc1500-partial-socket-receive-calls

- ISR struct:
    - gstrHifCxt is used for event.