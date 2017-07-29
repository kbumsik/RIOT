APPLICATION = driver_winc1500
include ../Makefile.tests_common

BOARD_INSUFFICIENT_MEMORY := msb-430 msb-430h nucleo32-f031 nucleo32-f042 \
                             nucleo32-l031 nucleo-l053 stm32f0discovery telosb \
                             weio z1

FEATURES_REQUIRED = periph_spi periph_gpio

USEMODULE += winc1500
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps

USEMODULE += gnrc_netdev
USEMODULE += gnrc_netdev_default
USEMODULE += auto_init_gnrc_netif
USEMODULE += gnrc_ipv6_router_default
USEMODULE += gnrc_icmpv6_echo

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DDEVELHELP -g3 -ggdb3 -O0 -Og

# Debugging flags. Uncomment to enable debugging mode
#CFLAGS += -DLOG_LEVEL=LOG_DEBUG 
#CFLAGS += -DENABLE_DEBUG=1


# lower pktbuf size
CFLAGS += -DGNRC_PKTBUF_SIZE=2048

# set board specific peripheral configurations
ifneq (,$(filter samr21-xpro,$(BOARD)))
  # Settings for SAMR21-XPRO Board
  CFLAGS += -DWINC1500_SPI=\(\SPI_DEV\(1\)\)
  CFLAGS += -DWINC1500_SPI_CS_PIN=\(\GPIO_PIN\(PB,3\)\)
  CFLAGS += -DWINC1500_INTN_PIN=\(\GPIO_PIN\(PA,22\)\)

  CFLAGS += -DWINC1500_RESET_PIN=\(\GPIO_PIN\(PA,13\)\)
  CFLAGS += -DWINC1500_CHIP_EN_PIN=\(\GPIO_PIN\(PA,23\)\)
  CFLAGS += -DWINC1500_WAKE_PIN=\(\GPIO_PIN\(PA,28\)\)

else ifneq (,$(filter samd21-xpro,$(BOARD)))
  # Settings for SAMD21-XPRO Board
  CFLAGS += -DWINC1500_SPI=\(SPI_DEV\(0\)\)
  CFLAGS += -DWINC1500_SPI_CS_PIN=\(GPIO_PIN\(PA,5\)\)
  CFLAGS += -DWINC1500_INTN_PIN=\(GPIO_PIN\(PB,4\)\)

  CFLAGS += -DWINC1500_RESET_PIN=\(GPIO_PIN\(PB,6\)\)
  CFLAGS += -DWINC1500_CHIP_EN_PIN=\(GPIO_PIN\(PB,5\)\)
  CFLAGS += -DWINC1500_WAKE_PIN=\(GPIO_PIN\(PB,7\)\)
endif

include $(RIOTBASE)/Makefile.include
