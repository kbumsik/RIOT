WINC1500_SUBDIR = common bsp bus_wrapper driver socket spi_flash
DIRS += $(addprefix pkg/, $(addsuffix /source, $(WINC1500_SUBDIR)))

CFLAGS += -I$(CURDIR)
ifneq (,$(filter gnrc_netdev,$(USEMODULE)))
  # In case we use GNRC the ethernet bypass mode should be enabled.
  CFLAGS += -DETH_MODE
endif

include $(RIOTBASE)/Makefile.base
