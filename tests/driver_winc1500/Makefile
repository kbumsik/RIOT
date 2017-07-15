APPLICATION = driver_winc1500
include ../Makefile.tests_common

USEMODULE += shell
USEMODULE += winc1500

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DDEVELHELP -g3 -ggdb3 -O0 -Og

# add current directory to the include path. Putting it in CFLAGS will make
# it go to the beginning, before the standard includes.
# That way xbee_params.h get's included and auto configuration can pick it up.
CFLAGS += -I$(RIOTBASE)/drivers/winc1500

include $(RIOTBASE)/Makefile.include
