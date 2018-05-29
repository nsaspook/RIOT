APPLICATION = rn4020_riot
RIOTBASE ?= $(CURDIR)/
BOARD ?= pic32-cpicmzef
USEMODULE += xtimer
QUIET ?= 1

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP ?= 1

FEATURES_REQUIRED += periph_hwrng
FEATURES_REQUIRED += periph_spi
FEATURES_REQUIRED += periph_timer
FEATURES_REQUIRED += periph_uart
FEATURES_REQUIRED += periph_dac
FEATURES_REQUIRED += periph_adc

include $(RIOTBASE)/Makefile.include
