#-----------------------------------------------------------------
# main make file for the stm32 dev template
# github.com/toranova
#-----------------------------------------------------------------

# TODO: project name
PROJECT := template_project

# TODO: project source location
SRC_PROJECT := src
#SRC_PROJECT := examples/usb_eth

# TODO: change the following to select for diff boards
include nucleoh743zi2.mk

include make.mk

# user CFLAGS
CFLAGS += \
  -DHELLO=1 \

# user includes
INC += \
  $(SRC_PROJECT) \

# user sources
SRC_C += \
	$(wildcard $(SRC_PROJECT)/*.c) \

# (optional, comment out if not using this lib)
# include tinyusb/networking and lwip stack
include tinyusb_net_lwip.mk

include rules.mk
