#-----------------------------------------------------------------
# main make file for the stm32 dev template
# based off the build system from tinyusb
#
# author: toranova (chia_jason96@live.com)
#-----------------------------------------------------------------

# TODO: project name
PROJECT := template_project

# TODO: project source location
#SRC_PROJECT := src
#SRC_PROJECT := examples/nucleoh743_blinky
#SRC_PROJECT := examples/nucleoh743_uartdma
SRC_PROJECT := examples/nucleoh743_usbrndis
#SRC_PROJECT := examples/nucleoh743_pppos

# TODO: change the following to select for diff boards
include nucleoh743zi2.mk

include init.mk

# user CFLAGS
CFLAGS += \
  -DHELLO=1 \

# user includes
INC += \
  $(SRC_PROJECT) \

# user sources
SRC_C += \
	$(wildcard $(SRC_PROJECT)/*.c) \

# TODO: comment the following (or uncomment)
# if use of the libs are required
# tinyusb stack
include tinyusb.mk
# lwip and networking libs
include lwipnet.mk

include rules.mk
