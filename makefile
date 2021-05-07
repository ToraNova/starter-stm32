#-----------------------------------------------------------------
# main make file for the stm32 dev template
# github.com/toranova
#-----------------------------------------------------------------

# TODO: project name
PROJECT := blinky_dma

# TODO: project source location
SRC_PROJECT := src

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
SRC_C += $(wildcard $(SRC_PROJECT)/*.c)

include rules.mk
