# TODO: project name
PROJECT := blinky

# TODO: change the following to select for diff boards
include nucleoh743zi2.mk

include make.mk

# user CFLAGS
CFLAGS += \
  -DHELLO=1 \

# user includes
INC += \
  src \

# user sources
SRC_C += $(wildcard src/*.c)

include rules.mk
