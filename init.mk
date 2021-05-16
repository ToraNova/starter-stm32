# ---------------------------------------
# Common make definition for all examples
# modified from tinyusb's stack
# ---------------------------------------

# Build directory
BUILD := build/$(BOARD)

#PROJECT := $(BOARD)-$(notdir $(CURDIR))
#BIN := $(TOP)/_bin/$(BOARD)/$(notdir $(CURDIR))

# Handy check parameter function
check_defined = \
    $(strip $(foreach 1,$1, \
    $(call __check_defined,$1,$(strip $(value 2)))))
__check_defined = \
    $(if $(value $1),, \
    $(error Undefined make flag: $1$(if $2, ($2))))

#-------------- Cross Compiler  ------------
# Can be set by board, default to ARM GCC
CROSS_COMPILE ?= arm-none-eabi-

CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
GDB = $(CROSS_COMPILE)gdb
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size
MKDIR = mkdir

ifeq ($(CMDEXE),1)
  CP = copy
  RM = del
else
  SED = sed
  CP = cp
  RM = rm
endif

# Compiler Flags
CFLAGS += \
  -ggdb \
  -fdata-sections \
  -ffunction-sections \
  -fsingle-precision-constant \
  -fno-strict-aliasing \
  -Wdouble-promotion \
  -Wstrict-prototypes \
  -Wall \
  -Wextra \
  -Wfatal-errors \
  -Werror-implicit-function-declaration \
  -Wfloat-equal \
  -Wundef \
  -Wshadow \
  -Wwrite-strings \
  -Wsign-compare \
  -Wmissing-format-attribute \
  -Wunreachable-code \
  -Wcast-align \
  #-Werror \

# Debugging/Optimization
ifeq ($(OPTI), 1)
  # optimize for code size if make OPTI=1
  CFLAGS += -Os
else
  # default (debugging)
  CFLAGS += -Og
endif

# Log level is mapped to TUSB DEBUG option
#ifneq ($(LOG),)
#  CMAKE_DEFSYM +=	-DLOG=$(LOG)
#  CFLAGS += -DCFG_TUSB_DEBUG=$(LOG)
#endif
#
## Logger: default is uart, can be set to rtt or swo
#ifneq ($(LOGGER),)
#	CMAKE_DEFSYM +=	-DLOGGER=$(LOGGER)
#endif
#
#ifeq ($(LOGGER),rtt)
#  CFLAGS += -DLOGGER_RTT -DSEGGER_RTT_MODE_DEFAULT=SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL
#  RTT_SRC = lib/SEGGER_RTT
#  INC   += $(TOP)/$(RTT_SRC)/RTT
#  SRC_C += $(RTT_SRC)/RTT/SEGGER_RTT.c
#else ifeq ($(LOGGER),swo)
#  CFLAGS += -DLOGGER_SWO
#endif
