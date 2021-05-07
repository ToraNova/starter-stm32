BOARD = nucleoh743zl2
ST_FAMILY = h7
UF2_FAMILY_ID = 0x6db66082
DEPS_SUBMODULES += st/cmsis_device_$(ST_FAMILY)

ST_CMSIS = st/cmsis_device_$(ST_FAMILY)

CFLAGS += \
  -flto \
  -mthumb \
  -mabi=aapcs \
  -mcpu=cortex-m7 \
  -mfloat-abi=hard \
  -mfpu=fpv5-d16 \
  -nostdlib -nostartfiles \
  -DSTM32H743xx \
  -DHSE_VALUE=32768 \

SRC_S += $(ST_CMSIS)/Source/Templates/gcc/startup_stm32h743xx.s
LD_FILE = linker_scripts/stm32h743xx_flash_sram2.ld

# For flash-jlink target
JLINK_DEVICE = stm32h743zi

# flash target using on-board stlink
flash: flash-stlink

# suppress warning caused by vendor mcu driver
CFLAGS += -Wno-error=maybe-uninitialized -Wno-error=cast-align

# All source paths should be relative to the top level.
SRC_C += \
	$(ST_CMSIS)/Source/Templates/system_stm32$(ST_FAMILY)xx.c \

INC += \
       $(ST_CMSIS)/Include \
	st/CMSIS_5/CMSIS/Core/Include \

# For freeRTOS port source
FREERTOS_PORT = ARM_CM7/r0p1
