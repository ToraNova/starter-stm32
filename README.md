# template for stm32 projects
what's that? why don't I use stm32cubeIDE? or HAL?
short answer: they suck! big time.
long answer: they prevent you from learning the hardware.

this template project is meant to work with just make-files (like how tinyusb does it) and a minimal set of good libraries. ideally, this is intended to kickstart projects.

## requirements to use (effectively)

 1. reference manual for your respective controller (this template uses a [nucleo32h743](https://www.st.com/resource/en/reference_manual/dm00314099-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)).
 2. install requirements (on archlinux)
```
pacman -S make arm-none-eabi-gcc arm-none-eabi-newlib
```
3. download all the libs (or download them as required for less bloat)
```
git submodule update --init lib/*
```

## examples
some examples to help better understand how to use the stm32 or for jump starting a project; all examples in this project uses the nucleoh743zi2 board (I work with what I have).
 - [blinky](examples/nucleoh743_blinky)
 - [uart using dma](examples/nucleoh743_uartdma)
 - [usb-rndis using tinyusb and lwip](examples/nucleoh743_usbrndis)

##  references
this project will never be fruit if not for the following open source project for me to bang my head on to study their sources.

 1. https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
 2. https://github.com/hathach/tinyusb
 3. https://github.com/lwip-tcpip/lwip
 4. https://www.youtube.com/user/sdf3e33
