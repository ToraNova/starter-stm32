
# template for stm32 projects
what's that? why don't I use stm32cubeIDE? or HAL?
short answer: they suck!
long answer: they prevent you from learning the hardware.

## requirements to use
reference manual for your respective controller (this template uses a [nucleo32h743](https://www.st.com/resource/en/reference_manual/dm00314099-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)).

```
pacman -S make arm-none-eabi-gcc arm-none-eabi-newlib
```
