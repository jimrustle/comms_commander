# Comms Commander

This software repository holds the firmware for the NEUDOSE UHF communications subsystem.

[http://mcmasterneudose.ca/](http://mcmasterneudose.ca/)

The UHF communications subsystem is controlled by an STM32L071 microcontroller (ARM Cortex-M0+), and interfaces with a Stensat CANSAT, and a Texas Instruments CC1125.

Currently, the firmware is written in bare-metal (ie. without libgcc.a) using the STM32 Low Layer driver libraries, but we would like to use a real-time operating system, such as FreeRTOS in combination with CMSIS libraries in a future development revision.


## Current file list as of 2017-09-15:
```
.
├── config                      | STM32CubeMX configuration and auto-generated source files
│
├── debug                       | openocd script files to interface with the STML071
│   └── openocd
│
├── libs                        | libraries
│   ├── alternate_libs          |   contains the libopencm3 and CMSIS libraries, both are good alternate library choices for further development
│   └── stm32l0_low_level       |   ST's Low Layer driver libraries
├── script                      | this is supposed to be a graphical tool to debug the UHF subsystem (it's low priority so non-functional for now)
│
└── src
    ├── alternates              | basically mainline development but using a more full-featured (not hand-written lol) linker script and vector/startup file
    └── main_development        | header files removed from this listing for brevity (except assert.h, since it's standalone)
        │
        ├── stm32l0xx[...].c    |   ST's Low Layer drivers
        ├── system_stm32l0xx.c
        │
        ├── Makefile            |   GNUMakefile
        │
        ├── assert.h            |   custom assert macro - crashes the application whilst using the debugger
        ├── command.c           |   UART command interpreter
        ├── comms_command.c     |   main entry point, interrupt definitons
        ├── log.c               |   logging functions to print strings to either USART1 or USART2
        ├── peripherals.c       |   initialization functions for each of the peripherals (system clocks, USART, SPI, GPIO, etc)
        ├── print_queue.c       |   queue implementation to handle printing of characters to each USART - note: interrupt-controlled and has a race condition (FIXME: 2017-09-15)
        ├── radio.c             |   high-level radio commands
        ├── radio_cc1125.c      |   CC1125 driver
        ├── spi.c               |   SPI driver - both bitbang and hardware implementations are included
        │
        ├── custom.ld           |   custom linker script (stolen from somewhere actually, but I forgot the source - FIXME: pls find)
        │
        ├── startup.c           |   reset handlers and whatever (stolen - FIXME: find source)
        └── vectors.c           |   vector table                (stolen - FIXME: find source)
```
