TARGET := comms_command
OBJECTS := $(TARGET).o startup.o vectors.o printf.o peripherals.o radio.o command.o system_stm32l0xx.o stm32l0xx_hal_cortex.o stm32l0xx_ll_utils.o stm32l0xx_ll_usart.o stm32l0xx_ll_rcc.o stm32l0xx_ll_tim.o

RAM_START := 0x8000000

ARCH	:= arm-none-eabi
MCU		:= -mthumb -mcpu=cortex-m0plus
CC	    := $(ARCH)-gcc
AR		:= $(ARCH)-ar
AS		:= $(ARCH)-as
LD		:= $(ARCH)-gcc
OBJCOPY := $(ARCH)-objcopy
OBJDUMP := $(ARCH)-objdump
SIZE    := $(ARCH)-size
GDB		:= $(ARCH)-gdb

OPT		:= -g -Os $(MCU) -msoft-float -DSTM32L0 -DUSE_FULL_LL_DRIVER -I../../libs/stm32l0_low_level/stm32l0_ll
CSTD	:= -std=c11


CFLAGS	+= $(OPT) $(CSTD) -pedantic
CFLAGS	+= -Wall -Wextra -Wshadow -Wimplicit-function-declaration
CFLAGS	+= -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes
CFLAGS	+= -fno-common -ffunction-sections -fdata-sections -fno-builtin

LDFLAGS		+=  --static -nostartfiles
LDFLAGS		+= -Wl,-Map=$(TARGET).map
LDFLAGS		+= -Wl,--gc-sections

LDLIBS		+= -Wl,--start-group -Wl,--end-group -T custom.ld

#WARNS := -Wall -Wshadow -Wpointer-arith -Wcast-qual -Wcast-align -Wstrict-prototypes -Wmissing-prototypes # -Wconversion -Wno-main
#OPTIMS = -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -finline-limit=3 -fno-inline-small-functions -ffunction-sections -fdata-sections -ffreestanding -Wl,--relax

.PHONY: clean all upload check_splint

all: $(TARGET).elf

%.o: %.s
	$(CC) $(MCU) $(CFLAGS) -c -o $@ $<

%.o: %.c
	$(CC) $(MCU) $(CFLAGS) -c -o $@ $<

$(TARGET).elf: $(OBJECTS)
	$(LD) $(MCU) $(LDFLAGS) $(LDLIBS) $(OBJECTS) -o $(TARGET).elf
	$(OBJCOPY) -O ihex $(TARGET).elf $(TARGET).hex
	$(OBJDUMP) -d $(TARGET).elf > $(TARGET).lst
	$(SIZE) $(TARGET).elf

debug:
	@echo "debug"

dump: $(TARGET).elf
	cat $(TARGET).lst

clean:
	rm $(TARGET).lst $(TARGET).elf $(OBJECTS) $(TARGET).map

upload: $(TARGET).elf
	@# get hex size of our program
	$(eval PROG_SIZE := $(shell arm-none-eabi-size comms_command.elf | tail -n 1 | cut -f 5 | xargs))
	@$(eval PROG_SIZE := $(shell echo "$(PROG_SIZE)" | tr '[:lower:]' '[:upper:]'))
	@# add ten bytes, and write to flash
	$(eval PROG_SIZE := $(shell echo "16o 16i $(PROG_SIZE) A + p" | dc))
	stm32flash /dev/ttyUSB0 -w $(TARGET).hex -v -S $(RAM_START):0x$(PROG_SIZE)

run: $(TARGET).elf
	stm32flash /dev/ttyUSB0 -g 0x0

.PHONY: images clean stylecheck styleclean elf bin hex srec list

