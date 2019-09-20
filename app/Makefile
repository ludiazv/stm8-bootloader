# HOST_PLAT=linux_x86_64 
HOST_PLAT=darwin_x86_64

TOOLCHAIN=../toolchain/$(HOST_PLAT)

MCU     ?= stm8s003f3
FAMILY  ?= STM8S
ARCH     = stm8

F_CPU   ?= 2000000
TARGET  ?= main.ihx

SRCS    := $(wildcard *.c)
OBJS     = $(SRCS:.c=.rel)

CC       = $(TOOLCHAIN)/toolchain-sdcc/bin/sdcc
OBJCOPY  = $(TOOLCHAIN)/toolchain-sdcc/bin/sdobjcopy
CFLAGS   = -m$(ARCH) -p$(MCU) -D$(FAMILY)
CFLAGS  += -DF_CPU=$(F_CPU)UL -I. -I$(LIBDIR)
CFLAGS  += --stack-auto --noinduction --use-non-free
LDFLAGS  = -m$(ARCH) -l$(ARCH) --out-fmt-ihx
### Serial bootloader
### ------------------
## RELOCATE_IVT set to 0:
#LDFLAGS += --code-loc 0x8200
## RELOCATE_IVT set to 1:
#LDFLAGS += --code-loc 0x8280
### ---------------------

### I2C bootloader
## RELOCATE_IVT set to 0:
#LDFLAGS += --code-loc 0x8380
## RELOCATE_IVT set to 1:
LDFLAGS += --code-loc 0x8300
### --------------

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $@
	@$(OBJCOPY) -I ihex --output-target=binary $(TARGET) firmware.bin

%.rel: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.map *.asm *.rel *.ihx *.o *.sym *.lk *.lst *.rst *.cdb *.bin

.PHONY: clean all
