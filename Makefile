## Toolchain directory
HOST_PLAT=linux_x86_64 
#HOST_PLAT=darwin_x86_64
TOOLCHAIN=toolchain/$(HOST_PLAT)

## STM8S
MCU     ?= stm8s003f3
FAMILY  ?= STM8S

## STM8L
# MCU     ?= stm8l051f3
# FAMILY  ?= STM8L

ARCH     = stm8

TARGET  ?= main.ihx

SRCS    := $(wildcard *.c)
ASRCS   := $(wildcard *.s)

OBJS     = $(SRCS:.c=.rel)
OBJS    += $(ASRCS:.s=.rel)

CC       = $(TOOLCHAIN)/toolchain-sdcc/bin/sdcc
LD       = $(TOOLCHAIN)/toolchain-sdcc/bin/sdld
AS       = $(TOOLCHAIN)/toolchain-sdcc/bin/sdasstm8
OBJCOPY  = $(TOOLCHAIN)/toolchain-sdcc/bin/sdobjcopy
FLASHT   = $(TOOLCHAIN)/tool-stm8tools/stm8flash
OBJSIZE  = $(TOOLCHAIN)/tool-stm8binutils/bin/stm8-size
ASFLAGS  = -plosgff
CFLAGS   = -m$(ARCH) -p$(MCU) -D$(FAMILY) -I.
CFLAGS  += --stack-auto --noinduction --use-non-free --noinvariant --opt-code-size
LDFLAGS  = -m$(ARCH) -l$(ARCH) --out-fmt-ihx
LDFLAGS += -Wl-bIVT=0x8000 -Wl-bGSINIT=0x8080

all: clean $(TARGET) size

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $@

%.rel: %.s
	$(AS) $(ASFLAGS) $<

%.rel: %.c
	$(CC) $(CFLAGS) -c $< -o $@

flash: $(TARGET)
	$(FLASHT) -c stlinkv2 -p $(MCU) -w $(TARGET)

clean:
	rm -f *.map *.rel *.ihx *.o *.sym *.lk *.lst *.rst *.cdb *.bin *.asm

size:
	@$(OBJCOPY) -I ihex --output-target=binary $(TARGET) $(TARGET).bin
	@./calc_code-loc.sh


## @TODO: separate option-bytes for stm8s and stm8l!
# enable write-protection on first 10 pages
opt-set:
	@echo '0x00 0x09 0xf6 0x00 0xff 0x00 0xff 0x00 0xff 0x00 0xff' | xxd -r > opt.bin
	$(FLASHT) -c stlinkv2 -p stm8s003f3 -s opt -w opt.bin

# reset option-bytes to factory defaults
opt-reset:
	@echo '0x00 0x00 0xff 0x00 0xff 0x00 0xff 0x00 0xff 0x00 0xff' | xxd -r > opt.bin
	$(FLASHT) -c stlinkv2 -p stm8s003f3 -s opt -w opt.bin

dump-opt:
	$(FLASHT) -c stlinkv2 -p $(MCU) -s opt -r dump_opt.bin

dump-flash:
	$(FLASHT) -c stlinkv2 -p $(MCU) -s flash -r dump_flash.bin

erase:
	tr '\000' '\377' < /dev/zero | dd of=empty.bin bs=8192 count=1
	$(FLASHT) -c stlinkv2 -p $(MCU) -s flash -w empty.bin

.PHONY: clean all flash size dump-opt dump-flash erase
