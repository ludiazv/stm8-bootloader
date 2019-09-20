# stm8-bootloader 
[![Build Status](https://travis-ci.org/ludiazv/stm8-bootloader.svg?branch=master)](https://travis-ci.org/ludiazv/stm8-bootloader)

Enhanced version of @lujji bootloader for STM8 supporting delay activation,led flash and i2c programming.

Major credit of this software are for @lujji. Please check the original repository [here](https://github.com/lujji/stm8-bootloader) and the awesome post were bootloaders internals are [explained](https://lujji.github.io/blog/serial-bootloader-for-stm8).


# Addtional features:

 * Delayed activation: Instead of using boot pin to activate the bootaloader. The bootloader waits arround 300ms for activation (serial or I2C activity). Parts such STM8S001J3 have limited number of pins and no reset pin. Delay activation is usefull in this scenario.
 * I2C upload: Use I2C instead of uart to upload code.
 * Flash activity notification: Led will flash during bootloader activity.
 * Calculation of base address: complication script calculates code location.


## Features

* **small** - fits in 547 bytes (SDCC v3.6.8 #9951) - and <700 bytes for I2C uploader
* **fast** - uploads 4k binary in 1 second @115200bps / I2C support 100khz or 400khz bus frequency.
* **configurable** - can be adjusted for parts with different flash block size

## Configuration

The default configuration targets low-density devices (STM8S003). To compile for a different target `MCU` and `FAMILY` makefile variables are adjusted accordingly.

Bootloader configuration is located in `config.h`.
* **BLOCK_SIZE** - flash block size according to device datasheet. This should be set to 64 for low-density devices or 128 for devices with >8k flash.
* **BOOT_ADDR** - application boot address.
* **BOOT_PIN** - entry jumper. This is set to PD4 by default.
* **RELOCATE_IVT** - when set to 1 (default) the interrupt vectors are relocated. When set to 0 the bootloader will overwrite it's own interrupt vector table with the application's IVT, thus eliminating additional CPU overhead during interrupts. Write-protection cannot be used in this case and resulting binary is slightly larger.
* **I2C_ADDR** - 7bit I2C address of the bootloader. If undefined the bootloader will use serial interface
* **DELAY_COUNT** - 16bit integer with a delay counter. 0xFFFF will provide a maximun delay of about 300ms. If this setting is defined **BOOT_PIN** will not be used.
* **FLASH_PIN** - Output pin of the led for signaling bootloader activity. if undefined the led signaling will not be used making the bootloader smaller.

### Changing boot address
Boot address must be a multiple of BLOCK_SIZE. Address is set in 2 places:
 * config.h
 * init.s

Main application is compiled with `--code-loc <address>` option. When RELOCATE_IVT is set to 0, 0x80 must be subtracted from application address and isr29 must be implemented: `void dummy_isr() __interrupt(29) __naked { ; }`.

Makefile will include a small script to compute the values for you.


## Build instructions
Build and flash the bootloader:

``` bash
$ make && make flash
```

Enable write-protection (UBC) on pages 0-9 _(TODO: must be adjusted for STML)_:

``` bash
$ make opt-set
```

## Uploading the firmware

There is a demo application inside `app` directory which toggles PD4 via interrupts. To upload the application short PD3 to ground, power-cycle the MCU and run the uploader utility. DTR pin on UART-USB converter can be connected to RESET pin on STM8 for automatic reset.

``` bash
$ cd app && make
$ python ../uploader/boot.py -p /dev/ttyUSB0 firmware.bin
```

For I2C using a i2c-dev capable linux device (such a RPI or SBC):

``` bash
$ cd app && make
$ python ../uploader/bi2c.py -p /dev/i2c-1 firmware.bin

```
