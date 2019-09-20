#!/bin/bash
set -e
echo "=========================================="
echo "Code location calculator"
echo "=========================================="
printf "Boot loader size (bytes):"
SZ_B=$(stat -f %z main.ihx.bin)
echo $SZ_B
SZ_BL=$(expr $SZ_B / 128)
t=$(expr $SZ_B % 128)

if [ $t -ne 0 ]; then
    SZ_BL=$(expr $SZ_BL + 1)
fi

printf "Bootloader size (128 bytes blocks):"
echo $SZ_BL
printf "Booloader size  (64 bytes blocks):"
echo $(expr $SZ_BL + $SZ_BL)

t=$(expr $SZ_BL "*" 128)
OFFSET=$(expr 32768 + $t)
printf "Code loc for RELOCATE_ITV=0 0x%x" $(expr $OFFSET - 128 )
echo 
printf "Code loc for RELOCATE_ITV=1 0x%x" $OFFSET
echo 
echo "=========================================="
echo "Check BOOT_ADDR in config.h &  macro jump in init.s"
echo "they must point to computed address."

