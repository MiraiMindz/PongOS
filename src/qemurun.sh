#!/usr/bin/env sh

if [ -e "$(command -v qemu-system-i386)" ]; then
    clear
    qemu-system-i386 -drive file="$(pwd)/src/bootloader/step0.bin",format=raw,index=0,media=disk
else
    printf "QEMU Emulator not found.\n"
fi

