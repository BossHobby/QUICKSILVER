#!/bin/bash

set -x

PROGRAMMER=interface/stlink-v2.cfg
TARGET=target/stm32f0x.cfg


ELF=`ls *.elf | head -1`
echo "ELF=$ELF"

if [ "$1" != "--no-backup" ]; then
    NOW=`date +"%F_%T"`
    DIR="backup/$NOW"
    echo "creating buckup: $DIR"
    mkdir -p $DIR
    cd $DIR
    openocd -f $PROGRAMMER  -f $TARGET -c "init; dump_image rom.bin 0x08000000 0x4000; shutdown " || exit_backup
    cd -
fi

if [ "$1" == "rom.bin" ]; then
    openocd -f $PROGRAMMER  -f $TARGET -c "program rom.bin 0x08000000 verify reset; shutdown"
else
    openocd -f $PROGRAMMER  -f $TARGET -c "program $ELF verify reset; shutdown"
fi