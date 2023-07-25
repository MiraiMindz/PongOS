#!/usr/bin/env sh

if [ -e "$(command -v hexdump )" ]; then
    FIRST_ARG="$1"
    FILE_NAME="${FIRST_ARG##*/}"
    clear
    printf "\n\t\t\tHEX DUMP PREVIEW OF %s\n\n" "$FILE_NAME"
    hexdump -vC "$FIRST_ARG"
else
    printf "Command HexDump not found.\n"
fi

