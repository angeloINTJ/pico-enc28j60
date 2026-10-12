#!/bin/bash
# Copyright 2026 Ângelo Moisés Alves
# Licensed under the MIT license, see the LICENSE file for details.
#
# Regenerate src/enc28j60-spi.pio.h from src/enc28j60-spi.pio. The Arduino IDE
# cannot run pioasm, so the result is committed; the CI checks that the program
# in it is the one that pioasm makes of the .pio file.
#
# Uses the pioasm that comes with the arduino-pico core (set PIOASM to use another
# one, ARDUINO_DATA if the Arduino data folder is not ~/.arduino15).
set -euo pipefail
cd "$(dirname "$0")/../.."

pioasm="${PIOASM:-}"
if [ -z "$pioasm" ]; then
    pioasm=$(find "${ARDUINO_DATA:-$HOME/.arduino15}/packages/rp2040/tools/pqt-pioasm" \
             -name pioasm -type f 2>/dev/null | sort | tail -n 1)
fi
if [ -z "$pioasm" ]; then
    echo "pioasm not found: install the rp2040 core (arduino-cli core install rp2040:rp2040)," >&2
    echo "or set PIOASM to the path of one." >&2
    exit 1
fi

"$pioasm" -o c-sdk src/enc28j60-spi.pio src/enc28j60-spi.pio.h
echo "src/enc28j60-spi.pio.h written by $pioasm"
