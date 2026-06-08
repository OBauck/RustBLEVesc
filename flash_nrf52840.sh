#!/bin/bash

# --- Configuration ---
# Target device name for J-Link
DEVICE="nrf52840_xxAA"
# J-Link interface (swd or jtag)
INTERFACE="swd"
# J-Link interface speed in kHz
SPEED="8000"

JLINK_SCRIPT="flash.jlink"

# Check if an ELF file path was provided as an argument.
if [ "$#" -ne 1 ]; then
    echo "Error: You must provide the path to the firmware file."
    echo "Usage: $0 /path/to/your/firmware"
    exit 1
fi

ELF_FILE=$1

# Check if the file we intend to flash actually exists.
if [ ! -f "$ELF_FILE" ]; then
    echo "Error: Firmware file not found at '$ELF_FILE'"
    exit 1
fi

TEMP_ELF_FILE="$(basename "$ELF_FILE").elf"
cp "$ELF_FILE" "$TEMP_ELF_FILE"

# Create temporary jlink script file which flashes the elf file to the device
cat > $JLINK_SCRIPT << EOF
r
h
loadfile "$TEMP_ELF_FILE"
r
g
q
EOF

echo "Flashing $ELF_FILE.elf using JLinkExe"
# Connect jlink and flash device
# Suppress stdout by directing output to /dev/null
JLinkExe -device $DEVICE -if $INTERFACE -speed $SPEED -autoconnect 1 -commandfile $JLINK_SCRIPT > /dev/null

# Remove temporary files
rm "$JLINK_SCRIPT"
rm "$TEMP_ELF_FILE"

echo "Attaching RTT (probe-rs)"
# Attach rtt (same as probe-rs run will do) 
probe-rs attach --chip nRF52840_xxAA --log-format '{t} {L} {s}' "$ELF_FILE"
