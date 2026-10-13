#!/usr/bin/env python3
# Copyright 2026 Ângelo Moisés Alves
# Licensed under the MIT license, see the LICENSE file for details.
"""Put a UF2 image on one Pico, and on no other board.

    python3 extras/bench/deploy.py /dev/serial/by-id/usb-Raspberry_Pi_Pico_<id>-if00 build/Sketch.ino.uf2

The uploader of the arduino-pico core restarts the board of the port in BOOTSEL, then writes the image to every
BOOTSEL drive that it finds: another board that is in BOOTSEL at that moment, for whatever reason, gets the image as
well. This script restarts the board of the port, waits for the drive that appears on the same USB port of the PC,
and writes the image there only. It needs Linux (sysfs and /proc/mounts); a drive that the desktop does not mount
is mounted with udisksctl, as the uploader of the core does.
"""
import os
import re
import shutil
import subprocess
import sys
import time

USB_PORT = re.compile(r"^\d+-\d+(\.\d+)*$")  # how the kernel names a USB port: 3-11, 1-1.4...


def usb_port(serial_port, sys_root="/sys"):
    """The USB port of the PC that a serial device is on, or None if it is not on USB."""
    tty = os.path.basename(os.path.realpath(serial_port))
    interface = os.path.realpath(os.path.join(sys_root, "class", "tty", tty, "device"))
    port = os.path.basename(os.path.dirname(interface))
    return port if USB_PORT.match(port) else None


def unescape(field):
    """A field of /proc/mounts: spaces, tabs, newlines and backslashes come as octal escapes."""
    return re.sub(r"\\([0-7]{3})", lambda m: chr(int(m.group(1), 8)), field)


def partitions_on(port, sys_root="/sys"):
    """The block devices of a disk on that USB port, partitions first: sdc1, sdc."""
    found = []
    for block in sorted(os.listdir(os.path.join(sys_root, "block"))):
        parts = os.path.realpath(os.path.join(sys_root, "block", block, "device")).split("/")
        if port in parts:
            subdirs = sorted(d for d in os.listdir(os.path.join(sys_root, "block", block))
                             if d.startswith(block) and d[len(block):].isdigit())
            found += subdirs + [block]
    return found


def bootsel_drive(port, sys_root="/sys", mounts="/proc/mounts"):
    """Where the BOOTSEL drive of the board on that USB port is mounted, or None."""
    devices = ["/dev/" + name for name in partitions_on(port, sys_root)]
    with open(mounts) as table:
        for line in table:
            source, target = (unescape(field) for field in line.split()[:2])
            if source in devices and os.path.exists(os.path.join(target, "INFO_UF2.TXT")):
                return target
    return None


def mount(port, sys_root="/sys"):
    """Ask udisks to mount the drive of the board on that USB port; quietly does nothing if it cannot."""
    for name in partitions_on(port, sys_root):
        try:
            subprocess.run(["udisksctl", "mount", "--no-user-interaction", "--block-device", "/dev/" + name],
                           stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
        except (OSError, subprocess.SubprocessError):
            pass
        return


def deploy(serial_port, uf2, timeout=20):
    if not os.path.exists(serial_port):
        sys.exit("deploy: %s does not exist. A board that is in BOOTSEL already has no serial port that tells which "
                 "one it is: copy %s to its drive yourself" % (serial_port, uf2))
    port = usb_port(serial_port)
    if port is None:
        sys.exit("deploy: %s is not a USB serial device" % serial_port)
    import serial  # pyserial, as the uploader of the core
    reset_error = None
    try:
        link = serial.Serial()
        link.port = os.path.realpath(serial_port)
        link.open()
        link.baudrate = 9600
        link.dtr = True
        time.sleep(0.1)
        link.dtr = False
        link.baudrate = 1200  # the firmware of arduino-pico restarts in BOOTSEL when the port goes to 1200 baud
        link.close()
    except (OSError, serial.SerialException) as e:
        reset_error = e
    start = time.time()
    drive = None
    mounted = False
    while drive is None:
        if time.time() - start > timeout:
            sys.exit("deploy: no BOOTSEL drive appeared on USB port %s within %d s%s" %
                     (port, timeout, "; the reset at 1200 baud failed: %s" % reset_error if reset_error else ""))
        time.sleep(0.5)
        drive = bootsel_drive(port)
        if drive is None and not mounted and time.time() - start > 5:
            mount(port)  # the desktop did not mount it
            mounted = True
    shutil.copyfile(uf2, os.path.join(drive, "NEW.UF2"))
    os.sync()
    print("deploy: %s -> %s, on USB port %s" % (os.path.basename(uf2), drive, port))


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    deploy(sys.argv[1], sys.argv[2])
