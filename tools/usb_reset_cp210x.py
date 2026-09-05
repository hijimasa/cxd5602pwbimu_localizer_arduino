#!/usr/bin/env python3
"""Reset the USB-serial bridge (CP210x) of the Spresense board.

A high-rate binary stream (RAW_LOG_OUTPUT) read at the wrong baud rate can
leave the CP210x bridge in a state where nothing is received any more and
the flash writer hangs waiting for the boot prompt. A USB port reset
(re-enumeration) recovers it without unplugging. The board itself keeps
running; use it before uploading if the upload hangs.

Usage: usb_reset_cp210x.py [VID:PID]   (default 10c4:ea60)
"""
import fcntl
import glob
import os
import sys

USBDEVFS_RESET = 0x5514


def find_device(vid, pid):
    for dev in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(os.path.join(dev, "idVendor")) as f:
                v = f.read().strip()
            with open(os.path.join(dev, "idProduct")) as f:
                p = f.read().strip()
        except OSError:
            continue
        if v == vid and p == pid:
            with open(os.path.join(dev, "busnum")) as f:
                bus = int(f.read())
            with open(os.path.join(dev, "devnum")) as f:
                num = int(f.read())
            return f"/dev/bus/usb/{bus:03d}/{num:03d}"
    return None


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    vid, pid = (argv[0] if argv else "10c4:ea60").split(":")
    node = find_device(vid.lower(), pid.lower())
    if node is None:
        print(f"no USB device {vid}:{pid} found", file=sys.stderr)
        sys.exit(1)
    fd = os.open(node, os.O_WRONLY)
    try:
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
    finally:
        os.close(fd)
    print(f"reset {node}")


if __name__ == "__main__":
    main()
