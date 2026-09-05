#!/usr/bin/env python3
"""Record the binary raw log of a MainCore built with -DRAW_LOG_OUTPUT.

Usage: record_raw.py PORT SECONDS OUTPUT.bin [--baud 921600]
       record_raw.py --parse FILE.bin      (statistics of an existing recording)

The board is reset by opening the port (DTR), so the recording contains the
whole run including the 15 s stationary calibration. Frame statistics and
timestamp gaps are printed at the end; a gap count of 0 means no sample was
lost on the serial link.

Closing the port while the 921600-baud stream is running tends to leave the
CP210x USB-serial bridge in a state where nothing is received any more (an
empty recording, or the next upload hangs). On Linux the script therefore
resets the bridge at the USB level before and after recording (see
usb_reset_cp210x.py); --no-usb-reset disables that.
"""
import argparse
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import serial

# sizeof() of the structs in shared_types.h (verified with the host compiler)
IMU_SIZE = 32
POS_SIZE = 76
STATS_SIZE = 68
SIZES = {0: IMU_SIZE, 1: POS_SIZE, 2: STATS_SIZE}
TICKS_PER_SAMPLE = 19200000 // 1920
CALIBRATION_SAMPLES = 15 * 1920  # LOCALIZER_CALIBRATION_SECONDS * LOCALIZER_SAMPLE_RATE


def parse_frames(data):
    """Yield (type, payload) for every valid frame in data; count bad bytes."""
    i, bad = 0, 0
    n = len(data)
    while i + 4 <= n:
        if data[i] != 0x53 or data[i + 1] != 0x50 or data[i + 2] not in SIZES:
            i += 1
            bad += 1
            continue
        t = data[i + 2]
        size = SIZES[t]
        if i + 4 + size > n:
            break
        payload = data[i + 3:i + 3 + size]
        chk = t
        for b in payload:
            chk ^= b
        if chk != data[i + 3 + size]:
            i += 1
            bad += 1
            continue
        yield t, payload
        i += 4 + size
    return


def summarize(data, seconds):
    imu = pos = stats = 0
    gaps = 0
    last_ts = None
    first_ts = None
    for t, payload in parse_frames(data):
        if t == 0:
            imu += 1
            ts = struct.unpack_from("<I", payload, 0)[0]
            if first_ts is None:
                first_ts = ts
            if last_ts is not None:
                d = (ts - last_ts) & 0xFFFFFFFF
                if d > TICKS_PER_SAMPLE * 3 // 2:
                    gaps += 1
            last_ts = ts
        elif t == 1:
            pos += 1
        elif t == 2:
            stats += 1
    span = ((last_ts - first_ts) & 0xFFFFFFFF) / 19.2e6 if imu > 1 else 0.0
    rate = imu / span if span > 0 else 0.0
    print(f"imu frames: {imu} over {span:.1f} s ({rate:.1f}/s), position frames: {pos}, stats frames: {stats}")
    print(f"timestamp gaps (> 1.5 samples): {gaps}")
    return imu


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?")
    ap.add_argument("seconds", nargs="?", type=float)
    ap.add_argument("output", nargs="?")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--parse", metavar="FILE", help="only summarize an existing recording")
    ap.add_argument("--no-usb-reset", action="store_true", help="do not reset the USB-serial bridge after recording")
    args = ap.parse_args()

    if args.parse:
        with open(args.parse, "rb") as f:
            data = f.read()
        sys.exit(0 if summarize(data, 0.0) > 0 else 1)
    if not (args.port and args.seconds and args.output):
        ap.error("PORT SECONDS OUTPUT are required unless --parse is used")

    def usb_reset():
        if args.no_usb_reset or not sys.platform.startswith("linux"):
            return
        try:
            import usb_reset_cp210x
            usb_reset_cp210x.main([])
            time.sleep(2.0)  # wait for re-enumeration
        except Exception as e:  # best effort
            print(f"usb reset skipped: {e}", file=sys.stderr)

    usb_reset()  # a bridge left locked by a previous session would record nothing
    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    t0 = time.time()
    chunks = []
    total = 0
    # Live progress: count IMU frames so the operator knows when the 15 s
    # stationary calibration (28800 samples) is over and the sensor may move.
    pending = b""
    imu_seen = 0
    announced = False
    print("recording... keep the sensor still until 'calibration complete'", flush=True)
    while time.time() - t0 < args.seconds:
        d = ser.read(65536)
        if not d:
            continue
        chunks.append(d)
        total += len(d)
        if not announced:
            pending += d
            consumed = 0
            for t, _payload in parse_frames(pending):
                consumed += 4 + SIZES[t]
                if t == 0:
                    imu_seen += 1
            pending = pending[consumed:] if consumed else pending[-64:]
            if imu_seen >= CALIBRATION_SAMPLES:
                print(f"[{time.time() - t0:5.1f} s] calibration complete - you may move the sensor now", flush=True)
                announced = True
            elif imu_seen == 0 and time.time() - t0 > 6.0:
                print("no frames yet after 6 s: is MainCore built with -DRAW_LOG_OUTPUT?", flush=True)
                announced = True  # stop nagging
    ser.close()
    data = b"".join(chunks)
    with open(args.output, "wb") as f:
        f.write(data)
    print(f"wrote {total} bytes to {args.output}")
    ok = summarize(data, args.seconds) > 0
    usb_reset()
    if not ok:
        print("no IMU frames: is MainCore built with -DRAW_LOG_OUTPUT and the baud rate right?", file=sys.stderr)
        sys.exit(1)



if __name__ == "__main__":
    main()
