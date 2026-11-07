#!/usr/bin/env python3
"""Shared input layer for the visualisation tools (imu_monitor.py, imu_viewer.py).

Three sources deliver the same Sample objects:

* a serial port with the normal MainCore build (115200 baud, 18 hex-encoded
  floats per line, 30 Hz; lines starting with '#' are board messages),
* a text file of such serial lines (e.g. saved with a terminal program),
  replayed at the pace of its timestamps,
* a replay CSV written by test/replay --csv, replayed at the pace of its
  t column.

Opening the serial port resets the board (DTR), so the first 15 s are the
stationary calibration during which nothing is received.
"""
import collections
import math
import struct
import threading
import time

TIMESTAMP_HZ = 19.2e6  # IMU timestamp counter
SERIAL_FIELDS = 18

Sample = collections.namedtuple(
    "Sample",
    ["t", "temp", "gyro", "accel", "quat", "vel", "pos", "stationary"])
# t: seconds (unwrapped), gyro: rad/s sensor frame (offset corrected),
# accel: m/s^2 earth frame (NWU, gravity removed), quat: (w, x, y, z) sensor
# relative to earth, vel/pos: m/s, m earth frame, stationary: bool or None
# (the serial line carries no flag; the integrator resets the velocity to
# exactly zero while stationary, which the tools use as the indication).


def hex_float(s):
    return struct.unpack("<f", struct.pack("<I", int(s, 16)))[0]


def float_hex(x):
    return "%08x" % struct.unpack("<I", struct.pack("<f", x))[0]


class TimestampUnwrapper:
    """32-bit IMU counter ticks -> continuous seconds."""

    def __init__(self):
        self.last = None
        self.offset = 0

    def seconds(self, ticks):
        if self.last is not None and ticks < self.last and self.last - ticks > 0x80000000:
            self.offset += 1 << 32
        self.last = ticks
        return (ticks + self.offset) / TIMESTAMP_HZ


def parse_serial_line(line, unwrapper):
    """Decode one line of the MainCore output; None if it is not a sample."""
    parts = line.strip().split(",")
    if len(parts) != SERIAL_FIELDS:
        return None
    try:
        ticks = int(parts[0], 16)
        v = [hex_float(p) for p in parts[1:]]
    except ValueError:
        return None
    # timestamp, temp, gx, gy, gz, ax, ay, az, qw, qx, qy, qz, vx, vy, vz, px, py, pz
    vel = (v[11], v[12], v[13])
    return Sample(
        t=unwrapper.seconds(ticks),
        temp=v[0],
        gyro=(v[1], v[2], v[3]),
        accel=(v[4], v[5], v[6]),
        quat=(v[7], v[8], v[9], v[10]),
        vel=vel,
        pos=(v[14], v[15], v[16]),
        stationary=(vel == (0.0, 0.0, 0.0)),
    )


def format_serial_line(sample, ticks):
    """Inverse of parse_serial_line (used by the tests and to convert CSVs)."""
    fields = [sample.temp, *sample.gyro, *sample.accel, *sample.quat, *sample.vel, *sample.pos]
    return "%08x," % (ticks & 0xFFFFFFFF) + ",".join(float_hex(x) for x in fields)


def parse_csv_row(row):
    """Decode one row of a replay CSV (dict from csv.DictReader)."""
    f = {k: float(v) for k, v in row.items()}
    return Sample(
        t=f["t"],
        temp=float("nan"),
        gyro=(f["gx"], f["gy"], f["gz"]),
        accel=(f["ax"], f["ay"], f["az"]),
        quat=(f["qw"], f["qx"], f["qy"], f["qz"]),
        vel=(f["vx"], f["vy"], f["vz"]),
        pos=(f["px"], f["py"], f["pz"]),
        stationary=f["stationary"] != 0.0,
    )


# ---- maths without numpy (the CLI tool has no numpy dependency) -------------

def quat_to_euler_deg(q):
    """Roll, pitch, yaw (deg) of a sensor-to-earth quaternion (w, x, y, z), ZYX."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = 2 * (w * y - z * x)
    s = max(-1.0, min(1.0, s))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def quat_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_rotate(q, v):
    """Rotate vector v by quaternion q (sensor -> earth for the output quaternion)."""
    p = (0.0, v[0], v[1], v[2])
    r = quat_mul(quat_mul(q, p), quat_conj(q))
    return (r[1], r[2], r[3])


def relative_pose(origin_q, origin_p, q, p):
    """Position and attitude of (q, p) expressed in the frame (origin_q, origin_p)."""
    d = (p[0] - origin_p[0], p[1] - origin_p[1], p[2] - origin_p[2])
    inv = quat_conj(origin_q)
    return quat_rotate(inv, d), quat_mul(inv, q)


# ---- sources ---------------------------------------------------------------

class Stats:
    def __init__(self):
        self.samples = 0
        self.bad_lines = 0
        self.last_wall = None
        self.rate = 0.0
        self._window = collections.deque(maxlen=60)

    def tick(self):
        now = time.monotonic()
        self.samples += 1
        self._window.append(now)
        if len(self._window) >= 2:
            span = self._window[-1] - self._window[0]
            self.rate = (len(self._window) - 1) / span if span > 0 else 0.0
        self.last_wall = now


class SerialSource:
    """Reads the serial port in a thread. poll() returns (samples, messages)."""

    def __init__(self, port, baud=115200):
        import serial  # pyserial
        self.name = port
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.unwrapper = TimestampUnwrapper()
        self.stats = Stats()
        self._lock = threading.Lock()
        self._samples = []
        self._messages = []
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        pending = b""
        while not self._stop:
            try:
                data = self.ser.read(4096)
            except Exception as e:  # port gone
                with self._lock:
                    self._messages.append("# serial error: %s" % e)
                return
            if not data:
                continue
            pending += data
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                text = line.decode(errors="replace").strip()
                if not text:
                    continue
                if text.startswith("#"):
                    with self._lock:
                        self._messages.append(text)
                    continue
                s = parse_serial_line(text, self.unwrapper)
                with self._lock:
                    if s is None:
                        self.stats.bad_lines += 1
                    else:
                        self.stats.tick()
                        self._samples.append(s)

    def poll(self):
        with self._lock:
            samples, self._samples = self._samples, []
            messages, self._messages = self._messages, []
        return samples, messages

    def write(self, text):
        self.ser.write(text.encode())

    def close(self):
        self._stop = True
        self._thread.join(timeout=1.0)
        self.ser.close()


class ReplaySource:
    """Replays samples from a file at the pace of their timestamps (x speed)."""

    def __init__(self, name, samples, speed=1.0, realtime=True):
        self.name = name
        self.samples_all = samples
        self.speed = speed
        self.realtime = realtime
        self.stats = Stats()
        self.index = 0
        self._t0_wall = time.monotonic()
        self._t0 = samples[0].t if samples else 0.0
        self.finished = not samples

    def poll(self):
        out = []
        if self.index >= len(self.samples_all):
            self.finished = True
            return out, []
        if self.realtime:
            elapsed = (time.monotonic() - self._t0_wall) * self.speed
            while self.index < len(self.samples_all) and self.samples_all[self.index].t - self._t0 <= elapsed:
                out.append(self.samples_all[self.index])
                self.index += 1
        else:
            out = self.samples_all[self.index:]
            self.index = len(self.samples_all)
        for _ in out:
            self.stats.tick()
        return out, []

    def write(self, text):
        pass

    def close(self):
        pass


def load_serial_log(path):
    unwrapper = TimestampUnwrapper()
    samples = []
    with open(path, errors="replace") as f:
        for line in f:
            s = parse_serial_line(line, unwrapper)
            if s is not None:
                samples.append(s)
    return samples


def load_replay_csv(path):
    import csv
    with open(path) as f:
        return [parse_csv_row(r) for r in csv.DictReader(f)]


def add_source_arguments(ap):
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--port", help="serial port of the normal MainCore build (e.g. /dev/ttyUSB0, COM3)")
    g.add_argument("--file", help="text file of saved serial lines, replayed in real time")
    g.add_argument("--csv", help="replay CSV from test/replay --csv, replayed in real time")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed factor for --file/--csv")
    ap.add_argument("--fast", action="store_true", help="replay --file/--csv as fast as possible")


def open_source(args):
    if args.file:
        return ReplaySource(args.file, load_serial_log(args.file), args.speed, not args.fast)
    if args.csv:
        return ReplaySource(args.csv, load_replay_csv(args.csv), args.speed, not args.fast)
    port = args.port or "/dev/ttyUSB0"
    return SerialSource(port, args.baud)
