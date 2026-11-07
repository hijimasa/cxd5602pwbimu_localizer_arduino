#!/usr/bin/env python3
"""Terminal dashboard for the localizer output (no GUI, no numpy).

Shows attitude, rates, acceleration, velocity and position with bars, a
sparkline of the last seconds and an ASCII top view of the path, refreshed in
place with ANSI escape codes (Linux terminals, Windows Terminal / PowerShell).

Usage: imu_monitor.py [--port /dev/ttyUSB0 | --file LINES.txt | --csv REPLAY.csv]
                      [--refresh 10] [--duration S] [--dump]

--dump prints one decoded CSV line per sample instead of the dashboard
(pipe it into a file to log the output in readable form).
Opening the serial port resets the board: keep it still for the 15 s
calibration; nothing is shown until the first estimate arrives.
"""
import argparse
import collections
import math
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import imu_stream  # noqa: E402

DEG = 180.0 / math.pi
SPARK = "▁▂▃▄▅▆▇█"


def enable_ansi():
    if os.name == "nt":
        try:
            import ctypes
            k = ctypes.windll.kernel32
            h = k.GetStdHandle(-11)
            mode = ctypes.c_uint32()
            if k.GetConsoleMode(h, ctypes.byref(mode)):
                k.SetConsoleMode(h, mode.value | 0x0004)  # ENABLE_VIRTUAL_TERMINAL_PROCESSING
        except Exception:
            pass
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


def signed_bar(value, limit, width=21):
    """Centered bar: negative values grow left, positive right."""
    half = width // 2
    n = int(round(max(-1.0, min(1.0, value / limit)) * half))
    cells = [" "] * width
    cells[half] = "|"
    if n > 0:
        for i in range(1, n + 1):
            cells[half + i] = "="
    elif n < 0:
        for i in range(1, -n + 1):
            cells[half - i] = "="
    return "[" + "".join(cells) + "]"


def sparkline(values, width):
    """One character per bin (max of the bin), the whole history fitted into width."""
    values = list(values)
    if not values:
        return ""
    if len(values) > width:
        per = len(values) / float(width)
        values = [max(values[int(i * per):max(int(i * per) + 1, int((i + 1) * per))]) for i in range(width)]
    top = max(values)
    if top <= 0:
        return SPARK[0] * len(values)
    return "".join(SPARK[min(7, int(v / top * 7.999))] for v in values)


def top_view(points, current, width, height):
    """ASCII map of the path in the horizontal plane: north up, west left (NWU)."""
    if width < 10 or height < 5:
        return []
    xs = [p[0] for p in points] + [0.0]
    ys = [p[1] for p in points] + [0.0]
    span = max(max(xs) - min(xs), max(ys) - min(ys), 0.2)
    cx = (max(xs) + min(xs)) / 2
    cy = (max(ys) + min(ys)) / 2
    scale = span / (min(width / 2.0, height) * 0.9)  # m per cell (cells are ~2:1)
    grid = [[" "] * width for _ in range(height)]

    def cell(p):
        col = int(round(width / 2 - (p[1] - cy) / scale * 2))
        row = int(round(height / 2 - (p[0] - cx) / scale))
        return row, col

    def put(p, ch):
        r, c = cell(p)
        if 0 <= r < height and 0 <= c < width:
            grid[r][c] = ch

    for p in points:
        put(p, ".")
    put((0.0, 0.0), "o")
    if current is not None:
        put(current, "@")
    lines = ["".join(r) for r in grid]
    lines[0] = ("N up, W left, %.2f m/cell" % scale)[:width].ljust(width)
    return lines


class Dashboard:
    def __init__(self, source, refresh_hz):
        self.source = source
        self.period = 1.0 / refresh_hz
        self.last = None
        self.first_wall = None
        self.messages = collections.deque(maxlen=3)
        self.accel_hist = collections.deque(maxlen=600)
        self.gyro_hist = collections.deque(maxlen=600)
        self.path = collections.deque(maxlen=4000)
        self.stationary_samples = 0
        self.total = 0

    def ingest(self, samples, messages):
        for m in messages:
            self.messages.append(m)
        for s in samples:
            self.last = s
            self.total += 1
            if s.stationary:
                self.stationary_samples += 1
            self.accel_hist.append(math.sqrt(sum(a * a for a in s.accel)))
            self.gyro_hist.append(math.sqrt(sum(g * g for g in s.gyro)) * DEG)
            if not self.path or sum((a - b) ** 2 for a, b in zip(self.path[-1], s.pos)) > 1e-6:
                self.path.append(s.pos)

    def render(self):
        cols, rows = shutil.get_terminal_size((100, 30))
        out = []
        src = self.source
        status = "WAITING for the first estimate (board calibrating, keep it still)"
        if self.last is not None:
            status = "RUNNING %5.1f Hz" % src.stats.rate
            if getattr(src, "finished", False):
                status = "REPLAY FINISHED"
        out.append(("IMU monitor  %s  %s  samples %d  bad lines %d" % (src.name, status, src.stats.samples, src.stats.bad_lines))[:cols])
        if self.last is None:
            out.append("")
            out.extend(("  " + m)[:cols] for m in self.messages)
            return out
        s = self.last
        roll, pitch, yaw = imu_stream.quat_to_euler_deg(s.quat)
        speed = math.sqrt(sum(v * v for v in s.vel))
        dist = math.sqrt(sum(p * p for p in s.pos))
        temp = "%.1f C" % s.temp if not math.isnan(s.temp) else "n/a"
        out.append("t %9.3f s   temp %s   ZUPT %s (%.0f %% of samples)" % (
            s.t, temp, "YES" if s.stationary else "no ", 100.0 * self.stationary_samples / max(1, self.total)))
        out.append("")
        out.append("attitude  roll  %+8.2f deg %s  pitch %+8.2f deg %s  yaw %+8.2f deg %s" % (
            roll, signed_bar(roll, 180), pitch, signed_bar(pitch, 90), yaw, signed_bar(yaw, 180)))
        g = [x * DEG for x in s.gyro]
        out.append("rate      x %+9.3f d/s %s  y %+9.3f d/s %s  z %+9.3f d/s %s" % (
            g[0], signed_bar(g[0], 200), g[1], signed_bar(g[1], 200), g[2], signed_bar(g[2], 200)))
        a = s.accel
        out.append("accel NWU N %+9.3f m/s2%s  W %+9.3f m/s2%s  U %+9.3f m/s2%s" % (
            a[0], signed_bar(a[0], 5), a[1], signed_bar(a[1], 5), a[2], signed_bar(a[2], 5)))
        v = s.vel
        out.append("velocity  N %+9.3f m/s %s  W %+9.3f m/s %s  U %+9.3f m/s %s  |v| %.3f" % (
            v[0], signed_bar(v[0], 1), v[1], signed_bar(v[1], 1), v[2], signed_bar(v[2], 1), speed))
        p = s.pos
        out.append("position  N %+9.3f m   %s  W %+9.3f m   %s  U %+9.3f m   %s  |p| %.3f" % (
            p[0], signed_bar(p[0], 2), p[1], signed_bar(p[1], 2), p[2], signed_bar(p[2], 2), dist))
        out.append("")
        width = max(10, cols - 26)
        out.append("|a| last %3.0f s  %s %.2f m/s2" % (len(self.accel_hist) / 30.0, sparkline(self.accel_hist, width), self.accel_hist[-1]))
        out.append("|w| last %3.0f s  %s %.1f deg/s" % (len(self.gyro_hist) / 30.0, sparkline(self.gyro_hist, width), self.gyro_hist[-1]))
        out.append("")
        map_rows = rows - len(out) - len(self.messages) - 2
        if map_rows >= 6:
            out.extend(top_view(self.path, s.pos, min(cols, 80), map_rows))
        out.extend(("  " + m)[:cols] for m in self.messages)
        return out

    def run(self, duration):
        enable_ansi()
        sys.stdout.write("\x1b[2J")
        t_end = time.monotonic() + duration if duration else None
        try:
            while True:
                samples, messages = self.source.poll()
                self.ingest(samples, messages)
                lines = self.render()
                sys.stdout.write("\x1b[H" + "\n".join(l + "\x1b[K" for l in lines) + "\x1b[J")
                sys.stdout.flush()
                if t_end and time.monotonic() >= t_end:
                    break
                if getattr(self.source, "finished", False) and not samples:
                    time.sleep(0.5)
                    break
                time.sleep(self.period)
        except KeyboardInterrupt:
            pass
        finally:
            sys.stdout.write("\n")


def dump(source, duration):
    print("t,temp,gx,gy,gz,ax,ay,az,qw,qx,qy,qz,vx,vy,vz,px,py,pz,stationary,roll,pitch,yaw")
    t_end = time.monotonic() + duration if duration else None
    try:
        while True:
            samples, messages = source.poll()
            for m in messages:
                print(m, file=sys.stderr)
            for s in samples:
                rpy = imu_stream.quat_to_euler_deg(s.quat)
                print("%.5f,%.2f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%.2f,%.2f,%.2f" % (
                    s.t, s.temp, *s.gyro, *s.accel, *s.quat, *s.vel, *s.pos, 1 if s.stationary else 0, *rpy))
            sys.stdout.flush()
            if t_end and time.monotonic() >= t_end:
                break
            if getattr(source, "finished", False) and not samples:
                break
            time.sleep(0.02)
    except (KeyboardInterrupt, BrokenPipeError):
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    imu_stream.add_source_arguments(ap)
    ap.add_argument("--refresh", type=float, default=10.0, help="dashboard refresh rate (Hz)")
    ap.add_argument("--duration", type=float, default=0.0, help="exit after this many seconds (0 = until Ctrl-C)")
    ap.add_argument("--dump", action="store_true", help="print decoded samples as CSV instead of the dashboard")
    args = ap.parse_args()
    if os.name != "nt":
        import signal
        signal.signal(signal.SIGPIPE, signal.SIG_DFL)  # quiet exit when piped into head etc.
    source = imu_stream.open_source(args)
    try:
        if args.dump:
            dump(source, args.duration)
        else:
            Dashboard(source, args.refresh).run(args.duration)
    finally:
        source.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
