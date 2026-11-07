#!/usr/bin/env python3
"""Real-time 3D viewer of the localizer pose and the origin frame (tkinter).

Draws, in a software-rendered 3D view (no OpenGL, no Qt):
  * the origin frame (N red, W green, U blue) - the frame the pose is
    expressed in; press z to move it to the current pose, Z to reset it,
  * the current sensor pose as a box with its body axes,
  * the path since start (press c to clear) and a drop line to the ground,
and next to it the numbers: position, velocity, attitude, the pose relative
to the origin frame, and strip charts of acceleration and angular rate.

The per-frame work (projection of the path, rotations) runs in numba-compiled
kernels when numba is installed (pip install numba) and falls back to numpy
otherwise; the panel shows the render time.

Usage: imu_viewer.py [--port /dev/ttyUSB0 | --file LINES.txt | --csv REPLAY.csv]
                     [--size 0.1] [--duration S] [--snapshot VIEW.ps]

Mouse: left drag = orbit, right drag = pan, wheel = zoom.
Keys: r reset the view to show the whole path, f follow the sensor,
      z/Z set/reset origin, c clear path, space pause, q quit.
Opening the serial port resets the board: keep it still for the 15 s
calibration; nothing is shown until the first estimate arrives.
"""
import argparse
import collections
import math
import os
import sys
import time
import tkinter as tk

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import imu_stream  # noqa: E402

try:
    from numba import njit
    HAVE_NUMBA = True
except ImportError:  # pragma: no cover - depends on the environment
    HAVE_NUMBA = False

    def njit(*args, **kwargs):
        if args and callable(args[0]):
            return args[0]
        return lambda f: f

DEG = 180.0 / math.pi
PATH_CAPACITY = 20000
PATH_CHUNK = 1000      # path points per canvas line item: with a still camera only the last chunk is updated
PATH_MIN_STEP = 0.002  # m, points closer than this to the last kept point are dropped
NEAR = 0.05            # m, points closer to the camera than this are not drawn


# ---- kernels ---------------------------------------------------------------

@njit(cache=True, fastmath=True)
def project_points(points, n, cam, right, up, fwd, focal, cx, cy, out):
    """Perspective projection of points[:n] into out[:n] = (sx, sy, depth)."""
    for i in range(n):
        vx = points[i, 0] - cam[0]
        vy = points[i, 1] - cam[1]
        vz = points[i, 2] - cam[2]
        z = vx * fwd[0] + vy * fwd[1] + vz * fwd[2]
        out[i, 2] = z
        if z > NEAR:
            x = vx * right[0] + vy * right[1] + vz * right[2]
            y = vx * up[0] + vy * up[1] + vz * up[2]
            out[i, 0] = cx + focal * x / z
            out[i, 1] = cy - focal * y / z
        else:
            out[i, 0] = 0.0
            out[i, 1] = 0.0


if not HAVE_NUMBA:  # vectorised fallback, same contract
    def project_points(points, n, cam, right, up, fwd, focal, cx, cy, out):  # noqa: F811
        v = points[:n] - cam
        z = v @ fwd
        x = v @ right
        y = v @ up
        ok = z > NEAR
        zs = np.where(ok, z, 1.0)
        out[:n, 0] = np.where(ok, cx + focal * x / zs, 0.0)
        out[:n, 1] = np.where(ok, cy - focal * y / zs, 0.0)
        out[:n, 2] = z


@njit(cache=True)
def compact_polyline(proj, n, min_px, out):
    """Flatten proj[:n] (sx, sy, depth) into out = [x0, y0, x1, y1, ...], skipping
    points behind the camera and points less than min_px away from the last
    kept one (invisible detail that would only cost drawing time). Returns
    the number of values written."""
    k = 0
    last_x = 1e30
    last_y = 1e30
    for i in range(n):
        if proj[i, 2] <= NEAR:
            continue
        x = proj[i, 0]
        y = proj[i, 1]
        if abs(x - last_x) + abs(y - last_y) < min_px and i != n - 1:
            continue
        out[k] = x
        out[k + 1] = y
        k += 2
        last_x = x
        last_y = y
    return k


if not HAVE_NUMBA:  # vectorised approximation: keep every point that moved on screen
    def compact_polyline(proj, n, min_px, out):  # noqa: F811
        ok = proj[:n, 2] > NEAR
        pts = proj[:n, :2][ok]
        if len(pts) == 0:
            return 0
        d = np.abs(np.diff(pts, axis=0)).sum(axis=1)
        keep = np.concatenate(([True], d >= min_px))
        keep[-1] = True
        pts = pts[keep]
        k = 2 * len(pts)
        out[:k] = pts.ravel()
        return k


@njit(cache=True)
def quat_to_matrix(w, x, y, z):
    """Rotation matrix (sensor -> earth) of a unit quaternion."""
    m = np.empty((3, 3))
    m[0, 0] = 1 - 2 * (y * y + z * z)
    m[0, 1] = 2 * (x * y - w * z)
    m[0, 2] = 2 * (x * z + w * y)
    m[1, 0] = 2 * (x * y + w * z)
    m[1, 1] = 1 - 2 * (x * x + z * z)
    m[1, 2] = 2 * (y * z - w * x)
    m[2, 0] = 2 * (x * z - w * y)
    m[2, 1] = 2 * (y * z + w * x)
    m[2, 2] = 1 - 2 * (x * x + y * y)
    return m


@njit(cache=True)
def transform_points(local, rot, pos, out):
    """out = rot @ local + pos for every row."""
    for i in range(local.shape[0]):
        for r in range(3):
            out[i, r] = rot[r, 0] * local[i, 0] + rot[r, 1] * local[i, 1] + rot[r, 2] * local[i, 2] + pos[r]


# ---- geometry --------------------------------------------------------------

def box_vertices(length, width, height):
    hl, hw, hh = length / 2, width / 2, height / 2
    return np.array([[sx * hl, sy * hw, sz * hh] for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)], dtype=np.float64)


BOX_EDGES = [(0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4), (0, 4), (1, 5), (2, 6), (3, 7)]


class Camera:
    def __init__(self):
        self.reset()

    def reset(self):
        self.az = -135.0  # deg, from +N towards +W
        self.el = 30.0
        self.dist = 1.5
        self.target = np.zeros(3)
        self.fov = 50.0

    def frame(self, points):
        """Look at the centre of the points from a distance that shows them all."""
        if len(points) == 0:
            self.target = np.zeros(3)
            self.dist = 1.5
            return
        lo, hi = points.min(axis=0), points.max(axis=0)
        self.target = (lo + hi) / 2
        self.dist = max(0.5, 2.5 * float(np.linalg.norm(hi - lo)))

    def basis(self):
        az, el = math.radians(self.az), math.radians(self.el)
        offset = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
        cam = self.target + self.dist * offset
        fwd = -offset
        world_up = np.array([0.0, 0.0, 1.0])
        right = np.cross(fwd, world_up)
        n = np.linalg.norm(right)
        right = right / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])
        up = np.cross(right, fwd)
        return cam, right, up, fwd


class Viewer:
    def __init__(self, root, source, size, duration, snapshot):
        self.root = root
        self.source = source
        self.size = size
        self.duration = duration
        self.snapshot = snapshot
        self.t_start = time.monotonic()
        self.camera = Camera()
        self.follow = False
        self.paused = False
        self.drag = None
        self.last = None
        self.origin_q = (1.0, 0.0, 0.0, 0.0)
        self.origin_p = (0.0, 0.0, 0.0)
        self.messages = collections.deque(maxlen=4)
        self.render_ms = 0.0
        self.dirty = True

        self.path = np.zeros((PATH_CAPACITY, 3))
        self.path_n = 0
        self.proj = np.zeros((PATH_CAPACITY, 3))
        self.flat = np.zeros(2 * PATH_CHUNK + 2)
        self.drawn_points = 0
        self.path_ids = []     # canvas line per chunk
        self.chunk_points = [] # drawn points per chunk
        self.drawn_n = 0       # path points whose chunk coordinates are current
        self.view_key = None
        self.static_key = None # view + origin for which the grid and origin frame are current
        self.box_local = box_vertices(size, size * 0.4, size * 0.1)
        self.box_world = np.zeros_like(self.box_local)
        self.axes_local = np.array([[0, 0, 0], [size, 0, 0], [0, size, 0], [0, 0, size]], dtype=np.float64)
        self.axes_world = np.zeros_like(self.axes_local)
        self.scratch = np.zeros((64, 3))

        root.title("IMU localizer viewer")
        root.configure(bg="#202020")
        self.canvas = tk.Canvas(root, width=900, height=650, bg="#181818", highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        panel = tk.Frame(root, bg="#202020")
        panel.pack(side=tk.RIGHT, fill=tk.Y)
        self.text = tk.Label(panel, font=("Courier", 10), justify=tk.LEFT, anchor="nw", bg="#202020", fg="#e0e0e0", width=46)
        self.text.pack(fill=tk.X, padx=6, pady=6)
        self.chart_a = StripChart(panel, "acceleration NWU (m/s^2)", ("#ff6060", "#60ff60", "#6090ff"))
        self.chart_g = StripChart(panel, "angular rate xyz (deg/s)", ("#ff6060", "#60ff60", "#6090ff"))
        help_text = ("mouse: L-drag orbit, R-drag pan, wheel zoom\n"
                     "keys: r view  f follow  z/Z origin  c clear  space pause  q quit")
        tk.Label(panel, text=help_text, font=("Courier", 9), justify=tk.LEFT, bg="#202020", fg="#909090").pack(padx=6, pady=6, anchor="w")

        self.grid_ids = []
        self.grid_extent = 0
        self.drop_id = self.canvas.create_line(0, 0, 0, 0, fill="#606060", dash=(3, 3))
        self.origin_ids = [self.canvas.create_line(0, 0, 0, 0, fill=c, width=3) for c in ("#ff4040", "#40ff40", "#4080ff")]
        self.origin_labels = [self.canvas.create_text(0, 0, text=t, fill=c, font=("Helvetica", 11, "bold"))
                              for t, c in (("N", "#ff4040"), ("W", "#40ff40"), ("U", "#4080ff"))]
        self.body_ids = [self.canvas.create_line(0, 0, 0, 0, fill=c, width=2) for c in ("#ff9090", "#90ff90", "#90b0ff")]
        self.box_ids = [self.canvas.create_line(0, 0, 0, 0, fill="#f0f0f0", width=2) for _ in BOX_EDGES]
        self.status_id = self.canvas.create_text(8, 8, anchor="nw", fill="#c0c0c0", font=("Courier", 10), text="")

        self.canvas.bind("<Configure>", lambda e: self.mark_dirty())
        self.canvas.bind("<ButtonPress-1>", lambda e: self.start_drag(e, "orbit"))
        self.canvas.bind("<ButtonPress-3>", lambda e: self.start_drag(e, "pan"))
        self.canvas.bind("<B1-Motion>", self.do_drag)
        self.canvas.bind("<B3-Motion>", self.do_drag)
        self.canvas.bind("<MouseWheel>", lambda e: self.zoom(-e.delta / 120.0))
        self.canvas.bind("<Button-4>", lambda e: self.zoom(-1))
        self.canvas.bind("<Button-5>", lambda e: self.zoom(1))
        root.bind("<Key>", self.on_key)
        self.warm_up()
        root.after(10, self.tick)

    # ---- interaction ----
    def mark_dirty(self):
        self.dirty = True

    def start_drag(self, event, mode):
        self.drag = (mode, event.x, event.y)

    def do_drag(self, event):
        if not self.drag:
            return
        mode, x0, y0 = self.drag
        dx, dy = event.x - x0, event.y - y0
        self.drag = (mode, event.x, event.y)
        if mode == "orbit":
            self.camera.az -= dx * 0.4
            self.camera.el = max(-89.0, min(89.0, self.camera.el + dy * 0.4))
        else:
            cam, right, up, fwd = self.camera.basis()
            scale = self.camera.dist / max(1, self.canvas.winfo_height())
            self.camera.target = self.camera.target - right * dx * scale + up * dy * scale
            self.follow = False
        self.mark_dirty()

    def zoom(self, steps):
        self.camera.dist = max(0.2, min(50.0, self.camera.dist * (1.15 ** steps)))
        self.mark_dirty()

    def on_key(self, event):
        k = event.keysym
        if k == "q" or k == "Escape":
            self.finish()
        elif k == "r":
            self.camera.reset()
            self.camera.frame(self.path[:self.path_n])
            self.follow = False
        elif k == "f":
            self.follow = not self.follow
        elif k == "z" and self.last is not None:
            self.origin_q, self.origin_p = self.last.quat, self.last.pos
        elif k == "Z":
            self.origin_q, self.origin_p = (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        elif k == "c":
            self.path_n = 0
            self.drawn_n = 0
            if self.last is not None:
                self.append_path(self.last.pos)
        elif k == "space":
            self.paused = not self.paused
        self.mark_dirty()

    # ---- data ----
    def warm_up(self):
        """Compile the kernels before the first frame (numba) and check them."""
        out = np.zeros((2, 3))
        project_points(np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 0.0]]), 2, np.array([-1.0, 0.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0]), 1.0, 0.0, 0.0, out)
        transform_points(self.axes_local, quat_to_matrix(1.0, 0.0, 0.0, 0.0), np.zeros(3), self.axes_world)

    def append_path(self, p):
        if self.path_n > 0:
            d = self.path[self.path_n - 1] - p
            if d @ d < PATH_MIN_STEP * PATH_MIN_STEP:
                return
        if self.path_n >= PATH_CAPACITY:
            self.path[:-1] = self.path[1:]
            self.path_n = PATH_CAPACITY - 1
            self.drawn_n = 0  # every chunk changed
        self.path[self.path_n] = p
        self.path_n += 1

    def ingest(self, samples, messages):
        for m in messages:
            self.messages.append(m)
        if self.paused or not samples:
            return
        for s in samples:
            self.append_path(s.pos)
            self.chart_a.push(s.t, s.accel)
            self.chart_g.push(s.t, (s.gyro[0] * DEG, s.gyro[1] * DEG, s.gyro[2] * DEG))
        self.last = samples[-1]
        self.dirty = True

    # ---- rendering ----
    def tick(self):
        samples, messages = self.source.poll()
        self.ingest(samples, messages)
        if self.dirty:
            t0 = time.perf_counter()
            self.draw()
            self.render_ms = (time.perf_counter() - t0) * 1000.0
            self.dirty = False
        if self.duration and time.monotonic() - self.t_start >= self.duration:
            self.finish()
            return
        self.root.after(33, self.tick)

    def finish(self):
        if self.snapshot:
            self.canvas.postscript(file=self.snapshot, colormode="color")
        self.root.destroy()

    def line(self, item, a, b):
        """Update a canvas line from two projected points; hide it if not in front."""
        if a[2] > NEAR and b[2] > NEAR:
            self.canvas.coords(item, a[0], a[1], b[0], b[1])
            self.canvas.itemconfigure(item, state="normal")
        else:
            self.canvas.itemconfigure(item, state="hidden")

    def grid_extent_for(self, n):
        extent = max(1, int(math.ceil(np.abs(self.path[:n]).max() if n else 1.0)))
        return min(extent, 50)

    def update_grid(self, cam, right, up, fwd, focal, cx, cy):
        extent = self.grid_extent_for(self.path_n)
        if extent != self.grid_extent:
            for i in self.grid_ids:
                self.canvas.delete(i)
            self.grid_ids = []
            step = 0.5 if extent <= 3 else 1.0
            n = int(extent / step)
            self.grid_lines = []
            for k in range(-n, n + 1):
                v = k * step
                self.grid_lines.append(((v, -extent, 0.0), (v, extent, 0.0)))
                self.grid_lines.append(((-extent, v, 0.0), (extent, v, 0.0)))
            self.grid_ids = [self.canvas.create_line(0, 0, 0, 0, fill="#3a3a3a") for _ in self.grid_lines]
            for i in self.grid_ids:
                self.canvas.tag_lower(i)
            self.grid_pts = np.array([p for seg in self.grid_lines for p in seg], dtype=np.float64)
            self.grid_proj = np.zeros_like(self.grid_pts)
            self.grid_extent = extent
        project_points(self.grid_pts, len(self.grid_pts), cam, right, up, fwd, focal, cx, cy, self.grid_proj)
        for i, item in enumerate(self.grid_ids):
            self.line(item, self.grid_proj[2 * i], self.grid_proj[2 * i + 1])

    def draw(self):
        w, h = max(2, self.canvas.winfo_width()), max(2, self.canvas.winfo_height())
        if self.follow and self.last is not None:
            self.camera.target = np.array(self.last.pos)
        cam, right, up, fwd = self.camera.basis()
        focal = 0.5 * h / math.tan(math.radians(self.camera.fov) / 2)
        cx, cy = w / 2.0, h / 2.0
        view_key = (w, h, self.camera.az, self.camera.el, self.camera.dist, tuple(self.camera.target))

        # grid and origin frame: only when the view or the origin changed
        static_key = (view_key, self.origin_q, self.origin_p, self.grid_extent_for(self.path_n))
        if static_key != self.static_key:
            self.static_key = static_key
            self.update_grid(cam, right, up, fwd, focal, cx, cy)
            oq, op = self.origin_q, np.array(self.origin_p)
            transform_points(self.axes_local * 2.0, quat_to_matrix(*oq), op, self.axes_world)
            project_points(self.axes_world, 4, cam, right, up, fwd, focal, cx, cy, self.scratch)
            for k in range(3):
                self.line(self.origin_ids[k], self.scratch[0], self.scratch[k + 1])
                self.canvas.coords(self.origin_labels[k], self.scratch[k + 1][0], self.scratch[k + 1][1] - 8)

        if self.last is not None:
            s = self.last
            rot = quat_to_matrix(*s.quat)
            pos = np.array(s.pos)
            transform_points(self.axes_local, rot, pos, self.axes_world)
            project_points(self.axes_world, 4, cam, right, up, fwd, focal, cx, cy, self.scratch)
            for k in range(3):
                self.line(self.body_ids[k], self.scratch[0], self.scratch[k + 1])
            transform_points(self.box_local, rot, pos, self.box_world)
            project_points(self.box_world, 8, cam, right, up, fwd, focal, cx, cy, self.scratch)
            for item, (a, b) in zip(self.box_ids, BOX_EDGES):
                self.line(item, self.scratch[a], self.scratch[b])
            drop = np.array([s.pos, (s.pos[0], s.pos[1], 0.0)])
            project_points(drop, 2, cam, right, up, fwd, focal, cx, cy, self.scratch)
            self.line(self.drop_id, self.scratch[0], self.scratch[1])

        self.draw_path(cam, right, up, fwd, focal, cx, cy, view_key)

        self.canvas.itemconfigure(self.status_id, text=self.status_text())
        self.text.configure(text=self.panel_text())
        self.chart_a.draw()
        self.chart_g.draw()

    def draw_path(self, cam, right, up, fwd, focal, cx, cy, view_key):
        """The path is split into chunks of PATH_CHUNK points, one canvas line
        each, reduced to the points at least a pixel apart on screen. When the
        view is unchanged only the chunk receiving new points is recomputed."""
        if view_key != self.view_key:
            self.view_key = view_key
            self.drawn_n = 0
        n = self.path_n
        if n < 2:
            for item in self.path_ids:
                self.canvas.itemconfigure(item, state="hidden")
            self.drawn_n = 0
            self.drawn_points = 0
            return
        if n == self.drawn_n:
            return
        first_chunk = max(0, self.drawn_n - 1) // PATH_CHUNK
        start = first_chunk * PATH_CHUNK
        project_points(self.path[start:n], n - start, cam, right, up, fwd, focal, cx, cy, self.proj[start:n])
        chunk = first_chunk
        while chunk * PATH_CHUNK < n:
            a = max(0, chunk * PATH_CHUNK - 1)  # repeat the previous point so the chunks join
            b = min((chunk + 1) * PATH_CHUNK, n)
            while len(self.path_ids) <= chunk:
                self.path_ids.append(self.canvas.create_line(0, 0, 0, 0, fill="#ffb030", width=2))
                self.chunk_points.append(0)
            k = compact_polyline(self.proj[a:b], b - a, 1.0, self.flat)
            item = self.path_ids[chunk]
            if k >= 4:
                self.canvas.coords(item, *self.flat[:k].tolist())
                self.canvas.itemconfigure(item, state="normal")
            else:
                self.canvas.itemconfigure(item, state="hidden")
            self.chunk_points[chunk] = k // 2
            chunk += 1
        for item in self.path_ids[chunk:]:
            self.canvas.itemconfigure(item, state="hidden")
        self.drawn_n = n
        self.drawn_points = sum(self.chunk_points[:chunk])

    def status_text(self):
        parts = ["%s" % self.source.name]
        if self.last is None:
            parts.append("waiting for the first estimate (board calibrating, keep it still)")
        else:
            parts.append("%.1f Hz" % self.source.stats.rate)
        if self.paused:
            parts.append("PAUSED")
        if self.follow:
            parts.append("follow")
        if getattr(self.source, "finished", False):
            parts.append("replay finished")
        parts.append("render %.1f ms (%s)" % (self.render_ms, "numba" if HAVE_NUMBA else "numpy"))
        return "   ".join(parts)

    def panel_text(self):
        if self.last is None:
            lines = ["no estimate yet"]
            lines += [m[:46] for m in self.messages]
            return "\n".join(lines)
        s = self.last
        rpy = imu_stream.quat_to_euler_deg(s.quat)
        rel_p, rel_q = imu_stream.relative_pose(self.origin_q, self.origin_p, s.quat, s.pos)
        rel_rpy = imu_stream.quat_to_euler_deg(rel_q)
        speed = math.sqrt(sum(v * v for v in s.vel))
        temp = "%.1f C" % s.temp if not math.isnan(s.temp) else "n/a"
        lines = [
            "t %9.3f s   temp %s" % (s.t, temp),
            "ZUPT %s   samples %d   bad %d" % ("YES" if s.stationary else "no", self.source.stats.samples, self.source.stats.bad_lines),
            "",
            "position NWU [m]    %+8.3f %+8.3f %+8.3f" % s.pos,
            "velocity NWU [m/s]  %+8.3f %+8.3f %+8.3f  |v| %.3f" % (*s.vel, speed),
            "attitude RPY [deg]  %+8.2f %+8.2f %+8.2f" % rpy,
            "rate xyz [deg/s]    %+8.2f %+8.2f %+8.2f" % tuple(g * DEG for g in s.gyro),
            "accel NWU [m/s^2]   %+8.3f %+8.3f %+8.3f" % s.accel,
            "",
            "relative to origin frame (z: set here, Z: reset)",
            "  position [m]      %+8.3f %+8.3f %+8.3f" % tuple(rel_p),
            "  distance [m]      %8.3f" % math.sqrt(sum(x * x for x in rel_p)),
            "  attitude RPY [deg]%+8.2f %+8.2f %+8.2f" % rel_rpy,
            "  origin at [m]     %+8.3f %+8.3f %+8.3f" % self.origin_p,
            "",
            "path points %d (drawn %d)" % (self.path_n, self.drawn_points),
        ]
        lines += [m[:46] for m in self.messages]
        return "\n".join(lines)


class StripChart:
    """Three signals over the last `window` seconds, kept in a numpy ring buffer."""

    def __init__(self, parent, title, colors, width=360, height=110, window=10.0, capacity=600):
        self.window = window
        self.width, self.height = width, height
        self.buf = np.zeros((capacity, 4))
        self.n = 0
        tk.Label(parent, text=title, font=("Courier", 9), bg="#202020", fg="#c0c0c0", anchor="w").pack(fill=tk.X, padx=6)
        self.canvas = tk.Canvas(parent, width=width, height=height, bg="#181818", highlightthickness=0)
        self.canvas.pack(padx=6, pady=(0, 6))
        self.zero = self.canvas.create_line(0, height / 2, width, height / 2, fill="#404040")
        self.lines = [self.canvas.create_line(0, 0, 0, 0, fill=c) for c in colors]
        self.label = self.canvas.create_text(4, 2, anchor="nw", fill="#909090", font=("Courier", 8), text="")

    def push(self, t, xyz):
        if self.n >= len(self.buf):
            half = len(self.buf) // 2
            self.buf[:half] = self.buf[-half:]
            self.n = half
        self.buf[self.n, 0] = t
        self.buf[self.n, 1:] = xyz
        self.n += 1

    def draw(self):
        if self.n < 2:
            return
        t1 = self.buf[self.n - 1, 0]
        first = int(np.searchsorted(self.buf[:self.n, 0], t1 - self.window))
        arr = self.buf[first:self.n]
        t = arr[:, 0]
        limit = max(1e-3, float(np.abs(arr[:, 1:]).max()))
        x = (t - (t1 - self.window)) / self.window * self.width
        for k, item in enumerate(self.lines):
            y = self.height / 2 - arr[:, k + 1] / limit * (self.height / 2 - 4)
            coords = np.empty(2 * len(x))
            coords[0::2] = x
            coords[1::2] = y
            self.canvas.coords(item, *coords.tolist())
        self.canvas.itemconfigure(self.label, text="±%.3g" % limit)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    imu_stream.add_source_arguments(ap)
    ap.add_argument("--size", type=float, default=0.1, help="length of the drawn sensor box and axes (m)")
    ap.add_argument("--duration", type=float, default=0.0, help="close after this many seconds (0 = until q)")
    ap.add_argument("--snapshot", help="save the 3D view as PostScript when closing")
    args = ap.parse_args()
    source = imu_stream.open_source(args)
    root = tk.Tk()
    Viewer(root, source, args.size, args.duration, args.snapshot)
    try:
        root.mainloop()
    finally:
        source.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
