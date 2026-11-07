#!/usr/bin/env python3
"""Unit tests of the visualisation tools (python3 -m unittest tools/test_imu_tools.py)."""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import imu_stream  # noqa: E402
import imu_monitor  # noqa: E402
import imu_viewer  # noqa: E402


def quat_from_axis_angle(axis, angle):
    s = math.sin(angle / 2)
    n = math.sqrt(sum(a * a for a in axis))
    return (math.cos(angle / 2), axis[0] / n * s, axis[1] / n * s, axis[2] / n * s)


class StreamTests(unittest.TestCase):
    def test_serial_line_round_trip(self):
        s = imu_stream.Sample(t=0.0, temp=36.7, gyro=(0.001, -0.002, 0.003), accel=(0.1, -0.2, 0.3),
                              quat=(0.9, 0.1, -0.2, 0.3), vel=(0.0, 0.0, 0.0), pos=(1.5, -2.5, 0.25), stationary=True)
        line = imu_stream.format_serial_line(s, 19200000 * 3)
        self.assertEqual(len(line.split(",")), 18)
        back = imu_stream.parse_serial_line(line, imu_stream.TimestampUnwrapper())
        self.assertAlmostEqual(back.t, 3.0)
        self.assertAlmostEqual(back.temp, 36.7, places=5)
        for a, b in zip(back.quat, s.quat):
            self.assertAlmostEqual(a, b, places=6)
        self.assertEqual(back.pos, s.pos)
        self.assertTrue(back.stationary)  # velocity exactly zero
        moving = s._replace(vel=(0.0, 1e-6, 0.0))
        back = imu_stream.parse_serial_line(imu_stream.format_serial_line(moving, 0), imu_stream.TimestampUnwrapper())
        self.assertFalse(back.stationary)

    def test_non_sample_lines_are_rejected(self):
        u = imu_stream.TimestampUnwrapper()
        self.assertIsNone(imu_stream.parse_serial_line("# calibration 1/6: ...", u))
        self.assertIsNone(imu_stream.parse_serial_line("", u))
        self.assertIsNone(imu_stream.parse_serial_line(",".join(["zz"] * 18), u))
        self.assertIsNone(imu_stream.parse_serial_line(",".join(["00000000"] * 17), u))

    def test_timestamp_unwrap(self):
        u = imu_stream.TimestampUnwrapper()
        t0 = u.seconds(0xFFFFFF00)
        t1 = u.seconds(0x00000100)  # wrapped
        self.assertAlmostEqual(t1 - t0, 0x200 / imu_stream.TIMESTAMP_HZ, places=9)
        # a small backwards step (out-of-order line) is not a wrap
        t2 = u.seconds(0x00000080)
        self.assertLess(t2, t1)

    def test_csv_row(self):
        row = {"t": "1.5", "stationary": "1", "qw": "1", "qx": "0", "qy": "0", "qz": "0", "vx": "0", "vy": "0", "vz": "0",
               "px": "0.1", "py": "0.2", "pz": "0.3", "ax": "0", "ay": "0", "az": "0", "gx": "0", "gy": "0", "gz": "0", "gain": "0.5"}
        s = imu_stream.parse_csv_row(row)
        self.assertEqual(s.pos, (0.1, 0.2, 0.3))
        self.assertTrue(s.stationary)
        self.assertTrue(math.isnan(s.temp))

    def test_euler_and_relative_pose(self):
        q = quat_from_axis_angle((0, 0, 1), math.radians(90))
        r, p, y = imu_stream.quat_to_euler_deg(q)
        self.assertAlmostEqual(y, 90.0, places=4)
        self.assertAlmostEqual(r, 0.0, places=4)
        self.assertAlmostEqual(p, 0.0, places=4)
        # a pose 1 m north of an origin that is yawed 90 deg lies on the origin's -y (east) side
        rel_p, rel_q = imu_stream.relative_pose(q, (0.0, 0.0, 0.0), q, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(rel_p[0], 0.0, places=5)
        self.assertAlmostEqual(rel_p[1], -1.0, places=5)
        self.assertAlmostEqual(abs(rel_q[0]), 1.0, places=5)  # same attitude as the origin
        # rotating a vector with the quaternion matches the matrix used by the viewer
        v = (0.3, -0.4, 0.5)
        qa = quat_from_axis_angle((1, 2, 3), 0.7)
        a = imu_stream.quat_rotate(qa, v)
        b = imu_viewer.quat_to_matrix(*qa) @ np.array(v)
        np.testing.assert_allclose(a, b, atol=1e-9)

    def test_replay_source_paces_by_timestamp(self):
        samples = [imu_stream.Sample(t=i * 0.1, temp=0, gyro=(0, 0, 0), accel=(0, 0, 0), quat=(1, 0, 0, 0),
                                     vel=(0, 0, 0), pos=(i, 0, 0), stationary=True) for i in range(5)]
        src = imu_stream.ReplaySource("mem", samples, realtime=False)
        out, _ = src.poll()
        self.assertEqual(len(out), 5)
        self.assertTrue(src.finished or src.index == 5)
        src = imu_stream.ReplaySource("mem", samples, speed=1000.0, realtime=True)
        out, _ = src.poll()  # first sample is due immediately
        self.assertGreaterEqual(len(out), 1)


class MonitorTests(unittest.TestCase):
    def test_signed_bar(self):
        self.assertEqual(imu_monitor.signed_bar(0.0, 1.0, 5), "[  |  ]")
        self.assertEqual(imu_monitor.signed_bar(1.0, 1.0, 5), "[  |==]")
        self.assertEqual(imu_monitor.signed_bar(-5.0, 1.0, 5), "[==|  ]")  # clamped

    def test_sparkline_bins_the_whole_history(self):
        values = [0.0] * 90 + [1.0] * 10
        s = imu_monitor.sparkline(values, 10)
        self.assertEqual(len(s), 10)
        self.assertEqual(s[-1], imu_monitor.SPARK[-1])
        self.assertEqual(s[0], imu_monitor.SPARK[0])

    def test_top_view_marks_origin_and_current(self):
        points = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)]
        lines = imu_monitor.top_view(points, (1.0, 1.0, 0.0), 40, 12)
        text = "\n".join(lines)
        self.assertIn("o", text)
        self.assertIn("@", text)
        # north is up: the current point (1 m north) is above the origin
        rows_o = [i for i, l in enumerate(lines) if "o" in l]
        rows_at = [i for i, l in enumerate(lines) if "@" in l]
        self.assertLess(rows_at[0], rows_o[0])
        # west is left: the current point (1 m west) is left of the origin
        self.assertLess(lines[rows_at[0]].index("@"), lines[rows_o[0]].index("o"))


class ViewerTests(unittest.TestCase):
    def test_projection(self):
        cam = np.array([-2.0, 0.0, 0.0])
        right = np.array([0.0, -1.0, 0.0])  # looking along +x with z up: right is -y
        up = np.array([0.0, 0.0, 1.0])
        fwd = np.array([1.0, 0.0, 0.0])
        pts = np.array([[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [-3.0, 0.0, 0.0]])
        out = np.zeros((4, 3))
        imu_viewer.project_points(pts, 4, cam, right, up, fwd, 100.0, 50.0, 40.0, out)
        np.testing.assert_allclose(out[0], [50.0, 40.0, 2.0])
        np.testing.assert_allclose(out[1, :2], [0.0, 40.0])   # +y (west) appears to the left
        np.testing.assert_allclose(out[2, :2], [50.0, -10.0])  # +z (up) appears higher (smaller y)
        self.assertLess(out[3, 2], imu_viewer.NEAR)            # behind the camera

    def test_compact_polyline(self):
        proj = np.array([[0.0, 0.0, 1.0], [0.2, 0.3, 1.0], [5.0, 0.0, 1.0], [5.0, 0.0, -1.0], [5.3, 0.1, 1.0]])
        out = np.zeros(10)
        k = imu_viewer.compact_polyline(proj, 5, 1.0, out)
        # the second point is < 1 px from the first, the fourth is behind the camera,
        # the last point is always kept
        self.assertEqual(k, 6)
        np.testing.assert_allclose(out[:6], [0.0, 0.0, 5.0, 0.0, 5.3, 0.1])
        self.assertEqual(imu_viewer.compact_polyline(np.array([[1.0, 1.0, -2.0]]), 1, 1.0, out), 0)

    def test_camera_basis_is_orthonormal(self):
        c = imu_viewer.Camera()
        c.az, c.el = 37.0, -20.0
        cam, right, up, fwd = c.basis()
        for v in (right, up, fwd):
            self.assertAlmostEqual(np.linalg.norm(v), 1.0, places=9)
        self.assertAlmostEqual(right @ up, 0.0, places=9)
        self.assertAlmostEqual(right @ fwd, 0.0, places=9)
        np.testing.assert_allclose(fwd, (c.target - cam) / np.linalg.norm(c.target - cam), atol=1e-9)

    def test_transform_points(self):
        local = np.array([[1.0, 0.0, 0.0]])
        out = np.zeros((1, 3))
        q = quat_from_axis_angle((0, 0, 1), math.radians(90))
        imu_viewer.transform_points(local, imu_viewer.quat_to_matrix(*q), np.array([1.0, 2.0, 3.0]), out)
        np.testing.assert_allclose(out[0], [1.0, 3.0, 3.0], atol=1e-9)

    def test_box_geometry(self):
        v = imu_viewer.box_vertices(2.0, 1.0, 0.5)
        self.assertEqual(v.shape, (8, 3))
        self.assertEqual(len(imu_viewer.BOX_EDGES), 12)
        for a, b in imu_viewer.BOX_EDGES:
            self.assertEqual(int(np.sum(v[a] != v[b])), 1)  # every edge changes exactly one coordinate


if __name__ == "__main__":
    unittest.main()
