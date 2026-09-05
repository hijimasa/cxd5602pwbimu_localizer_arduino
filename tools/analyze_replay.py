#!/usr/bin/env python3
"""Summarize a replay CSV (test/replay --csv): rest/motion segments,
displacement of every motion segment, drift during rest and attitude change.

Usage: analyze_replay.py REPLAY.csv [--min-motion 0.2]
"""
import argparse
import csv
import math


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


def quat_angle(q0, q1):
    d = abs(sum(a * b for a, b in zip(q0, q1)))
    return 2.0 * math.degrees(math.acos(min(1.0, d)))


def up_vector(q):
    w, x, y, z = q
    return [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--min-motion", type=float, default=0.2, help="ignore motion segments shorter than this (s)")
    args = ap.parse_args()

    rows = []
    with open(args.csv) as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) for k, v in r.items()})
    if not rows:
        print("empty")
        return

    # segments of constant stationary flag
    segs = []
    start = 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or rows[i]["stationary"] != rows[start]["stationary"]:
            segs.append((start, i - 1))
            start = i
    t0 = rows[0]["t"]
    q0 = [rows[0][k] for k in ("qw", "qx", "qy", "qz")]
    qN = [rows[-1][k] for k in ("qw", "qx", "qy", "qz")]
    print(f"duration {rows[-1]['t'] - t0:.1f} s, rows {len(rows)}, attitude change first->last {quat_angle(q0, qN):.3f} deg")
    print(f"gravity direction in sensor frame first {['%.3f' % v for v in up_vector(q0)]} last {['%.3f' % v for v in up_vector(qN)]}")
    speeds = [norm([r["vx"], r["vy"], r["vz"]]) for r in rows]
    print(f"max speed {max(speeds):.3f} m/s")

    print("\nsegments (t from first output):")
    print(" idx  state     start     end   dur[s]   displacement[m] (dx dy dz)      |d|    max|a|   max|w|[deg/s]")
    prev_rest_pos = None
    n_short = 0
    for k, (a, b) in enumerate(segs):
        st = rows[a]["stationary"] == 1
        dur = rows[b]["t"] - rows[a]["t"]
        pa = [rows[a]["px"], rows[a]["py"], rows[a]["pz"]]
        pb = [rows[b]["px"], rows[b]["py"], rows[b]["pz"]]
        d = sub(pb, pa)
        maxa = max(norm([r["ax"], r["ay"], r["az"]]) for r in rows[a:b + 1])
        maxw = max(norm([r["gx"], r["gy"], r["gz"]]) for r in rows[a:b + 1]) * 57.2958
        if not st and dur < args.min_motion:
            n_short += 1
            continue
        label = "REST" if st else "MOTION"
        print(f" {k:3d}  {label:7s} {rows[a]['t'] - t0:8.2f} {rows[b]['t'] - t0:8.2f} {dur:7.2f}   "
              f"{d[0]:+.4f} {d[1]:+.4f} {d[2]:+.4f}   {norm(d):.4f}   {maxa:6.3f}   {maxw:7.2f}")
        if st:
            if prev_rest_pos is not None:
                # displacement between consecutive rests (after blending)
                dd = sub(pb, prev_rest_pos)
                print(f"        rest-to-rest displacement {dd[0]:+.4f} {dd[1]:+.4f} {dd[2]:+.4f}  |d| = {norm(dd):.4f} m")
            prev_rest_pos = pb
    if n_short:
        print(f" ({n_short} motion segment(s) shorter than {args.min_motion} s omitted: brief ZUPT drop-outs)")
    pe = [rows[-1]["px"], rows[-1]["py"], rows[-1]["pz"]]
    print(f"\nfinal position {pe[0]:+.4f} {pe[1]:+.4f} {pe[2]:+.4f}  |p| = {norm(pe):.4f} m")


if __name__ == "__main__":
    main()
