#!/usr/bin/env python3
"""Run GenZ-LIO headless on a COMFORT/GrandTour `comfort_offline` folder (LVIO_exp harness, branch lvio-exp).

python lvio_exp/run_comfort.py <comfort_offline> <config.yaml> <out_dir> [--name MISSION]

Reads the Hesai XT32 scans (lidar/<scan start ns>.bin) and imu.txt directly; no bag conversion. Scans and IMU
samples are paired by upstream's own _ScanBuffer, exactly as the rosbag reader does. Writes, in <out_dir>:
  <name>_imu.tum   the IMU-frame pose per scan (result.pose), TUM: t x y z qx qy qz qw
  timing.csv       stamp, ms, rss_mb, leaf_m, inliers: the columns LVIO_exp reads for every system
  summary.json     frames, skipped, empty clouds, wall time
"""
from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import numpy as np
import yaml

from genz_lio import GenZLIO, load_config
from genz_lio.genz_lio_pybind import _ScanBuffer

XT32 = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "timestamp"],
                 "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f8"],
                 "offsets": [0, 4, 8, 12, 16, 18], "itemsize": 26})   # x y z intensity ring t(absolute s)
PAGE_MB = os.sysconf("SC_PAGE_SIZE") / 2**20


def rss_mb() -> float:
    with open("/proc/self/statm") as f:
        return int(f.read().split()[1]) * PAGE_MB


def events(offline: Path):
    """IMU samples and scans of the folder, in time order: ('imu', row) or ('scan', stamp_s, path)."""
    raw = np.loadtxt(offline / "imu.txt")                    # t_ns gx gy gz ax ay az
    imu = np.column_stack([raw[:, 0] * 1e-9, raw[:, 4:7], raw[:, 1:4]])   # -> t ax ay az gx gy gz (genz order)
    scans = sorted((int(p.stem) * 1e-9, p) for p in (offline / "lidar").glob("*.bin"))
    out = [(t, 0, ("imu", row)) for t, row in zip(imu[:, 0], imu)]
    out += [(t, 1, ("scan", t, p)) for t, p in scans]
    out.sort(key=lambda e: (e[0], e[1]))
    return [e[2] for e in out]


def quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> x, y, z, w."""
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("offline", type=Path)
    ap.add_argument("config", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--name", default=None, help="file prefix, default: the mission folder name")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    name = a.name or a.offline.parent.name

    settings = load_config(a.config)
    time_offset = float((yaml.safe_load(a.config.read_text()).get("common") or {}).get("time_offset", 0.0))  # reader key
    lio = GenZLIO(settings)
    buffer = _ScanBuffer(settings.preprocess)
    poses, timing, skipped, empty = [], [], 0, 0
    started = time.perf_counter()

    def drain():
        nonlocal skipped
        while True:
            bundle = buffer.pop()
            if bundle is None:
                return
            points, begin, end, imu, times, intensities, rings, _ = bundle
            r = lio.register_scan(points, begin, end, imu, timestamps=times, intensities=intensities,
                                  rings=rings, timing_prepared=True)
            if not r.valid:
                skipped += 1
                continue
            P = r.pose
            poses.append((r.timestamp, P[:3, 3], quat(P[:3, :3])))
            timing.append((r.timestamp, r.processing_time_ms, rss_mb(), r.leaf_size,
                           r.plane_matches + r.point_matches))

    for e in events(a.offline):
        if e[0] == "imu":
            event = buffer.push_imu(e[1][None, :])
        else:
            _, stamp, path = e
            p = np.fromfile(path, dtype=XT32)
            if len(p) == 0:
                empty += 1
                continue
            pts = np.column_stack([p["x"], p["y"], p["z"]]).astype(np.float64)
            event = buffer.push_scan(pts, stamp + time_offset, p["timestamp"] - stamp,   # hesai: relative s
                                     p["intensity"].astype(np.float32), p["ring"].astype(np.int32))
        if event in (1, 2):
            raise SystemExit("timestamps went backwards in the input")
        drain()
    drain()

    with open(a.out / f"{name}_imu.tum", "w") as f:
        for t, p, q in poses:
            f.write(f"{t:.9f} " + " ".join(f"{x:.6f}" for x in p) + " " + " ".join(f"{x:.8f}" for x in q) + "\n")
    with open(a.out / "timing.csv", "w") as f:
        f.write("stamp,ms,rss_mb,leaf_m,inliers\n")
        for row in timing:
            f.write("{:.9f},{:.3f},{:.0f},{:.3f},{}\n".format(*row))
    summary = {"frames": len(poses), "skipped": skipped, "empty_clouds": empty,
               "pending_scans": int(buffer.pending_scans), "wall_s": round(time.perf_counter() - started, 1)}
    (a.out / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
