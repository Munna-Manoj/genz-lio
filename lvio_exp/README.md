# lvio_exp: GenZ-LIO in the LVIO_exp harness

This branch (`lvio-exp`) of the fork adds only what [LVIO_exp](https://github.com/Munna-Manoj/LVIO_exp) needs to
run GenZ-LIO on COMFORT/GrandTour missions. Upstream's code is unchanged; ablation flags will be added here, each
defaulting to upstream behaviour.

| File | What it is |
|---|---|
| `run_comfort.py` | Headless runner: reads a mission's `comfort_offline/` folder (Hesai XT32 scans + `imu.txt`), pairs scans and IMU with upstream's `_ScanBuffer`, writes the IMU-frame TUM trajectory, `timing.csv` and `summary.json` |
| `grandtour_xt32.yaml` | `ros/config/default/hesai.yaml` with only the platform keys changed (marked `# LVIO_exp`): 4 threads, 0.6 m blind range, the LiDAR–IMU extrinsic |
| `Containerfile` | The image the harness runs: this checkout, built with `pip install` |

```bash
podman build -f lvio_exp/Containerfile -t lvx/genz-lio:py310 .
podman run --rm -v <mission>:/mission:ro -v <out>:/out lvx/genz-lio:py310 \
  taskset -c 0-3 python3 lvio_exp/run_comfort.py /mission/comfort_offline lvio_exp/grandtour_xt32.yaml /out --name arc-6
```

Licence: GPL-2.0, as upstream.
