#!/usr/bin/env python3
"""Batch mission harness.

Runs N headless 60-second missions in Docker (image built from
docker/Dockerfile) and aggregates each run's summary metrics into one
table + CSV, so tuning changes can be judged statistically instead of
from a single lucky/unlucky run.

Usage:
    python3 batch_run.py            # 10 runs, 2 in parallel
    python3 batch_run.py -n 20 -p 1 # 20 sequential runs

Requires: the iturover-noetic image (see docker/Dockerfile) and Docker
with the repo path shareable into the VM (on Colima: under /Users).
"""
import argparse
import concurrent.futures
import csv
import re
import statistics
import subprocess
import sys
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
LOGS = Path(__file__).resolve().parent / "logs"

# Columns for the printed table (aggregate.csv gets every parsed key).
TABLE_COLS = [
    ("gt_net_displacement", "gt_net"),
    ("net_displacement",    "enc_net"),
    ("encoder_minus_gt_path", "slip"),
    ("wedge_triggers",      "wedges"),
    ("min_r360_seen",       "min_r360"),
    ("w_flips",             "w_flips"),
    ("gap_switches",        "gap_sw"),
    ("fgm_fallbacks",       "fgm_fb"),
    ("commit_fallbacks",    "com_fb"),
    ("stuck",               "stuck"),
    ("wrong_way",           "wrong"),
    ("regress",             "regr"),
    ("cleared",             "doors"),
    ("scan_gaps",           "gaps"),
    ("cb_max",              "cb_max"),
]


def run_one(idx: int, batch_dir: Path, image: str) -> tuple:
    run_dir = batch_dir / f"run_{idx:02d}"
    run_dir.mkdir(parents=True, exist_ok=True)
    name = f"iturover-batch-{batch_dir.name}-{idx:02d}"
    cmd = ["docker", "run", "--rm", "--name", name,
           "-v", f"{REPO}:/repo:ro",
           "-v", f"{run_dir}:/root/rover_run_logs",
           image, "bash", "/run_sim.sh"]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
        ok = res.returncode == 0
    except subprocess.TimeoutExpired:
        subprocess.run(["docker", "rm", "-f", name], capture_output=True)
        ok = False
    print(f"  run {idx:02d}: {'done' if ok else 'FAILED'}", flush=True)
    return idx, ok


def parse_run(run_dir: Path) -> dict:
    """Pull every key=number pair out of the node's SUMMARY* log lines."""
    logs = sorted(run_dir.glob("run_*.log"))
    if not logs:
        return {}
    vals = {}
    for line in logs[-1].read_text().splitlines():
        if "SUMMARY" in line:
            for key, num in re.findall(r"(\w+)=\(?([-+]?\d+(?:\.\d+)?)", line):
                vals[key] = float(num)
    return vals


def fmt(v) -> str:
    if v is None:
        return "-"
    return f"{v:.2f}".rstrip("0").rstrip(".") if isinstance(v, float) else str(v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", "--runs", type=int, default=10)
    ap.add_argument("-p", "--parallel", type=int, default=2)
    ap.add_argument("--image", default="iturover-noetic")
    args = ap.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    batch_dir = LOGS / f"batch_{stamp}"
    batch_dir.mkdir(parents=True)
    print(f"Batch of {args.runs} missions ({args.parallel} parallel) → {batch_dir}")

    with concurrent.futures.ThreadPoolExecutor(max_workers=args.parallel) as ex:
        results = list(ex.map(lambda i: run_one(i, batch_dir, args.image),
                              range(1, args.runs + 1)))

    rows = []
    for idx, ok in results:
        vals = parse_run(batch_dir / f"run_{idx:02d}") if ok else {}
        vals["run"] = idx
        vals["ok"] = int(ok and bool(vals.get("net_displacement") is not None))
        rows.append(vals)

    all_keys = ["run", "ok"] + sorted({k for r in rows for k in r} - {"run", "ok"})
    with open(batch_dir / "aggregate.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=all_keys)
        writer.writeheader()
        writer.writerows(rows)

    # Printed table
    headers = ["run"] + [short for _, short in TABLE_COLS]
    widths = [max(7, len(h) + 1) for h in headers]
    print("\n" + "".join(h.rjust(w) for h, w in zip(headers, widths)))
    for r in rows:
        cells = [str(r["run"])] + [fmt(r.get(key)) for key, _ in TABLE_COLS]
        print("".join(c.rjust(w) for c, w in zip(cells, widths)))

    good = [r for r in rows if r.get("ok")]
    if len(good) >= 2:
        print("\nmean ± std over successful runs:")
        for key, short in TABLE_COLS:
            vals = [r[key] for r in good if key in r]
            if len(vals) >= 2:
                print(f"  {short:>8}: {statistics.mean(vals):6.2f} ± {statistics.stdev(vals):.2f}"
                      f"   (min {min(vals):.2f}, max {max(vals):.2f})")
    print(f"\nFull data: {batch_dir}/aggregate.csv")
    return 0 if len(good) == args.runs else 1


if __name__ == "__main__":
    sys.exit(main())
