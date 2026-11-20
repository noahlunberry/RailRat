"""
Side-view (cross-section) plot of both rails from a rail_scan recording.

Every lidar point from the whole run is turned into (sideways, down)
millimetres, leveled, and drawn on top of each other. Because the rail
profile barely changes along the track, thousands of revolutions stack
up into one sharp "reverse C" outline per rail. One row per scan mode.

    python examples/rail_scan/plot_rails.py scans/<run>/points.csv [out.png]

Needs numpy and matplotlib.
"""
import csv
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# The lidar is mounted slightly rolled. This slope (about 3.9 degrees) makes
# the tie tops flat; it was fit from the 2026-11-12 runs.
ROLL_SLOPE = -0.0677
SKIP_SECS = 2.3          # ignore the start while the lidar spins up
MAX_RANGE_MM = 1500      # anything further is trees, not track
RAIL_X_MM = (550, 950)   # sideways window around each rail
DEPTH_MM = (0, 420)      # down from the lidar


def load(path):
    t, ang, dist, mode = [], [], [], []
    with open(path) as f:
        rows = csv.reader(f)
        ix = {name: i for i, name in enumerate(next(rows))}
        for row in rows:
            t.append(int(row[ix["t_us"]]))
            ang.append(float(row[ix["angle_deg"]]))
            dist.append(float(row[ix["distance_mm"]]))
            mode.append(row[ix["scan_mode"]])
    return np.array(t) / 1e6, np.array(ang), np.array(dist), np.array(mode)


def lip(x, y, sel, side):
    """Head top, then the innermost point 8-24 mm below it (the gauge lip)."""
    head = sel & (side * x > 650) & (side * x < 820) & (y > 60) & (y < 230)
    if head.sum() < 50:
        return None
    top = np.percentile(y[head], 3)
    band = head & (y > top + 8) & (y < top + 24)
    if band.sum() < 20:
        return None
    return np.percentile(x[band], 95 if side < 0 else 5)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    out = sys.argv[2] if len(sys.argv) > 2 else "rails_by_mode.png"
    t, ang, dist, mode = load(sys.argv[1])

    # angle 0 = straight down, positive = right; then undo the roll
    th = np.radians(ang)
    sx, sy = dist * np.sin(th), dist * np.cos(th)
    k = np.arctan(ROLL_SLOPE)
    x = sx * np.cos(k) + sy * np.sin(k)
    y = -sx * np.sin(k) + sy * np.cos(k)
    valid = (dist > 0) & (dist < MAX_RANGE_MM) & (t > SKIP_SECS)

    modes = sorted(set(mode[valid]))
    fig, axs = plt.subplots(len(modes), 2, figsize=(13, 5 * len(modes)), squeeze=False)
    for r, m in enumerate(modes):
        sel = valid & (mode == m)
        for c, side in enumerate((-1, 1)):
            ax = axs[r][c]
            win = (sel & (side * x > RAIL_X_MM[0]) & (side * x < RAIL_X_MM[1])
                   & (y > DEPTH_MM[0]) & (y < DEPTH_MM[1]))
            ax.scatter(x[win][::4], y[win][::4], s=0.2, alpha=0.15)
            ax.set_xlim(*sorted((side * RAIL_X_MM[0], side * RAIL_X_MM[1])))
            ax.set_ylim(DEPTH_MM[1], DEPTH_MM[0])
            ax.set_aspect("equal")
            ax.grid(alpha=0.3)
            name = "left" if side < 0 else "right"
            ax.set_title(f"{m}: {name} rail (leveled, mm below lidar)")
        left, right = lip(x, y, sel, -1), lip(x, y, sel, 1)
        if left is not None and right is not None:
            print(f"{m}: lip-to-lip about {right - left:.0f} mm (not calibrated)")

    plt.tight_layout()
    plt.savefig(out, dpi=80)
    print(f"Saved {out}")


if __name__ == "__main__":
    main()
