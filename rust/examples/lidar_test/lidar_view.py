"""Live top-down view of RPLIDAR points logged by the firmware.

Usage, from the rust folder:
    python examples/lidar_test/lidar_view.py            flash the board and show live data
    python examples/lidar_test/lidar_view.py run.log    replay a saved log instead
"""

import math
import pathlib
import re
import subprocess
import sys
import threading
import time
import tkinter as tk

RANGE_MM = 3000  # Edge of the view; farther points are off screen
RING_STEP_MM = 500
POINT_LIFETIME_S = 0.5  # Points vanish after this long without an update
SIZE_PX = 700
REFRESH_MS = 50

POINT_PATTERN = re.compile(r"distance=([\d.]+) mm, angle=([\d.]+) deg")

points = {}  # Whole-degree angle -> (distance_mm, angle_deg, last_seen_time)
process = None


def handle_line(line):
    match = POINT_PATTERN.search(line)
    if match:
        distance_mm = float(match.group(1))
        angle_deg = float(match.group(2))
        points[int(angle_deg) % 360] = (distance_mm, angle_deg, time.monotonic())
    elif "WARN" in line or "error" in line.lower():
        print(line.rstrip())


def read_live():
    global process
    rust_folder = pathlib.Path(__file__).resolve().parents[2]
    process = subprocess.Popen(
        ["cargo", "run", "--release", "-p", "lidar_test"],
        cwd=rust_folder,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        errors="replace",
    )
    for line in process.stdout:
        handle_line(line)


def read_replay(path):
    with open(path, errors="replace") as log:
        for line in log:
            handle_line(line)
            time.sleep(0.0005)


def draw(canvas):
    center = SIZE_PX / 2
    scale = (center - 20) / RANGE_MM
    canvas.delete("all")

    for ring_mm in range(RING_STEP_MM, RANGE_MM + 1, RING_STEP_MM):
        radius = ring_mm * scale
        canvas.create_oval(
            center - radius, center - radius, center + radius, center + radius,
            outline="#333333",
        )
        canvas.create_text(
            center + 4, center - radius + 8,
            text=f"{ring_mm} mm", fill="#777777", anchor="w",
        )

    canvas.create_oval(center - 4, center - 4, center + 4, center + 4, fill="#ff5555", outline="")

    now = time.monotonic()
    for distance_mm, angle_deg, seen in list(points.values()):
        if now - seen > POINT_LIFETIME_S:
            continue
        # 0 degrees points up and angles increase clockwise
        angle_rad = math.radians(angle_deg)
        x = center + distance_mm * scale * math.sin(angle_rad)
        y = center - distance_mm * scale * math.cos(angle_rad)
        canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill="#55ff88", outline="")

    canvas.after(REFRESH_MS, draw, canvas)


def main():
    source = sys.argv[1] if len(sys.argv) > 1 else None
    target = read_replay if source else read_live
    args = (source,) if source else ()
    threading.Thread(target=target, args=args, daemon=True).start()

    window = tk.Tk()
    window.title("RailRat lidar (red dot is the lidar)")
    canvas = tk.Canvas(window, width=SIZE_PX, height=SIZE_PX, bg="black", highlightthickness=0)
    canvas.pack()
    draw(canvas)
    window.mainloop()

    if process:
        process.terminate()


if __name__ == "__main__":
    main()
