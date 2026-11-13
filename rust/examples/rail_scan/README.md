# rail_scan: record every lidar point on the track

Streams all raw lidar bytes to the laptop and saves them, while driving the
cart and estimating how far it has gone along the track.

Run from `rust/`, with the Nucleo plugged in over USB:

```
python examples/rail_scan/record.py
```

1. At startup the lidar lists its scan modes and the fastest one is picked
   (`Scan mode ...` / `Using scan mode N` lines).
2. The cart starts on its own `AUTO_START_SECS` (5 s) after boot and drives
   continuously. The blue USER button stops it; pressing again starts it again.
   (`STOP_AND_GO = true` switches to drive/stop/scan steps instead.)
3. It drives until you press Ctrl+C, which ends the recording and stops the cart
   within ~3-5 s (see Safety). The blue button stops it instantly.

After each station it prints how far out the lidar saw on each side and how
many points landed in the rail zone (|x| > 680 mm). If those counts stay near 0
on a side, that rail is not visible to the lidar. With `STOP_AND_GO = false` it
prints rate stats every 2 s instead.

Output goes to `rust/scans/<date_time>/`:

| File | Contents |
| --- | --- |
| `raw.log` | Everything probe-rs printed. Re-decode with `python examples/rail_scan/record.py <raw.log>` |
| `points.csv` | `t_us, along_track_mm, rev, angle_deg, distance_mm, quality, start_flag`, one row per measurement |
| `events.csv` | Cart start/stop times and speeds, scan starts, any missing chunks |
| `stations.csv` | One row per stop: position, revolutions, widest point seen left/right, rail-zone point counts |

`points.csv` also has a `station` column: the station number for points taken
while stopped, blank while moving. Stack all points of one station to get a
clean cross-section.

At the end it prints the point count and `Chunks missing`. 0 means nothing was lost.

## Settings (top of `src/main.rs`)

| Constant | What it does |
| --- | --- |
| `DRIVE_SPEED` | Motor %, negative reverses |
| `CART_MM_PER_SEC` | Estimated cart speed at `DRIVE_SPEED`. Calibrate: drive 10 s, measure, divide by 10 |
| `AUTO_START_SECS` | `None` = wait for the button instead |
| `STOP_AND_GO` | `true` = step/stop/scan, `false` = drive continuously |
| `STEP_MS`, `SETTLE_MS`, `HOLD_MS` | Drive time per step, wait for rocking to stop, scan time per station |
| `LIDAR_MOTOR_SPEED` | Slower spin = more points per degree, but more track between cross-sections |
| `SCAN_MODE` | `None` = fastest mode, `Some(id)` forces one |

## Point density

The lidar takes a fixed number of samples per second, so:

- degrees between points = 360 x (revolutions/s) / (samples/s)
- track between cross-sections = cart speed / (revolutions/s)

At 4 rev/s: standard mode (2k samples/s) is ~0.7 deg apart, the 8k modes are
~0.18 deg apart, which is ~2 mm between points on a rail 0.7 m away.

There is no range setting on the RPLIDAR A1. Closer targets are measured more
precisely anyway. The minimum range is about 15 cm, so keep the rails farther
than that from the lidar.

## How it avoids losing data

- The board does not decode anything. It forwards the lidar's raw bytes in
  numbered 250-byte chunks, so the laptop can detect any gap.
- probe-rs puts the defmt channel in blocking mode: if the laptop falls behind,
  the board waits instead of dropping data.
- While the board waits, the lidar keeps filling a 48 KB DMA buffer (about 4 s of
  data), which drains once the laptop catches up.

## Safety

A 3 s watchdog resets the chip if the firmware stops running, for example when
probe-rs halts it on Ctrl+C or the laptop stops reading. The reset stops the
cart. After a watchdog reset the cart does NOT auto-start; it waits for the
button. (A normal power-up or reset button press still auto-starts.)

## For a 3D map

Mount the lidar with its scan plane vertical and across the rails, so each
revolution is one cross-section of the track. Each point then maps to
x = `along_track_mm`, and (y, z) from `angle_deg` and `distance_mm`.

## Rail side view

```
python examples/rail_scan/plot_rails.py scans/<run>/points.csv scans/<run>/rails_by_mode.png
```

Converts every point to (sideways, down) mm, levels out the lidar's ~3.9 deg roll
(`ROLL_SLOPE`), and plots all revolutions on top of each other around each rail.
The rail barely changes along the track, so thousands of revolutions stack up
into one sharp outline per rail, one row per scan mode. It also prints a rough
lip-to-lip distance (not calibrated yet).

## Saved run: `scans/2026-11-12_13-46-39`

Real track, about 52 m: Boost (mode 2) for the first ~26.7 m, then Stability
(mode 4). 1.12 M points, 0 chunks missing. Boost and Stability both see the
rail head and gauge face well. Sensitivity (mode 3, the lidar's default) does
not, which is why `SCAN_MODE` is forced to Boost.

`points.csv` (56 MB) is not committed. Rebuild it from `raw.log` with
`python examples/rail_scan/record.py scans/2026-11-12_13-46-39/raw.log`, which
writes it into a new `scans/<date_time>/` folder.
