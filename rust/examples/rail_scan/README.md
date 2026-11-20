# rail_scan

Drives the cart down the track while the lidar scans, and saves every point to
the laptop. Each lidar revolution is one cross-section of the track, so you can
stack them into a 3D map and measure gauge.

All commands run from `rust/`.

## Record a run

Plug in the Nucleo and run:

```
python examples/rail_scan/record.py
```

The cart starts driving 5 s after boot (90% speed, Boost scan mode). Press
Ctrl+C to stop the cart and end the recording. The blue button also
stops and starts the cart.

The files go in `scans/<date_time>/`. When it finishes, check that it prints
`Chunks missing: 0`, which means no data was lost.

## Make the CSV from a saved log

You don't need the board for this:

```
python examples/rail_scan/record.py scans/2026-11-12_13-46-39/raw.log
```

That creates `points.csv` (one row per lidar point) in a new `scans/` folder.

## Plot the rails

```
python examples/rail_scan/plot_rails.py scans/<folder>/points.csv
```

This draws every revolution on top of each other, so you get a sharp side view
of each rail. It also prints a rough lip-to-lip distance (not calibrated yet).
Needs numpy and matplotlib.

## Our best run so far

`scans/2026-11-12_13-46-39` is about 52 m of real track: the first half in
Boost mode and the second half in Stability mode. Both modes picked up the
rail head well. Sensitivity mode (the lidar's default) didn't, which is why
the firmware now forces Boost. `points.csv` isn't committed because it's
56 MB, so make it from `raw.log` with the command above.

## Settings

These are at the top of `src/main.rs`:

- `DRIVE_SPEED` is the motor % (60% won't move the cart).
- `CART_MM_PER_SEC` is the estimated speed, used for distance along the track.
  It's a guess, so measure it and update it.
- `LIDAR_MOTOR_SPEED` sets the spin speed. Faster spin gives more
  cross-sections per metre but fewer points in each one.
- `SCAN_MODE` sets the lidar mode (2 = Boost).

If the firmware hangs, a watchdog stops the cart within 3 s.
