"""Record a rail_scan run: every lidar point, saved to disk.

Usage, from the rust folder:
    python examples/rail_scan/record.py               flash the board and record until Ctrl+C
    python examples/rail_scan/record.py raw.log       re-decode a saved raw log into a new CSV

Each run makes a folder scans/<date_time>/ containing:
    raw.log      everything probe-rs printed (the source of truth, can be re-decoded)
    points.csv   one row per lidar measurement
    events.csv   cart drive start/stop, scan starts, missing chunks

points.csv columns:
    t_us            board time in microseconds (estimated per point, see below)
    along_track_mm  estimated distance the cart has moved since boot (speed x time)
    rev             lidar revolution number, +1 each time the lidar passes angle 0
    angle_deg       0-360, clockwise looking down on the lidar
    distance_mm     0 means the lidar got no valid return at that angle
    quality         signal strength 0-63 in standard mode; blank in the faster
                    modes, which don't report it
    start_flag      1 on the first point of each revolution

The board stamps each chunk with the time its last byte was read, and points are
spread back from there by UART byte time. If the computer fell behind, chunks
get read late and in bursts, so then times (and along_track_mm) lag behind
reality; rev is still exact.

Scan formats (picked by the firmware, see the "Scan mode" lines at startup):
    0x81 standard, 5 bytes per sample
    0x82 express capsules, 32 samples per 84 bytes
    0x84 ultra capsules, 96 samples per 132 bytes (A1 "Boost"/"Sensitivity")
    0x85 dense capsules, 40 samples per 84 bytes
Capsule decoding follows the Slamtec rplidar SDK.

Only uses the Python standard library.
"""

import csv
import datetime
import math
import pathlib
import re
import subprocess
import sys
import time

PACKAGE = "rail_scan"
BYTE_TIME_US = 10 / 115_200 * 1e6  # 8N1 framing: 10 bits per byte

CHUNK_PATTERN = re.compile(r"\bD (\d+) (\d+) \[([\d, ]*)\]")
SCAN_START_PATTERN = re.compile(r"\bSCAN_START (\d+)(?: (\d+))?(?: (\d+))?")
SCAN_MODE_PATTERN = re.compile(r"\bScan mode (\d+): (\S+)")
DRIVE_PATTERN = re.compile(r"\bDRIVE (\d+) (-?\d+)(?: (-?\d+))?")
STATION_START_PATTERN = re.compile(r"\bSTATION_START (\d+) (\d+) (-?\d+)")
STATION_END_PATTERN = re.compile(r"\bSTATION_END (\d+) (\d+)")

# Station report: points farther out than this (mm, either side) count as "rail zone"
RAIL_ZONE_MM = 680
# Ignore returns farther than this (stray hits on things beside the track)
MAX_RANGE_MM = 1500

RUST_FOLDER = pathlib.Path(__file__).resolve().parents[2]

ANS_STANDARD = 0x81
ANS_CAPSULED = 0x82
ANS_ULTRA_CAPSULED = 0x84
ANS_DENSE_CAPSULED = 0x85


class StandardDecoder:
    """Standard scan: 5 bytes per point.

    Same packet checks as the firmware driver: bit 0 and bit 1 of byte 0 must
    differ, and bit 0 of byte 1 must be 1. Otherwise slide forward one byte.
    Random bytes pass that check 1 time in 4, so after a gap a packet only counts
    once the packet after it passes too.
    """

    def __init__(self, emit):
        self.emit = emit
        self.pending = bytearray()
        self.locked = False
        self.skipped_bytes = 0

    def restart(self, aligned=False):
        """Drop partial bytes. aligned=True when the next byte is known to start a packet."""
        self.pending.clear()
        self.locked = aligned

    @staticmethod
    def looks_valid(buf, i):
        return ((buf[i] & 1) != ((buf[i] >> 1) & 1)) and (buf[i + 1] & 1) == 1

    def feed(self, data, chunk_end_us):
        self.pending += data
        i = 0
        end = len(self.pending)

        while end - i >= 5:
            if not self.looks_valid(self.pending, i):
                self.locked = False
                i += 1
                self.skipped_bytes += 1
                continue

            if not self.locked:
                if end - i < 10:
                    break  # wait for the next chunk to confirm
                if not self.looks_valid(self.pending, i + 5):
                    i += 1
                    self.skipped_bytes += 1
                    continue
                self.locked = True

            b0, b1, b2, b3, b4 = self.pending[i:i + 5]

            # Bytes received after this packet, still inside this chunk
            bytes_after = end - (i + 5)
            t_us = chunk_end_us - bytes_after * BYTE_TIME_US

            angle_deg = ((b1 >> 1) | (b2 << 7)) / 64.0
            distance_mm = (b3 | (b4 << 8)) / 4.0
            self.emit(t_us, angle_deg, distance_mm, b0 >> 2, b0 & 1)
            i += 5

        del self.pending[:i]


def varbitscale_decode(scaled):
    """Ultra capsule distance compression. Returns (value, scale level)."""
    for base, level, target in ((3328, 4, 1 << 14), (1792, 3, 1 << 12), (1280, 2, 1 << 11), (512, 1, 1 << 9)):
        if scaled >= base:
            return target + ((scaled - base) << level), level
    return scaled, 0


def signed10(value):
    return value - 1024 if value & 0x200 else value


class CapsuleDecoder:
    """Express / ultra / dense capsules.

    Each capsule holds a start angle plus a batch of distances. The angles of a
    capsule's samples are spread between its start angle and the NEXT capsule's,
    so a capsule is decoded once the one after it arrives.
    """

    SIZES = {ANS_CAPSULED: 84, ANS_ULTRA_CAPSULED: 132, ANS_DENSE_CAPSULED: 84}

    def __init__(self, answer_type, emit):
        self.answer_type = answer_type
        self.size = self.SIZES[answer_type]
        self.emit = emit
        self.pending = bytearray()
        self.previous = None  # (start_deg, capsule bytes, time_us)
        self.last_angle = None
        self.skipped_bytes = 0

    def restart(self, aligned=False):
        self.pending.clear()
        self.previous = None
        self.last_angle = None

    def header_ok(self, i):
        buf = self.pending
        b0, b1 = buf[i], buf[i + 1]
        if (b0 >> 4) != 0xA or (b1 >> 4) != 0x5:
            return False
        checksum = 0
        for b in buf[i + 2:i + self.size]:
            checksum ^= b
        return checksum == ((b0 & 0xF) | ((b1 & 0xF) << 4))

    def feed(self, data, chunk_end_us):
        self.pending += data
        i = 0
        end = len(self.pending)

        while end - i >= self.size:
            if not self.header_ok(i):
                i += 1
                self.skipped_bytes += 1
                self.previous = None
                continue

            capsule = bytes(self.pending[i:i + self.size])
            bytes_after = end - (i + self.size)
            t_us = chunk_end_us - bytes_after * BYTE_TIME_US

            start_word = capsule[2] | (capsule[3] << 8)
            start_deg = (start_word & 0x7FFF) / 64.0
            if start_word & 0x8000:
                self.previous = None  # first capsule after the scan (re)started

            if self.previous is not None:
                self.decode_previous(start_deg, capsule, t_us)

            self.previous = (start_deg, capsule, t_us)
            i += self.size

        del self.pending[:i]

    def decode_previous(self, next_start_deg, next_capsule, next_t_us):
        start_deg, cap, t_us = self.previous
        span = (next_start_deg - start_deg) % 360.0
        samples = []  # (angle_deg, distance_mm)

        if self.answer_type == ANS_CAPSULED:
            step = span / 32
            for c in range(16):
                o = 4 + 5 * c
                d1 = cap[o] | (cap[o + 1] << 8)
                d2 = cap[o + 2] | (cap[o + 3] << 8)
                offsets = cap[o + 4]
                offset1_q3 = (offsets & 0xF) | ((d1 & 0x3) << 4)
                offset2_q3 = (offsets >> 4) | ((d2 & 0x3) << 4)
                samples.append((start_deg + step * (2 * c) - offset1_q3 / 8, d1 >> 2))
                samples.append((start_deg + step * (2 * c + 1) - offset2_q3 / 8, d2 >> 2))

        elif self.answer_type == ANS_ULTRA_CAPSULED:
            step = span / 96
            for c in range(32):
                o = 4 + 4 * c
                combined = int.from_bytes(cap[o:o + 4], "little")
                major = combined & 0xFFF
                predict1 = signed10((combined >> 12) & 0x3FF)
                predict2 = signed10((combined >> 22) & 0x3FF)

                if c < 31:
                    major2 = int.from_bytes(cap[o + 4:o + 8], "little") & 0xFFF
                else:
                    major2 = int.from_bytes(next_capsule[4:8], "little") & 0xFFF

                major, level1 = varbitscale_decode(major)
                major2, level2 = varbitscale_decode(major2)
                base1, base2 = major, major2
                if major == 0 and major2 != 0:
                    base1, level1 = major2, level2

                dist1 = 0 if predict1 in (-512, 511) else (predict1 << level1) + base1
                dist2 = 0 if predict2 in (-512, 511) else (predict2 << level2) + base2

                for k, distance in enumerate((major, dist1, dist2)):
                    samples.append((start_deg + step * (3 * c + k), max(distance, 0)))

        else:  # dense
            step = span / 40
            for k in range(40):
                o = 4 + 2 * k
                samples.append((start_deg + step * k, cap[o] | (cap[o + 1] << 8)))

        count = len(samples)
        for k, (angle_deg, distance_mm) in enumerate(samples):
            angle_deg %= 360.0
            start_flag = int(self.last_angle is not None and angle_deg < self.last_angle - 180)
            self.last_angle = angle_deg
            sample_t = t_us + (next_t_us - t_us) * k / count
            self.emit(sample_t, angle_deg, float(distance_mm), "", start_flag)


class Recorder:
    def __init__(self, out_folder):
        self.points_file = open(out_folder / "points.csv", "w", newline="")
        self.events_file = open(out_folder / "events.csv", "w", newline="")
        self.points = csv.writer(self.points_file)
        self.points.writerow([
            "t_us", "along_track_mm", "rev", "angle_deg", "distance_mm", "quality", "start_flag", "station", "scan_mode",
        ])
        self.events = csv.writer(self.events_file)
        self.events.writerow(["t_us", "event", "value", "cart_mm_per_s"])
        self.stations_file = open(out_folder / "stations.csv", "w", newline="")
        self.stations = csv.writer(self.stations_file)
        self.stations.writerow([
            "station", "start_t_us", "end_t_us", "along_track_mm", "revs", "valid_points",
            "left_edge_mm", "right_edge_mm", "left_rail_zone_points", "right_rail_zone_points",
        ])

        # Stop-and-go stations. A station's last points are decoded a little after
        # its STATION_END line arrives, so a closed station stays open for late points.
        self.open_station = None     # (n, start_t_us, along_mm)
        self.closing_station = None  # (n, start_t_us, along_mm, end_t_us)
        self.station_points = []     # (x_mm, y_mm) of valid points
        self.station_revs = 0
        self.station_count = 0

        # Old logs have no format on SCAN_START; those were standard scans
        self.decoder = StandardDecoder(self.emit)
        self.skipped_bytes = 0
        self.expected_seq = None
        self.missing_chunks = 0
        self.scan_restarts = 0

        self.rev = 0
        self.point_count = 0

        # Scan mode labels, e.g. "2 Boost", from the firmware's "Scan mode" lines
        self.mode_names = {}
        self.scan_mode = ""
        self.points_per_mode = {}

        # Odometer: speed is constant between DRIVE events
        self.cart_mm_per_s = 0
        self.odometer_mm = 0.0
        self.odometer_t_us = None
        self.last_along_mm = 0.0

        self.last_status = time.monotonic()
        self.window_points = 0
        self.window_revs = 0
        self.window_start_t_us = None

    def along_track_mm(self, t_us):
        if self.odometer_t_us is None:
            return self.odometer_mm
        return self.odometer_mm + self.cart_mm_per_s * (t_us - self.odometer_t_us) / 1e6

    def station_at(self, t_us):
        if self.closing_station and self.closing_station[1] <= t_us <= self.closing_station[3]:
            return self.closing_station[0]
        if self.open_station and t_us >= self.open_station[1]:
            return self.open_station[0]
        return ""

    def emit(self, t_us, angle_deg, distance_mm, quality, start_flag):
        if self.closing_station and t_us > self.closing_station[3]:
            self.report_station()

        if start_flag:
            self.rev += 1
            self.window_revs += 1
        along = self.along_track_mm(t_us)
        self.last_along_mm = along

        station = self.station_at(t_us)
        if station != "":
            self.station_revs += start_flag
            if 0 < distance_mm < MAX_RANGE_MM:
                angle = math.radians(angle_deg)
                self.station_points.append((distance_mm * math.sin(angle), distance_mm * math.cos(angle)))

        self.points.writerow([
            round(t_us), f"{along:.1f}", self.rev, f"{angle_deg:.4f}", f"{distance_mm:.2f}", quality, start_flag,
            station, self.scan_mode,
        ])
        self.points_per_mode[self.scan_mode] = self.points_per_mode.get(self.scan_mode, 0) + 1
        self.point_count += 1
        self.window_points += 1
        if self.window_start_t_us is None:
            self.window_start_t_us = t_us
        self.window_end_t_us = t_us

    def new_decoder(self, answer_type):
        self.skipped_bytes += self.decoder.skipped_bytes
        if answer_type in CapsuleDecoder.SIZES:
            self.decoder = CapsuleDecoder(answer_type, self.emit)
        else:
            if answer_type != ANS_STANDARD:
                print(f"!! unknown scan format 0x{answer_type:02x}, decoding as standard")
            self.decoder = StandardDecoder(self.emit)
        # The board already consumed the scan descriptor, so the next byte starts a packet
        self.decoder.restart(aligned=True)

    def handle_line(self, line):
        match = CHUNK_PATTERN.search(line)
        if match:
            seq = int(match.group(1))
            t_us = int(match.group(2))
            data = bytes(int(b) for b in match.group(3).split(",") if b.strip())

            if self.expected_seq is not None and seq != self.expected_seq:
                gap = (seq - self.expected_seq) % 2**32
                self.missing_chunks += gap
                print(f"!! {gap} chunk(s) missing before chunk {seq}")
                self.events.writerow([t_us, "missing_chunks", gap, ""])
                self.decoder.restart()
            self.expected_seq = (seq + 1) % 2**32

            self.decoder.feed(data, t_us)
            self.print_status()
            return

        match = SCAN_START_PATTERN.search(line)
        if match:
            self.scan_restarts += 1
            answer_type = int(match.group(2)) if match.group(2) else ANS_STANDARD
            if match.group(3):
                mode_id = int(match.group(3))
                self.scan_mode = f"{mode_id} {self.mode_names.get(mode_id, '?')}"
            self.new_decoder(answer_type)
            self.events.writerow([match.group(1), "scan_start", f"0x{answer_type:02x}", self.scan_mode])
            print(f">> scan started, mode {self.scan_mode or '?'}, format 0x{answer_type:02x}")
            return

        match = SCAN_MODE_PATTERN.search(line)
        if match:
            self.mode_names[int(match.group(1))] = match.group(2)

        match = STATION_START_PATTERN.search(line)
        if match:
            if self.closing_station:
                self.report_station()
            n, t_us, along = (int(g) for g in match.groups())
            self.open_station = (n, t_us, along)
            self.station_points = []
            self.station_revs = 0
            return

        match = STATION_END_PATTERN.search(line)
        if match:
            n, t_us = int(match.group(1)), int(match.group(2))
            if self.open_station and self.open_station[0] == n:
                self.closing_station = (*self.open_station, t_us)
            self.open_station = None
            return

        match = DRIVE_PATTERN.search(line)
        if match:
            t_us = int(match.group(1))
            percent = int(match.group(2))
            mm_per_s = int(match.group(3)) if match.group(3) else 0

            self.odometer_mm = self.along_track_mm(t_us)
            self.odometer_t_us = t_us
            self.cart_mm_per_s = mm_per_s

            self.events.writerow([t_us, "drive", percent, mm_per_s])
            self.events_file.flush()
            # Stop-and-go toggles the motor every step; only announce when not in stations
            if self.station_count == 0 and self.open_station is None:
                if percent:
                    print(f">> cart DRIVING at {percent}% (~{mm_per_s} mm/s), at {self.odometer_mm / 1000:.2f} m")
                else:
                    print(f">> cart STOPPED at ~{self.odometer_mm / 1000:.2f} m along the track")
            return

        # Skip probe-rs location lines, show everything else (scan modes, warnings, flashing)
        stripped = line.strip()
        if stripped and not stripped.startswith("└─"):
            print(line.rstrip())

    def report_station(self):
        """Summarize the closed station: how far out to each side the lidar saw."""
        n, start_t, along, end_t = self.closing_station
        self.closing_station = None
        self.station_count += 1

        points = self.station_points
        left = [x for x, y in points if x < 0]
        right = [x for x, y in points if x > 0]
        left_edge = min(left) if left else 0
        right_edge = max(right) if right else 0
        left_zone = sum(1 for x in left if x < -RAIL_ZONE_MM)
        right_zone = sum(1 for x in right if x > RAIL_ZONE_MM)

        self.stations.writerow([
            n, start_t, end_t, along, self.station_revs, len(points),
            round(left_edge), round(right_edge), left_zone, right_zone,
        ])
        self.stations_file.flush()
        print(
            f"   station {n:4d} @ {along / 1000:6.2f} m: {self.station_revs} revs, {len(points)} pts | "
            f"widest L {left_edge:5.0f} R {right_edge:5.0f} mm | "
            f"rail-zone pts (|x|>{RAIL_ZONE_MM}) L {left_zone:4d} R {right_zone:4d}"
        )
        self.station_points = []

    def print_status(self):
        # Stop-and-go prints a line per station instead
        if self.station_count or self.open_station or self.closing_station:
            return
        now = time.monotonic()
        if now - self.last_status < 2 or self.window_start_t_us is None:
            return
        self.last_status = now

        seconds = (self.window_end_t_us - self.window_start_t_us) / 1e6
        if seconds > 0 and self.window_revs > 0:
            revs_per_s = self.window_revs / seconds
            points_per_rev = self.window_points / self.window_revs
            spacing_mm = abs(self.cart_mm_per_s) / revs_per_s
            print(
                f"   {self.point_count} pts | {self.window_points / seconds:.0f} pts/s, "
                f"{revs_per_s:.1f} rev/s, {points_per_rev:.0f} pts/rev ({360 / points_per_rev:.2f} deg apart) | "
                f"cart {self.last_along_mm / 1000:.2f} m, {spacing_mm:.0f} mm between slices | "
                f"{self.missing_chunks} chunks missing"
            )
        self.points_file.flush()
        self.window_points = 0
        self.window_revs = 0
        self.window_start_t_us = None

    def close(self):
        if self.closing_station:
            self.report_station()
        self.points_file.close()
        self.events_file.close()
        self.stations_file.close()
        if self.station_count:
            print(f"\nStations scanned: {self.station_count} (see stations.csv)")
        if any(self.points_per_mode):
            print("\nPoints per scan mode (scan_mode column in points.csv):")
            for mode, count in self.points_per_mode.items():
                print(f"   {mode or 'unlabeled'}: {count}")
        skipped = self.skipped_bytes + self.decoder.skipped_bytes
        print()
        print(f"Saved {self.point_count} points over {self.rev} revolutions")
        print(f"Cart moved about {self.last_along_mm / 1000:.2f} m (estimated from speed x time)")
        print(f"Chunks missing: {self.missing_chunks}   Bytes skipped while resyncing: {skipped}")
        print(f"Scan (re)starts: {self.scan_restarts}")


def make_out_folder():
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    folder = RUST_FOLDER / "scans" / stamp
    folder.mkdir(parents=True)
    return folder


def record_live():
    out_folder = make_out_folder()
    print(f"Saving to {out_folder}")
    print("The cart starts on its own a few seconds after boot. Blue button stops/starts it.")
    print("Ctrl+C ends the recording and stops the cart within ~3-5 s (watchdog). Blue button stops it instantly.")
    print()

    recorder = Recorder(out_folder)
    process = subprocess.Popen(
        ["cargo", "run", "--release", "-p", PACKAGE],
        cwd=RUST_FOLDER,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        bufsize=1,
    )

    with open(out_folder / "raw.log", "w", encoding="utf-8") as raw:
        try:
            for line in process.stdout:
                raw.write(line)
                recorder.handle_line(line)
        except KeyboardInterrupt:
            # Ctrl+C also reaches probe-rs; save whatever it still prints
            try:
                for line in process.stdout:
                    raw.write(line)
                    recorder.handle_line(line)
            except KeyboardInterrupt:
                pass
        finally:
            process.wait()
            recorder.close()
            print(f"Files are in {out_folder}")


def decode_saved(path):
    out_folder = make_out_folder()
    recorder = Recorder(out_folder)
    with open(path, encoding="utf-8", errors="replace") as raw:
        for line in raw:
            recorder.handle_line(line)
    recorder.close()
    print(f"Files are in {out_folder}")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        decode_saved(sys.argv[1])
    else:
        record_live()
