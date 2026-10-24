#!/usr/bin/env python3
"""
tof_localizer.py - particle-filter localization from three VL6180X ranges and the course map.

Frame: origin at course point (row 1, col 1), x increases with rows, y with cols,
heading = atan2(y, x), same as sim/lidar_sim.py and the 3pi+ odometry frame.

Library:
    walls = load_walls("course.json")
    pf = ParticleFilter(walls, cfg)
    pf.predict(d_dist_m, d_yaw_deg)
    pf.update({"left": 0.05, "front": None, "right": 0.06})   # metres
    x, y, yaw_deg, conf = pf.estimate()

LocalizationNode runs this on a thread: it reads ODOM/SET lines from the UART, polls
the sensors, and sends LOC/TOF lines back (see POSE_PROTOCOL.md).

Self-test (no hardware):  python3 tof_localizer.py --selftest
"""
import json
import math
import threading
import time
from pathlib import Path
from typing import Dict, NamedTuple, Optional

import numpy as np

IN_TO_M = 0.0254


def _cumulative(lengths):
    out = [0.0]
    for v in lengths:
        out.append(out[-1] + float(v))
    return out


def load_walls(course_path: str) -> np.ndarray:
    """Return wall segments as an (N, 4) array of x1, y1, x2, y2 in metres."""
    with open(course_path, "r") as f:
        data = json.load(f)
    course = data["course"]
    rows = _cumulative(course["row_heights_in"])
    cols = _cumulative(course["col_widths_in"])
    ox, oy = rows[1], cols[1]

    segs = []
    for w in data.get("horizontal_walls", []):
        r, c = int(w["row"]), int(w["col"])
        segs.append((rows[r] - ox, cols[c] - oy, rows[r] - ox, cols[c + 1] - oy))
    for w in data.get("vertical_walls", []):
        r, c = int(w["row"]), int(w["col"])
        segs.append((rows[r] - ox, cols[c] - oy, rows[r + 1] - ox, cols[c] - oy))
    return np.array(segs, dtype=np.float64) * IN_TO_M


def cast_rays(walls: np.ndarray, ox, oy, ang, max_range: float) -> np.ndarray:
    """Distance from each ray origin along angle to the nearest wall, capped at max_range."""
    dx = np.cos(ang)[:, None]
    dy = np.sin(ang)[:, None]
    x1, y1, x2, y2 = walls.T
    ex = (x2 - x1)[None, :]
    ey = (y2 - y1)[None, :]
    px = x1[None, :] - ox[:, None]
    py = y1[None, :] - oy[:, None]

    denom = dx * ey - dy * ex
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (px * ey - py * ex) / denom
        s = (px * dy - py * dx) / denom
    hit = (np.abs(denom) > 1e-9) & (t > 1e-4) & (s >= 0.0) & (s <= 1.0)
    t = np.where(hit, t, np.inf)
    return np.minimum(t.min(axis=1), max_range)


def _wrap_pi(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


class Mount(NamedTuple):
    x: float            # m, robot frame
    y: float
    angle: float        # rad, relative to robot heading
    max_range: float    # m
    sigma: float        # m
    beam_half: float    # rad; >0 casts 3 rays and keeps the nearest hit (wide-FoV sensors)


class ParticleFilter:
    def __init__(self, walls: np.ndarray, cfg: dict, rng: Optional[np.random.Generator] = None):
        self.walls = walls
        self.rng = rng or np.random.default_rng()
        self.n = int(cfg.get("particles", 150))
        self.sigma_range = float(cfg.get("sigma_range_m", 0.015))
        self.floor = float(cfg.get("outlier_floor", 0.05))
        self.max_range = float(cfg.get("max_valid_mm", 180)) / 1000.0
        self.motion = dict(cfg.get("motion", {}))
        self.sensors = {
            name: Mount(
                float(s["x_m"]), float(s["y_m"]), math.radians(float(s["angle_deg"])),
                float(s.get("max_valid_mm", cfg.get("max_valid_mm", 180))) / 1000.0,
                float(s.get("sigma_range_m", self.sigma_range)),
                math.radians(float(s.get("beam_half_angle_deg", 0.0))),
            )
            for name, s in cfg["sensors"].items()
        }
        self.x = np.zeros(self.n)
        self.y = np.zeros(self.n)
        self.yaw = np.zeros(self.n)
        self.w = np.full(self.n, 1.0 / self.n)
        ip = cfg.get("initial_pose", {})
        self.reset(float(ip.get("x_m", 0.0)), float(ip.get("y_m", 0.0)),
                   float(ip.get("yaw_deg", 0.0)),
                   float(ip.get("sigma_xy_m", 0.02)), float(ip.get("sigma_yaw_deg", 3.0)))

    def reset(self, x: float, y: float, yaw_deg: float, sigma_xy: float, sigma_yaw_deg: float) -> None:
        self.x = x + self.rng.normal(0.0, sigma_xy, self.n)
        self.y = y + self.rng.normal(0.0, sigma_xy, self.n)
        self.yaw = math.radians(yaw_deg) + self.rng.normal(0.0, math.radians(sigma_yaw_deg), self.n)
        self.w = np.full(self.n, 1.0 / self.n)

    def predict(self, d_dist_m: float, d_yaw_deg: float, missed: int = 0) -> None:
        m = self.motion
        d_yaw = math.radians(d_yaw_deg)
        sd_t = m.get("trans_min_m", 0.0005) + m.get("trans_per_m", 0.05) * abs(d_dist_m)
        sd_r = (m.get("rot_min_rad", 0.0005) + m.get("rot_per_rad", 0.05) * abs(d_yaw)
                + m.get("rot_per_m", 0.05) * abs(d_dist_m))

        dd = d_dist_m + self.rng.normal(0.0, sd_t, self.n)
        dyaw = d_yaw + self.rng.normal(0.0, sd_r, self.n)
        mid = self.yaw + 0.5 * dyaw
        self.x += dd * np.cos(mid)
        self.y += dd * np.sin(mid)
        self.yaw = _wrap_pi(self.yaw + dyaw)

        if missed > 0:
            self.x += self.rng.normal(0.0, missed * m.get("missed_msg_xy_m", 0.02), self.n)
            self.y += self.rng.normal(0.0, missed * m.get("missed_msg_xy_m", 0.02), self.n)
            self.yaw = _wrap_pi(self.yaw + self.rng.normal(
                0.0, math.radians(missed * m.get("missed_msg_yaw_deg", 3.0)), self.n))

    def expected_range(self, ox, oy, yaw, m: Mount) -> np.ndarray:
        """Expected reading for sensor origins (ox, oy) and robot headings yaw."""
        expected = cast_rays(self.walls, ox, oy, yaw + m.angle, m.max_range)
        if m.beam_half > 0.0:
            for off in (-m.beam_half, m.beam_half):
                expected = np.minimum(
                    expected, cast_rays(self.walls, ox, oy, yaw + m.angle + off, m.max_range))
        return expected

    def update(self, readings_m: Dict[str, Optional[float]]) -> bool:
        """Weight particles by the valid readings (metres). Returns False if none were valid."""
        valid = [(n, z) for n, z in readings_m.items() if z is not None and n in self.sensors]
        if not valid:
            return False

        c = np.cos(self.yaw)
        s = np.sin(self.yaw)
        logw = np.zeros(self.n)
        for name, z in valid:
            m = self.sensors[name]
            ox = self.x + m.x * c - m.y * s
            oy = self.y + m.x * s + m.y * c
            expected = self.expected_range(ox, oy, self.yaw, m)
            err = (min(z, m.max_range) - expected) / m.sigma
            logw += np.log(self.floor + (1.0 - self.floor) * np.exp(-0.5 * err * err))

        w = self.w * np.exp(logw - logw.max())
        total = w.sum()
        if not np.isfinite(total) or total <= 0.0:
            return False
        self.w = w / total

        if 1.0 / np.sum(self.w ** 2) < 0.5 * self.n:
            self._resample()
        return True

    def _resample(self) -> None:
        positions = (self.rng.random() + np.arange(self.n)) / self.n
        idx = np.minimum(np.searchsorted(np.cumsum(self.w), positions), self.n - 1)
        self.x = self.x[idx] + self.rng.normal(0.0, 0.002, self.n)
        self.y = self.y[idx] + self.rng.normal(0.0, 0.002, self.n)
        self.yaw = _wrap_pi(self.yaw[idx] + self.rng.normal(0.0, math.radians(0.5), self.n))
        self.w = np.full(self.n, 1.0 / self.n)

    def estimate(self):
        """Return (x_m, y_m, yaw_deg, confidence 0..1); confidence reflects particle spread only."""
        w = self.w
        mx = float(np.sum(w * self.x))
        my = float(np.sum(w * self.y))
        sc = float(np.sum(w * np.cos(self.yaw)))
        ss = float(np.sum(w * np.sin(self.yaw)))
        yaw = math.atan2(ss, sc)
        std_xy = math.sqrt(float(np.sum(w * ((self.x - mx) ** 2 + (self.y - my) ** 2))))
        r = min(1.0, max(1e-6, math.hypot(sc, ss)))
        std_yaw = math.sqrt(-2.0 * math.log(r))
        conf = 1.0 / (1.0 + (std_xy / 0.03) ** 2 + (std_yaw / 0.15) ** 2)
        return mx, my, math.degrees(yaw), conf


class LocalizationNode(threading.Thread):
    """Reads ODOM/SET from the UART, polls the ToF array, sends LOC and TOF lines."""

    def __init__(self, config_path: str, uart=None, uart_lock: Optional[threading.Lock] = None,
                 verbose: bool = False):
        super().__init__(daemon=True, name="tof-localizer")
        config_path = str(config_path)
        with open(config_path, "r") as f:
            self.cfg = json.load(f)
        base = Path(config_path).resolve().parent
        self.walls = load_walls(str(base / self.cfg.get("course_file", "course.json")))
        self.uart = uart
        self.lock = uart_lock or threading.Lock()
        self.verbose = verbose
        self._stop_evt = threading.Event()

    def stop(self) -> None:
        self._stop_evt.set()

    def _send(self, line: str) -> None:
        if self.uart is None:
            return
        try:
            with self.lock:
                self.uart.write(line.encode("ascii"))
        except Exception as e:
            if self.verbose:
                print(f"  LOC UART send error: {e}")

    def _handle_line(self, line: str, pf: ParticleFilter) -> None:
        parts = line.split(",")
        try:
            if parts[0] == "ODOM" and len(parts) == 5:
                seq = int(parts[1])
                if self.last_seq is not None:
                    missed = ((seq - self.last_seq) & 0xFFFF) - 1
                    if missed > 100:
                        missed = 0   # seq went backwards (3pi+ restart)
                else:
                    missed = 0
                self.last_seq = seq
                pf.predict(float(parts[3]), float(parts[4]), missed)
            elif parts[0] == "SET" and len(parts) == 4:
                sp = self.cfg.get("set_pose_sigma", {})
                pf.reset(float(parts[1]), float(parts[2]), float(parts[3]),
                         float(sp.get("sigma_xy_m", 0.01)), float(sp.get("sigma_yaw_deg", 2.0)))
        except ValueError:
            pass

    def _drain_uart(self, pf: ParticleFilter) -> None:
        if self.uart is None:
            return
        try:
            n = self.uart.in_waiting
            if n <= 0:
                return
            self._rx.extend(self.uart.read(n))
        except Exception:
            return
        if len(self._rx) > 512:
            self._rx = self._rx[-128:]
        while True:
            idx = self._rx.find(b"\n")
            if idx < 0:
                break
            line = bytes(self._rx[:idx]).decode("ascii", "ignore").strip()
            del self._rx[:idx + 1]
            if line:
                self._handle_line(line, pf)

    def run(self) -> None:
        from vl6180_mux import ToFArray

        try:
            tof = ToFArray(self.cfg)
        except Exception as e:
            print(f"ToF localizer disabled: {e}")
            return

        pf = ParticleFilter(self.walls, self.cfg)
        self.last_seq = None
        self._rx = bytearray()
        loc_period_s = float(self.cfg.get("loc_period_ms", 100)) / 1000.0
        t0 = time.monotonic()
        next_send = t0
        latest: Dict[str, Optional[float]] = {}

        try:
            while not self._stop_evt.is_set():
                self._drain_uart(pf)

                try:
                    latest, new_sample = tof.poll()
                except Exception as e:
                    if self.verbose:
                        print(f"  ToF poll error: {e}")
                    new_sample = False
                if new_sample:
                    pf.update({k: (v / 1000.0 if v is not None else None) for k, v in latest.items()})   # inf stays inf

                now = time.monotonic()
                if now >= next_send:
                    next_send = now + loc_period_s
                    ts_ms = int((now - t0) * 1000)
                    x, y, yaw_deg, conf = pf.estimate()
                    seq = -1 if self.last_seq is None else self.last_seq

                    def mm(name):
                        v = latest.get(name)
                        if v is None:
                            return -1
                        return 255 if math.isinf(v) else int(round(v))

                    self._send(f"LOC,{seq},{ts_ms},{x:.4f},{y:.4f},{yaw_deg:.2f},{conf:.2f}\n")
                    self._send(f"TOF,{ts_ms},{mm('left')},{mm('front')},{mm('right')}\n")
                    if self.verbose:
                        print(f"  LOC x={x:+.3f} y={y:+.3f} yaw={yaw_deg:+6.1f} conf={conf:.2f} "
                              f"L={mm('left')} C={mm('front')} R={mm('right')}")
                time.sleep(0.002)
        finally:
            tof.close()


def _selftest(config_path: str) -> int:
    with open(config_path, "r") as f:
        cfg = json.load(f)
    walls = load_walls(str(Path(config_path).resolve().parent / cfg.get("course_file", "course.json")))
    rng = np.random.default_rng(1)
    pf = ParticleFilter(walls, cfg, rng)

    # Truth drives forward in 1 cm steps along a cell centre line where walls are in range.
    tx, ty, tyaw = 0.1, 0.267, 0.0
    pf.reset(tx + 0.02, ty - 0.015, 4.0, 0.02, 3.0)
    for step in range(60):
        step_m = 0.01
        tx += step_m * math.cos(tyaw)
        ty += step_m * math.sin(tyaw)
        pf.predict(step_m * 1.03, rng.normal(0.0, 0.3))   # odometry with scale error

        readings = {}
        for name, m in pf.sensors.items():
            ox = tx + m.x * math.cos(tyaw) - m.y * math.sin(tyaw)
            oy = ty + m.x * math.sin(tyaw) + m.y * math.cos(tyaw)
            d = float(pf.expected_range(np.array([ox]), np.array([oy]), np.array([tyaw]), m)[0])
            if rng.random() < 0.05:
                readings[name] = None
            else:
                readings[name] = math.inf if d >= m.max_range else d + rng.normal(0.0, 0.005)
        pf.update(readings)

    ex, ey, eyaw, conf = pf.estimate()
    err_xy = math.hypot(ex - tx, ey - ty)
    err_yaw = abs(eyaw - math.degrees(tyaw))
    print(f"walls={len(walls)} truth=({tx:.3f},{ty:.3f},{math.degrees(tyaw):.1f}) "
          f"est=({ex:.3f},{ey:.3f},{eyaw:.1f}) err_xy={err_xy * 100:.1f} cm "
          f"err_yaw={err_yaw:.1f} deg conf={conf:.2f}")
    return 0 if (err_xy < 0.03 and err_yaw < 5.0) else 1


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="ToF localizer self-test")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--config", default=str(Path(__file__).parent / "tof_config.json"))
    a = ap.parse_args()
    if a.selftest:
        raise SystemExit(_selftest(a.config))
    ap.print_help()
