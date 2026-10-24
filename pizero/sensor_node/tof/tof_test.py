#!/usr/bin/env python3
"""Print left / front / right VL6180X distances (mm)."""
import argparse
import signal
import time
from pathlib import Path

from vl6180_mux import ToFArray

RUNNING = True


def _stop(signum, frame):
    global RUNNING
    RUNNING = False


def fmt(v):
    if v is None:
        return "  -- "
    return "  inf" if v == float("inf") else f"{v:5.0f}"


def main() -> int:
    parser = argparse.ArgumentParser(description="VL6180X left/center/right distance test")
    parser.add_argument("--config", default=str(Path(__file__).parent / "tof_config.json"))
    parser.add_argument("--hz", type=float, default=10.0, help="print rate")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    tof = ToFArray.from_config_file(args.config)
    interval = 1.0 / max(0.1, args.hz)
    next_print = time.monotonic()
    latest = {}

    try:
        while RUNNING:
            latest, _ = tof.poll()
            now = time.monotonic()
            if now >= next_print:
                next_print = now + interval
                line = "  ".join(f"{name[:6]}{'' if tof.is_ok(name) else '!'}={fmt(latest.get(name))}"
                                 for name in tof.names)
                bad = [n for n in tof.names if not tof.is_ok(n)]
                print(line + ("  (! = not initialised)" if bad else "  (mm)"))
            time.sleep(0.002)
    finally:
        tof.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
