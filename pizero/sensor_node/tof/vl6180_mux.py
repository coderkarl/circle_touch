#!/usr/bin/env python3
"""
vl6180_mux.py - VL6180X and VL53L0X time-of-flight sensors behind a PCA9548A I2C mux (Pi Zero).

Raw smbus2 register drivers; the only dependency is smbus2.  Sensors run in
continuous ranging mode, so poll() never blocks on a measurement.  Each sensor in the
config has a "type" ("vl6180x" default, or "vl53l0x").

Typical use:
    tof = ToFArray.from_config_file("tof_config.json")
    readings, new_sample = tof.poll()     # {"left": mm|inf|None, "front": ..., ...}
"""
import json
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

VL6180_ADDR = 0x29
MUX_ADDR_DEFAULT = 0x70

REG_MODEL_ID = 0x000
REG_FRESH_OUT_OF_RESET = 0x016
REG_INTERRUPT_CONFIG = 0x014
REG_INTERRUPT_CLEAR = 0x015
REG_RANGE_START = 0x018
REG_INTERMEASUREMENT = 0x01B
REG_RANGE_STATUS = 0x04D
REG_INTERRUPT_STATUS = 0x04F
REG_RANGE_VAL = 0x062

# Mandatory private-register settings from the ST VL6180X application note AN4545.
_MANDATORY_SETTINGS = (
    (0x0207, 0x01), (0x0208, 0x01), (0x0096, 0x00), (0x0097, 0xFD),
    (0x00E3, 0x01), (0x00E4, 0x03), (0x00E5, 0x02), (0x00E6, 0x01),
    (0x00E7, 0x03), (0x00F5, 0x02), (0x00D9, 0x05), (0x00DB, 0xCE),
    (0x00DC, 0x03), (0x00DD, 0xF8), (0x009F, 0x00), (0x00A3, 0x3C),
    (0x00B7, 0x00), (0x00BB, 0x3C), (0x00B2, 0x09), (0x00CA, 0x09),
    (0x0198, 0x01), (0x01B0, 0x17), (0x01AD, 0x00), (0x00FF, 0x05),
    (0x0100, 0x05), (0x0199, 0x05), (0x01A6, 0x1B), (0x01AC, 0x3E),
    (0x01A7, 0x1F), (0x0030, 0x00),
)

# Range value reported when nothing is detected.
_NO_TARGET_MM = 255


class VL6180X:
    """One VL6180X on an already-selected bus/mux channel."""

    no_target_mm = _NO_TARGET_MM

    def __init__(self, bus, address: int = VL6180_ADDR):
        self.bus = bus
        self.address = address

    def _write(self, reg: int, value: int) -> None:
        from smbus2 import i2c_msg
        self.bus.i2c_rdwr(i2c_msg.write(self.address, [reg >> 8, reg & 0xFF, value & 0xFF]))

    def _read(self, reg: int) -> int:
        from smbus2 import i2c_msg
        wr = i2c_msg.write(self.address, [reg >> 8, reg & 0xFF])
        rd = i2c_msg.read(self.address, 1)
        self.bus.i2c_rdwr(wr, rd)
        return list(rd)[0]

    def _ensure_idle(self) -> None:
        # 0x01 stops continuous ranging left running by a previous process.
        self._write(REG_RANGE_START, 0x01)
        t0 = time.monotonic()
        while not (self._read(REG_RANGE_STATUS) & 0x01) and time.monotonic() - t0 < 0.3:
            time.sleep(0.005)
        self._write(REG_INTERRUPT_CLEAR, 0x07)

    def init(self, cfg: dict) -> None:
        period_ms = int(cfg.get("period_ms", 20))
        if self._read(REG_MODEL_ID) != 0xB4:
            raise IOError("VL6180X model ID mismatch")
        self._ensure_idle()
        if self._read(REG_FRESH_OUT_OF_RESET) == 1:
            for reg, val in _MANDATORY_SETTINGS:
                self._write(reg, val)
            self._write(REG_FRESH_OUT_OF_RESET, 0)

        self._write(0x010A, 0x30)   # readout averaging
        self._write(0x0031, 0xFF)   # auto-calibration period
        self._write(0x002E, 0x01)   # recalibrate temperature
        self._write(REG_INTERRUPT_CONFIG, 0x04)   # range new-sample-ready
        self._write(REG_INTERMEASUREMENT, max(0, min(254, period_ms // 10 - 1)))
        self._write(REG_INTERRUPT_CLEAR, 0x07)
        self._write(REG_RANGE_START, 0x03)   # start continuous ranging

    def read_ready(self) -> Optional[Tuple[int, int]]:
        """Return (range_mm, error_code) if a new sample is ready, else None."""
        if (self._read(REG_INTERRUPT_STATUS) & 0x07) != 0x04:
            return None
        status = self._read(REG_RANGE_STATUS) >> 4
        rng = self._read(REG_RANGE_VAL)
        self._write(REG_INTERRUPT_CLEAR, 0x07)
        return rng, status

    def stop(self) -> None:
        self._write(REG_RANGE_START, 0x01)


# VL53L0X register map (subset) and tuning, ported from the Pololu vl53l0x-arduino driver via
# Adafruit_CircuitPython_VL53L0X (MIT, Tony DiCola for Adafruit Industries).
VL53_SYSRANGE_START = 0x00
VL53_SEQUENCE_CONFIG = 0x01
VL53_INTERRUPT_CONFIG = 0x0A
VL53_INTERRUPT_CLEAR = 0x0B
VL53_INTERRUPT_STATUS = 0x13
VL53_RANGE_STATUS = 0x14
VL53_MSRC_CONFIG_CONTROL = 0x60
VL53_MSRC_CONFIG_TIMEOUT = 0x46
VL53_FINAL_MIN_COUNT_RATE = 0x44
VL53_PRE_VCSEL_PERIOD = 0x50
VL53_PRE_TIMEOUT_HI = 0x51
VL53_FINAL_VCSEL_PERIOD = 0x70
VL53_FINAL_TIMEOUT_HI = 0x71
VL53_GPIO_HV_MUX_ACTIVE_HIGH = 0x84

_VL53_TUNING = (
    (0xFF, 0x01), (0x00, 0x00), (0xFF, 0x00), (0x09, 0x00), (0x10, 0x00), (0x11, 0x00),
    (0x24, 0x01), (0x25, 0xFF), (0x75, 0x00), (0xFF, 0x01), (0x4E, 0x2C), (0x48, 0x00),
    (0x30, 0x20), (0xFF, 0x00), (0x30, 0x09), (0x54, 0x00), (0x31, 0x04), (0x32, 0x03),
    (0x40, 0x83), (0x46, 0x25), (0x60, 0x00), (0x27, 0x00), (0x50, 0x06), (0x51, 0x00),
    (0x52, 0x96), (0x56, 0x08), (0x57, 0x30), (0x61, 0x00), (0x62, 0x00), (0x64, 0x00),
    (0x65, 0x00), (0x66, 0xA0), (0xFF, 0x01), (0x22, 0x32), (0x47, 0x14), (0x49, 0xFF),
    (0x4A, 0x00), (0xFF, 0x00), (0x7A, 0x0A), (0x7B, 0x00), (0x78, 0x21), (0xFF, 0x01),
    (0x23, 0x34), (0x42, 0x00), (0x44, 0xFF), (0x45, 0x26), (0x46, 0x05), (0x40, 0x40),
    (0x0E, 0x06), (0x20, 0x1A), (0x43, 0x40), (0xFF, 0x00), (0x34, 0x03), (0x35, 0x44),
    (0xFF, 0x01), (0x31, 0x04), (0x4B, 0x09), (0x4C, 0x05), (0x4D, 0x04), (0xFF, 0x00),
    (0x44, 0x00), (0x45, 0x20), (0x47, 0x08), (0x48, 0x28), (0x67, 0x00), (0x70, 0x04),
    (0x71, 0x01), (0x72, 0xFE), (0x76, 0x00), (0x77, 0x00), (0xFF, 0x01), (0x0D, 0x01),
    (0xFF, 0x00), (0x80, 0x01), (0x01, 0xF8), (0xFF, 0x01), (0x8E, 0x01), (0x00, 0x01),
    (0xFF, 0x00), (0x80, 0x00),
)


def _vl53_decode_timeout(val: int) -> int:
    return ((val & 0xFF) << ((val >> 8) & 0xFF)) + 1


def _vl53_encode_timeout(mclks: int) -> int:
    mclks = int(mclks) & 0xFFFF
    if mclks <= 0:
        return 0
    ls_byte = mclks - 1
    ms_byte = 0
    while ls_byte > 255:
        ls_byte >>= 1
        ms_byte += 1
    return ((ms_byte << 8) | (ls_byte & 0xFF)) & 0xFFFF


def _vl53_mclks_to_us(mclks: int, vcsel_pclks: int) -> int:
    macro_ns = ((2304 * vcsel_pclks * 1655) + 500) // 1000
    return ((mclks * macro_ns) + (macro_ns // 2)) // 1000


def _vl53_us_to_mclks(us: int, vcsel_pclks: int) -> int:
    macro_ns = ((2304 * vcsel_pclks * 1655) + 500) // 1000
    return ((us * 1000) + (macro_ns // 2)) // macro_ns


class VL53L0X:
    """One VL53L0X on an already-selected bus/mux channel (back-to-back continuous ranging)."""

    # Range reported when nothing is detected (8190/8191).
    no_target_mm = 8190

    def __init__(self, bus, address: int = VL6180_ADDR):
        self.bus = bus
        self.address = address
        self._stop_variable = 0

    def _r8(self, reg: int) -> int:
        return self.bus.read_byte_data(self.address, reg)

    def _w8(self, reg: int, val: int) -> None:
        self.bus.write_byte_data(self.address, reg, val & 0xFF)

    def _r16(self, reg: int) -> int:
        hi, lo = self.bus.read_i2c_block_data(self.address, reg, 2)
        return (hi << 8) | lo

    def _w16(self, reg: int, val: int) -> None:
        self.bus.write_i2c_block_data(self.address, reg, [(val >> 8) & 0xFF, val & 0xFF])

    def _pairs(self, pairs) -> None:
        for reg, val in pairs:
            self._w8(reg, val)

    def _wait(self, done, what: str, timeout_s: float = 0.5) -> None:
        t0 = time.monotonic()
        while not done():
            if time.monotonic() - t0 > timeout_s:
                raise IOError(f"VL53L0X timeout: {what}")

    def _spad_info(self) -> Tuple[int, bool]:
        self._pairs(((0x80, 0x01), (0xFF, 0x01), (0x00, 0x00), (0xFF, 0x06)))
        self._w8(0x83, self._r8(0x83) | 0x04)
        self._pairs(((0xFF, 0x07), (0x81, 0x01), (0x80, 0x01), (0x94, 0x6B), (0x83, 0x00)))
        self._wait(lambda: self._r8(0x83) != 0x00, "SPAD info")
        self._w8(0x83, 0x01)
        tmp = self._r8(0x92)
        self._pairs(((0x81, 0x00), (0xFF, 0x06)))
        self._w8(0x83, self._r8(0x83) & ~0x04)
        self._pairs(((0xFF, 0x01), (0x00, 0x01), (0xFF, 0x00), (0x80, 0x00)))
        return tmp & 0x7F, ((tmp >> 7) & 0x01) == 1

    def _single_ref_calibration(self, vhv_init_byte: int) -> None:
        self._w8(VL53_SYSRANGE_START, 0x01 | vhv_init_byte)
        self._wait(lambda: (self._r8(VL53_INTERRUPT_STATUS) & 0x07) != 0, "ref calibration")
        self._w8(VL53_INTERRUPT_CLEAR, 0x01)
        self._w8(VL53_SYSRANGE_START, 0x00)

    def _vcsel_period(self, reg: int) -> int:
        return ((self._r8(reg) + 1) & 0xFF) << 1

    def _step_enables(self) -> Tuple[bool, bool, bool, bool, bool]:
        sc = self._r8(VL53_SEQUENCE_CONFIG)
        # tcc, dss, msrc, pre_range, final_range
        return (bool((sc >> 4) & 1), bool((sc >> 3) & 1), bool((sc >> 2) & 1),
                bool((sc >> 6) & 1), bool((sc >> 7) & 1))

    def _step_timeouts(self, pre_range: bool) -> Tuple[int, int, int, int, int]:
        pre_pclks = self._vcsel_period(VL53_PRE_VCSEL_PERIOD)
        msrc_mclks = (self._r8(VL53_MSRC_CONFIG_TIMEOUT) + 1) & 0xFF
        msrc_us = _vl53_mclks_to_us(msrc_mclks, pre_pclks)
        pre_mclks = _vl53_decode_timeout(self._r16(VL53_PRE_TIMEOUT_HI))
        pre_us = _vl53_mclks_to_us(pre_mclks, pre_pclks)
        final_pclks = self._vcsel_period(VL53_FINAL_VCSEL_PERIOD)
        final_mclks = _vl53_decode_timeout(self._r16(VL53_FINAL_TIMEOUT_HI))
        if pre_range:
            final_mclks -= pre_mclks
        final_us = _vl53_mclks_to_us(final_mclks, final_pclks)
        return msrc_us, pre_us, final_us, final_pclks, pre_mclks

    def _get_budget_us(self) -> int:
        budget = 1910 + 960
        tcc, dss, msrc, pre_range, final_range = self._step_enables()
        msrc_us, pre_us, final_us, _, _ = self._step_timeouts(pre_range)
        if tcc:
            budget += msrc_us + 590
        if dss:
            budget += 2 * (msrc_us + 690)
        elif msrc:
            budget += msrc_us + 660
        if pre_range:
            budget += pre_us + 660
        if final_range:
            budget += final_us + 550
        return budget

    def _set_budget_us(self, budget_us: int) -> None:
        used = 1320 + 960
        tcc, dss, msrc, pre_range, final_range = self._step_enables()
        msrc_us, pre_us, _, final_pclks, pre_mclks = self._step_timeouts(pre_range)
        if tcc:
            used += msrc_us + 590
        if dss:
            used += 2 * (msrc_us + 690)
        elif msrc:
            used += msrc_us + 660
        if pre_range:
            used += pre_us + 660
        if final_range:
            used += 550
            if used > budget_us:
                raise ValueError("VL53L0X timing budget too small")
            final_mclks = _vl53_us_to_mclks(budget_us - used, final_pclks)
            if pre_range:
                final_mclks += pre_mclks
            self._w16(VL53_FINAL_TIMEOUT_HI, _vl53_encode_timeout(final_mclks))

    def _soft_reset(self) -> None:
        # A device left mid-ranging (killed process, I2C glitch) will not re-init without this.
        for val in (0x00, 0x01):
            try:
                self._w8(0xBF, val)
            except OSError:
                pass   # device may NACK while resetting
            time.sleep(0.005)
        self._wait(lambda: self._safe_id() == 0xEE, "reset")

    def _safe_id(self) -> int:
        try:
            return self._r8(0xC0)
        except OSError:
            return 0

    def init(self, cfg: dict) -> None:
        self._soft_reset()
        if (self._r8(0xC0), self._r8(0xC1), self._r8(0xC2)) != (0xEE, 0xAA, 0x10):
            raise IOError("VL53L0X ID mismatch")

        self._pairs(((0x88, 0x00), (0x80, 0x01), (0xFF, 0x01), (0x00, 0x00)))
        self._stop_variable = self._r8(0x91)
        self._pairs(((0x00, 0x01), (0xFF, 0x00), (0x80, 0x00)))

        self._w8(VL53_MSRC_CONFIG_CONTROL, self._r8(VL53_MSRC_CONFIG_CONTROL) | 0x12)
        self._w16(VL53_FINAL_MIN_COUNT_RATE, int(0.25 * (1 << 7)))   # 0.25 MCPS
        self._w8(VL53_SEQUENCE_CONFIG, 0xFF)

        spad_count, is_aperture = self._spad_info()
        ref_map = bytearray(self.bus.read_i2c_block_data(self.address, 0xB0, 6))
        self._pairs(((0xFF, 0x01), (0x4F, 0x00), (0x4E, 0x2C), (0xFF, 0x00), (0xB6, 0xB4)))
        first_to_enable = 12 if is_aperture else 0
        enabled = 0
        for i in range(48):
            if i < first_to_enable or enabled == spad_count:
                ref_map[i // 8] &= ~(1 << (i % 8)) & 0xFF
            elif (ref_map[i // 8] >> (i % 8)) & 0x1:
                enabled += 1
        self.bus.write_i2c_block_data(self.address, 0xB0, list(ref_map))

        self._pairs(_VL53_TUNING)

        self._w8(VL53_INTERRUPT_CONFIG, 0x04)   # new sample ready
        self._w8(VL53_GPIO_HV_MUX_ACTIVE_HIGH, self._r8(VL53_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10)
        self._w8(VL53_INTERRUPT_CLEAR, 0x01)

        budget_us = self._get_budget_us()
        self._w8(VL53_SEQUENCE_CONFIG, 0xE8)
        self._set_budget_us(budget_us)
        self._w8(VL53_SEQUENCE_CONFIG, 0x01)
        self._single_ref_calibration(0x40)
        self._w8(VL53_SEQUENCE_CONFIG, 0x02)
        self._single_ref_calibration(0x00)
        self._w8(VL53_SEQUENCE_CONFIG, 0xE8)
        self._set_budget_us(int(cfg.get("timing_budget_us", 33000)))

        self._pairs(((0x80, 0x01), (0xFF, 0x01), (0x00, 0x00), (0x91, self._stop_variable),
                     (0x00, 0x01), (0xFF, 0x00), (0x80, 0x00), (VL53_SYSRANGE_START, 0x02)))
        self._wait(lambda: (self._r8(VL53_SYSRANGE_START) & 0x01) == 0, "start continuous")

    def read_ready(self) -> Optional[Tuple[int, int]]:
        """Return (range_mm, 0) if a new sample is ready, else None."""
        if (self._r8(VL53_INTERRUPT_STATUS) & 0x07) == 0:
            return None
        rng = self._r16(VL53_RANGE_STATUS + 10)
        self._w8(VL53_INTERRUPT_CLEAR, 0x01)
        return rng, 0

    def stop(self) -> None:
        self._pairs(((VL53_SYSRANGE_START, 0x01), (0xFF, 0x01), (0x00, 0x00), (0x91, 0x00),
                     (0x00, 0x01), (0xFF, 0x00)))


_DEVICE_TYPES = {"vl6180x": VL6180X, "vl53l0x": VL53L0X}


class ToFArray:
    """Named ToF sensors (VL6180X / VL53L0X) on PCA9548A mux channels."""

    def __init__(self, config: dict, bus=None):
        if bus is None:
            from smbus2 import SMBus
            bus = SMBus(int(config.get("i2c_bus", 1)))
        self.bus = bus
        self.mux_addr = int(config.get("mux_address", MUX_ADDR_DEFAULT))
        self.stale_s = float(config.get("stale_s", 0.2))
        self.watchdog_s = float(config.get("watchdog_s", 1.0))
        self.init_retries = int(config.get("init_retries", 3))
        default_max_mm = float(config.get("max_valid_mm", 180))

        self._cfg = config
        self._channels: Dict[str, int] = {}
        self._offsets_mm: Dict[str, float] = {}
        self._max_mm: Dict[str, float] = {}
        self._min_mm: Dict[str, float] = {}
        self._dev: Dict[str, object] = {}
        self._ok: Dict[str, bool] = {}
        self._last_sample: Dict[str, float] = {}
        self._last_init_try: Dict[str, float] = {}
        self.reinit_count: Dict[str, int] = {}
        self._latest: Dict[str, Tuple[Optional[float], float]] = {}
        self._current_channel: Optional[int] = None

        for name, s in config["sensors"].items():
            self._channels[name] = int(s["channel"])
            self._offsets_mm[name] = float(s.get("offset_mm", 0.0))
            self._max_mm[name] = float(s.get("max_valid_mm", default_max_mm))
            self._min_mm[name] = float(s.get("min_valid_mm", 0.0))
            self._latest[name] = (None, 0.0)
            dev_type = str(s.get("type", "vl6180x")).lower()
            if dev_type not in _DEVICE_TYPES:
                raise ValueError(f"ToF '{name}': unknown type '{dev_type}'")
            self._dev[name] = _DEVICE_TYPES[dev_type](bus)
            self.reinit_count[name] = 0
            self._init_sensor(name)

    def _init_sensor(self, name: str) -> bool:
        """(Re)initialise one sensor with retries; updates and returns its health flag."""
        s = self._cfg["sensors"][name]
        self._last_init_try[name] = time.monotonic()
        self._ok[name] = False
        for attempt in range(1, self.init_retries + 1):
            try:
                self._current_channel = None
                self._select(self._channels[name])
                self._dev[name].init({**self._cfg, **s})
                self._ok[name] = True
                self._last_sample[name] = time.monotonic()
                return True
            except (OSError, ValueError) as e:
                print(f"ToF '{name}' (mux ch {self._channels[name]}) init attempt "
                      f"{attempt}/{self.init_retries} failed: {e}")
                time.sleep(0.05)
        return False

    @classmethod
    def from_config_file(cls, path: str, bus=None) -> "ToFArray":
        with open(path, "r") as f:
            return cls(json.load(f), bus)

    @property
    def names(self) -> List[str]:
        return list(self._channels.keys())

    def is_ok(self, name: str) -> bool:
        return self._ok.get(name, False)

    def _select(self, channel: int) -> None:
        if channel != self._current_channel:
            self.bus.write_byte(self.mux_addr, 1 << channel)
            self._current_channel = channel

    def poll(self) -> Tuple[Dict[str, Optional[float]], bool]:
        """
        Read any ready samples and return ({name: mm}, new_sample).
        mm is a float distance, inf for no target within the sensor's valid range, or
        None for a sensor error / too-close / stale reading.
        """
        now = time.monotonic()
        new_sample = False
        for name, dev in self._dev.items():
            if not self._ok[name]:
                if now - self._last_init_try[name] >= 5.0:
                    self.reinit_count[name] += 1
                    if self._init_sensor(name):
                        print(f"ToF '{name}' recovered")
                continue
            if now - self._last_sample[name] > self.watchdog_s:
                print(f"ToF '{name}' no data for {self.watchdog_s:.1f} s, re-initialising")
                self.reinit_count[name] += 1
                self._init_sensor(name)
                continue
            try:
                self._select(self._channels[name])
                res = dev.read_ready()
            except OSError:
                self._current_channel = None
                continue
            if res is None:
                continue
            self._last_sample[name] = now
            rng, status = res
            new_sample = True
            if status != 0:
                value = None
            elif rng >= dev.no_target_mm:
                value = float("inf")
            else:
                value = rng + self._offsets_mm[name]
                if value > self._max_mm[name]:
                    value = float("inf")
                elif value < self._min_mm[name] or value <= 0.0:
                    value = None
            self._latest[name] = (value, now)

        out: Dict[str, Optional[float]] = {}
        for name in self._channels:
            value, ts = self._latest[name]
            out[name] = value if (now - ts) <= self.stale_s else None
        return out, new_sample

    def close(self) -> None:
        for name, dev in self._dev.items():
            if not self._ok[name]:
                continue
            try:
                self._select(self._channels[name])
                dev.stop()
            except OSError:
                pass
        try:
            self.bus.write_byte(self.mux_addr, 0)
            self.bus.close()
        except OSError:
            pass
