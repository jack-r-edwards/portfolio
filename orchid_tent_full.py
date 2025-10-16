
"""
orchid_tent_full.py — Self-regulating orchid tent controller with ready-to-use I2C adapters
Sensors:
  • Temp/RH: SHT31 (I2C, default addr 0x44)
  • Lux:     BH1750 (I2C, default addr 0x23)

Notes
-----
• Requires I2C enabled (e.g., on Raspberry Pi: raspi-config → Interface Options → I2C)
• Python dependency: smbus2  (pip install smbus2)
• GPIO relays optional; simulation mode works without any hardware.
• If only one sensor is connected, the other can be simulated (see --simulate-lux / --simulate-env).

Quick Start
-----------
Simulation:
  python3 orchid_tent_full.py --simulate

Real I2C sensors + simulated relays:
  python3 orchid_tent_full.py --i2c --simulate-relays

Real I2C sensors + GPIO relays (example pins inside build_gpio_controller):
  python3 orchid_tent_full.py --i2c

CLI Options (see --help for all):
  --i2c                 Use SHT31 + BH1750 hardware sensors
  --i2c-bus 1           I2C bus number (default 1 on most Pi)
  --sht31-addr 0x44     SHT31 address (0x44 or 0x45 are common)
  --bh1750-addr 0x23    BH1750 address (0x23 or 0x5C)
  --simulate            Force full simulation for everything
  --simulate-relays     Use simulated switches even if GPIO is available
  --simulate-lux        Simulate lux values even when using SHT31 for temp/RH
  --once                Run a single control step then exit

"""

from __future__ import annotations
import argparse
import csv
import os
import random
import signal
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timedelta, time as dtime
from typing import Optional, Protocol, Tuple

# ========= Configuration =========

LOG_PATH = os.environ.get("ORCHID_LOG_PATH", "orchid_tent_log.csv")

# Photoperiod (hours of light per 24h)
PHOTOPERIOD_HOURS = 12
LIGHTS_ON_AT_LOCAL = dtime(hour=8, minute=0)   # 08:00 local

# Targets — starter values (tune for species)
TARGETS = {
    "day":   {"temp_c": 24.0, "rh_pct": 65.0, "lux_min": 10000},
    "night": {"temp_c": 20.0, "rh_pct": 70.0, "lux_min": 0},
}

# Hysteresis + timing
HYSTERESIS = {
    "temp_c": 0.5,
    "rh_pct": 2.0,
    "lux": 500,
    "min_on_s": 60,
    "min_off_s": 60,
}

ENABLE_VPD_NUDGE = True
VPD_NUDGE_LIMIT = 4.0
LOOP_PERIOD_S = 10

# ========= Abstractions =========

class Sensor(Protocol):
    def read(self) -> Tuple[float, float, float]:
        """Return (temp_c, rh_pct, lux)"""
        ...

class Switch(Protocol):
    name: str
    def on(self) -> None: ...
    def off(self) -> None: ...
    def is_on(self) -> bool: ...
    def safe_name(self) -> str: ...

@dataclass
class TimedSwitch:
    inner: Switch
    min_on_s: int = HYSTERESIS["min_on_s"]
    min_off_s: int = HYSTERESIS["min_off_s"]
    _last_changed: float = field(default_factory=lambda: 0.0)

    def _time_since_change(self) -> float:
        return time.time() - self._last_changed

    def can_turn_on(self) -> bool:
        if self.inner.is_on():
            return True
        return self._time_since_change() >= self.min_off_s

    def can_turn_off(self) -> bool:
        if not self.inner.is_on():
            return True
        return self._time_since_change() >= self.min_on_s

    def on(self) -> None:
        if self.can_turn_on():
            self.inner.on()
            self._last_changed = time.time()

    def off(self) -> None:
        if self.can_turn_off():
            self.inner.off()
            self._last_changed = time.time()

    def is_on(self) -> bool:
        return self.inner.is_on()

    def safe_name(self) -> str:
        return self.inner.safe_name()

    @property
    def name(self) -> str:
        return self.inner.name

# ========= Simulation Adapters =========

class SimSensor:
    def __init__(self):
        self.temp_c = 23.0
        self.rh_pct = 60.0
        self.lux = 0.0

    def read(self) -> Tuple[float, float, float]:
        self.temp_c += random.uniform(-0.05, 0.05)
        self.rh_pct += random.uniform(-0.3, 0.3)
        return round(self.temp_c, 2), round(self.rh_pct, 1), round(self.lux, 0)

class SimSwitch:
    def __init__(self, name: str, sensor: Optional[SimSensor] = None, effect: str = ""):
        self.name = name
        self._on = False
        self._sensor = sensor
        self._effect = effect

    def on(self) -> None: self._on = True
    def off(self) -> None: self._on = False
    def is_on(self) -> bool: return self._on
    def safe_name(self) -> str: return f"SIM::{self.name}"

# ========= Hardware Adapters (GPIO relays) =========

HAVE_GPIO = False
try:
    import RPi.GPIO as GPIO  # type: ignore
    HAVE_GPIO = True
except Exception:
    pass

class GPIOSwitch:
    def __init__(self, name: str, pin: int, active_high: bool = False):
        if not HAVE_GPIO:
            raise RuntimeError("GPIO not available.")
        self.name = name
        self.pin = pin
        self.active_high = active_high
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.output(self.pin, GPIO.HIGH if not self.active_high else GPIO.LOW)

    def on(self) -> None:
        GPIO.output(self.pin, GPIO.HIGH if self.active_high else GPIO.LOW)

    def off(self) -> None:
        GPIO.output(self.pin, GPIO.LOW if self.active_high else GPIO.HIGH)

    def is_on(self) -> bool:
        state = GPIO.input(self.pin)
        return bool(state) if self.active_high else not bool(state)

    def safe_name(self) -> str:
        return f"GPIO(pin={self.pin})::{self.name}"

# ========= I2C Sensor Adapters (SHT31 + BH1750) =========

HAVE_SMBUS = False
try:
    from smbus2 import SMBus, i2c_msg  # type: ignore
    HAVE_SMBUS = True
except Exception:
    pass

class SHT31:
    def __init__(self, bus: int = 1, address: int = 0x44):
        if not HAVE_SMBUS:
            raise RuntimeError("smbus2 not available. Install with: pip install smbus2")
        self.busno = bus
        self.addr = address
        self.bus = SMBus(self.busno)
        try:
            self._write_cmd(0x30A2)  # soft reset
            time.sleep(0.01)
        except Exception:
            pass

    def _write_cmd(self, cmd: int):
        msb = (cmd >> 8) & 0xFF
        lsb = cmd & 0xFF
        self.bus.write_i2c_block_data(self.addr, msb, [lsb])

    def read_temp_rh(self) -> Tuple[float, float]:
        self._write_cmd(0x2400)  # single shot, high repeatability
        time.sleep(0.015)
        data = self.bus.read_i2c_block_data(self.addr, 0x00, 6)
        raw_t = (data[0] << 8) | data[1]
        raw_rh = (data[3] << 8) | data[4]
        temp_c = -45.0 + 175.0 * (raw_t / 65535.0)
        rh = 100.0 * (raw_rh / 65535.0)
        rh = max(0.0, min(100.0, rh))
        return temp_c, rh

class BH1750:
    CONT_H_RES_MODE = 0x10
    def __init__(self, bus: int = 1, address: int = 0x23):
        if not HAVE_SMBUS:
            raise RuntimeError("smbus2 not available. Install with: pip install smbus2")
        self.busno = bus
        self.addr = address
        self.bus = SMBus(self.busno)
        self.bus.write_byte(self.addr, self.CONT_H_RES_MODE)
        time.sleep(0.2)

    def read_lux(self) -> float:
        data = self.bus.read_i2c_block_data(self.addr, 0x00, 2)
        raw = (data[0] << 8) | data[1]
        lux = raw / 1.2
        return float(lux)

class CompositeRealSensor:
    def __init__(self, sht31: SHT31, bh1750: Optional[BH1750] = None, simulate_lux: bool = False):
        self.sht31 = sht31
        self.bh1750 = bh1750
        self.simulate_lux = simulate_lux
        self._sim_lux = 0.0

    def read(self) -> Tuple[float, float, float]:
        temp_c, rh = self.sht31.read_temp_rh()
        if self.simulate_lux or self.bh1750 is None:
            return round(temp_c, 2), round(rh, 1), round(self._sim_lux, 0)
        lux = self.bh1750.read_lux()
        return round(temp_c, 2), round(rh, 1), round(lux, 0)

    def set_sim_lux(self, lux: float) -> None:
        self._sim_lux = lux

# ========= Utilities =========

def is_daytime(now: datetime) -> bool:
    on = datetime.combine(now.date(), LIGHTS_ON_AT_LOCAL)
    off = on + timedelta(hours=PHOTOPERIOD_HOURS)
    return on <= now < off

def vpd_nudge_rh_target(temp_c: float, base_rh: float) -> float:
    if not ENABLE_VPD_NUDGE:
        return base_rh
    delta = temp_c - 24.0
    nudge = max(-VPD_NUDGE_LIMIT, min(VPD_NUDGE_LIMIT, delta * 1.5))
    rh = base_rh + nudge
    return max(40.0, min(85.0, rh))

# ========= Controller =========

@dataclass
class EnvironmentReading:
    timestamp: datetime
    temp_c: float
    rh_pct: float
    lux: float
    mode: str
    target_temp_c: float
    target_rh_pct: float
    target_lux_min: float

@dataclass
class OrchidTentController:
    sensor: Sensor
    light: TimedSwitch
    humidifier: TimedSwitch
    heater: TimedSwitch
    fan: TimedSwitch
    simulate_env: bool = True
    sensor_hook: Optional[CompositeRealSensor] = None

    def _current_targets(self, now: datetime) -> Tuple[str, float, float, float]:
        mode = "day" if is_daytime(now) else "night"
        t = TARGETS[mode]
        nudged_rh = vpd_nudge_rh_target(t["temp_c"], t["rh_pct"])
        return mode, t["temp_c"], nudged_rh, t["lux_min"]

    def step(self) -> EnvironmentReading:
        now = datetime.now()
        mode, tgt_temp, tgt_rh, tgt_lux = self._current_targets(now)
        temp_c, rh_pct, lux = self.sensor.read()

        # LIGHT
        if mode == "day":
            if lux < (tgt_lux - HYSTERESIS["lux"]):
                self.light.on()
            elif lux > (tgt_lux + HYSTERESIS["lux"]):
                self.light.off()
        else:
            self.light.off()

        # HEATER
        if temp_c < (tgt_temp - HYSTERESIS["temp_c"]):
            self.heater.on()
        elif temp_c > (tgt_temp + HYSTERESIS["temp_c"]):
            self.heater.off()

        # HUMIDIFIER + FAN
        if rh_pct < (tgt_rh - HYSTERESIS["rh_pct"]):
            self.humidifier.on()
            self.fan.on()
        elif rh_pct > (tgt_rh + HYSTERESIS["rh_pct"]):
            self.humidifier.off()
            self.fan.on()
        else:
            if not (self.heater.is_on() or self.humidifier.is_on() or self.light.is_on()):
                self.fan.off()

        # Simulation
        if self.simulate_env:
            if isinstance(self.sensor, SimSensor):
                if self.heater.is_on():
                    self.sensor.temp_c += random.uniform(0.03, 0.06)
                else:
                    self.sensor.temp_c += random.uniform(-0.02, 0.01)

                if self.humidifier.is_on():
                    self.sensor.rh_pct += random.uniform(0.4, 0.8)
                else:
                    self.sensor.rh_pct += random.uniform(-0.2, 0.1)

                if self.fan.is_on():
                    self.sensor.temp_c += random.uniform(-0.02, 0.0)
                    self.sensor.rh_pct += random.uniform(-0.3, 0.0)

                if self.light.is_on():
                    self.sensor.lux = max(self.sensor.lux, float(TARGETS["day"]["lux_min"]) + random.uniform(500, 1500))
                    self.sensor.temp_c += random.uniform(0.01, 0.03)
                else:
                    self.sensor.lux = max(0.0, self.sensor.lux - random.uniform(800, 1200))

                self.sensor.rh_pct = max(30.0, min(95.0, self.sensor.rh_pct))
                self.sensor.temp_c = max(10.0, min(35.0, self.sensor.temp_c))

            if isinstance(self.sensor_hook, CompositeRealSensor) and self.sensor_hook.simulate_lux:
                lux_val = 0.0
                if mode == "day" and self.light.is_on():
                    lux_val = float(TARGETS["day"]["lux_min"]) + random.uniform(500, 1500)
                self.sensor_hook.set_sim_lux(lux_val)

        return EnvironmentReading(
            timestamp=now,
            temp_c=temp_c,
            rh_pct=rh_pct,
            lux=lux,
            mode=mode,
            target_temp_c=tgt_temp,
            target_rh_pct=tgt_rh,
            target_lux_min=tgt_lux,
        )

    def log(self, reading: EnvironmentReading) -> None:
        file_exists = os.path.isfile(LOG_PATH)
        with open(LOG_PATH, "a", newline="") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow([
                    "timestamp",
                    "mode",
                    "temp_c", "target_temp_c",
                    "rh_pct", "target_rh_pct",
                    "lux", "target_lux_min",
                    "light_on", "humidifier_on", "heater_on", "fan_on",
                ])
            writer.writerow([
                reading.timestamp.isoformat(timespec="seconds"),
                reading.mode,
                f"{reading.temp_c:.2f}", f"{reading.target_temp_c:.2f}",
                f"{reading.rh_pct:.1f}", f"{reading.target_rh_pct:.1f}",
                f"{reading.lux:.0f}", f"{reading.target_lux_min:.0f}",
                int(self.light.is_on()), int(self.humidifier.is_on()),
                int(self.heater.is_on()), int(self.fan.is_on()),
            ])

    def pretty_print(self, reading: EnvironmentReading) -> None:
        def b(v: bool) -> str: return "ON " if v else "OFF"
        print(
            f"[{reading.timestamp.strftime('%H:%M:%S')}] "
            f"{reading.mode.upper()} | "
            f"T:{reading.temp_c:.1f}°C→{reading.target_temp_c:.1f}  "
            f"RH:{reading.rh_pct:.0f}%→{reading.target_rh_pct:.0f}  "
            f"LX:{reading.lux:.0f}→{reading.target_lux_min:.0f}  "
            f"| Light:{b(self.light.is_on())} Hum:{b(self.humidifier.is_on())} "
            f"Heat:{b(self.heater.is_on())} Fan:{b(self.fan.is_on())}"
        )

# ========= Wiring =========

def build_sim_controller() -> OrchidTentController:
    sensor = SimSensor()
    light = TimedSwitch(SimSwitch("GrowLight", sensor, effect="light"))
    humidifier = TimedSwitch(SimSwitch("Humidifier", sensor, effect="humidity"))
    heater = TimedSwitch(SimSwitch("Heater", sensor, effect="heat"))
    fan = TimedSwitch(SimSwitch("Fan", sensor, effect="air"))
    return OrchidTentController(sensor, light, humidifier, heater, fan, simulate_env=True)

def build_i2c_controller(bus: int, sht31_addr: int, bh1750_addr: Optional[int], simulate_lux: bool, simulate_relays: bool) -> OrchidTentController:
    if not HAVE_SMBUS:
        raise RuntimeError("smbus2 not available. Install with: pip install smbus2")
    sht = SHT31(bus=bus, address=sht31_addr)
    lux = None if simulate_lux or bh1750_addr is None else BH1750(bus=bus, address=bh1750_addr)
    sensor = CompositeRealSensor(sht31=sht, bh1750=lux, simulate_lux=(lux is None))

    if simulate_relays:
        light = TimedSwitch(SimSwitch("GrowLight"))
        humidifier = TimedSwitch(SimSwitch("Humidifier"))
        heater = TimedSwitch(SimSwitch("Heater"))
        fan = TimedSwitch(SimSwitch("Fan"))
    else:
        if not HAVE_GPIO:
            raise RuntimeError("GPIO not available; use --simulate-relays or install RPi.GPIO")
        light = TimedSwitch(GPIOSwitch("GrowLight", pin=17, active_high=False))
        humidifier = TimedSwitch(GPIOSwitch("Humidifier", pin=27, active_high=False))
        heater = TimedSwitch(GPIOSwitch("Heater", pin=22, active_high=False))
        fan = TimedSwitch(GPIOSwitch("Fan", pin=23, active_high=False))

    return OrchidTentController(sensor, light, humidifier, heater, fan, simulate_env=True if lux is None else False, sensor_hook=sensor)

def build_gpio_controller() -> OrchidTentController:
    if not HAVE_GPIO:
        raise RuntimeError("GPIO not available.")
    sensor = SimSensor()
    light = TimedSwitch(GPIOSwitch("GrowLight", pin=17, active_high=False))
    humidifier = TimedSwitch(GPIOSwitch("Humidifier", pin=27, active_high=False))
    heater = TimedSwitch(GPIOSwitch("Heater", pin=22, active_high=False))
    fan = TimedSwitch(GPIOSwitch("Fan", pin=23, active_high=False))
    return OrchidTentController(sensor, light, humidifier, heater, fan, simulate_env=True)

# ========= Main =========

def run_loop(controller: OrchidTentController, once: bool = False) -> None:
    print("Starting Orchid Tent Controller (Ctrl+C to stop)")
    print(f"Logging to: {LOG_PATH}")
    running = True

    def handle_sigint(sig, frame):
        nonlocal running
        running = False
        print("\nStopping...")

    signal.signal(signal.SIGINT, handle_sigint)

    while running:
        reading = controller.step()
        controller.log(reading)
        controller.pretty_print(reading)
        if once:
            break
        time.sleep(LOOP_PERIOD_S)

    for sw in (controller.light, controller.humidifier, controller.heater, controller.fan):
        try: sw.off()
        except Exception: pass
    print("All actuators set to OFF. Bye!")

def parse_int_auto(x: str) -> int:
    x = x.strip().lower()
    if x.startswith("0x"):
        return int(x, 16)
    return int(x)

def parse_args(argv):
    p = argparse.ArgumentParser(description="Self-regulating orchid tent controller (with I2C adapters)")
    p.add_argument("--simulate", action="store_true", help="Force full simulation for sensors and relays")
    p.add_argument("--simulate-relays", action="store_true", help="Use simulated switches even if GPIO is available")
    p.add_argument("--simulate-lux", action="store_true", help="Simulate lux even when using SHT31 (only temp/RH real)")
    p.add_argument("--i2c", action="store_true", help="Use I2C sensors (SHT31 + BH1750)")
    p.add_argument("--i2c-bus", type=int, default=1, help="I2C bus number (default 1)")
    p.add_argument("--sht31-addr", type=parse_int_auto, default="0x44", help="SHT31 I2C address (e.g., 0x44 or 0x45)")
    p.add_argument("--bh1750-addr", type=str, default="0x23", help="BH1750 I2C address (0x23 or 0x5C); use 'none' to disable")
    p.add_argument("--once", action="store_true", help="Run a single control step then exit")
    return p.parse_args(argv)

if __name__ == "__main__":
    args = parse_args(sys.argv[1:])
    try:
        if args.simulate:
            ctl = build_sim_controller()
        elif args.i2c:
            bh_addr = None if str(args.bh1750_addr).strip().lower() == "none" else int(args.bh1750_addr, 16) if str(args.bh1750_addr).startswith("0x") else int(args.bh1750_addr)
            ctl = build_i2c_controller(bus=args.i2c_bus, sht31_addr=int(args.sht31_addr), bh1750_addr=bh_addr, simulate_lux=args.simulate_lux, simulate_relays=args.simulate_relays)
        elif HAVE_GPIO:
            ctl = build_gpio_controller()
        else:
            ctl = build_sim_controller()
        run_loop(ctl, once=args.once)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
