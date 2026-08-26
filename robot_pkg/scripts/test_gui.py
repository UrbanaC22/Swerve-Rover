#!/usr/bin/env python3
"""
Rover GUI Control & Test Bench
===============================
GUI front-end for the dual-motor-group rover (SteadyWin GIM6010-36 x4 "Group A" +
SteadyWin GIM8108-9 x4 "Group B"), built on the same CAN protocol and safety logic as
the curses dual_motor_rover script, but as a proper desktop GUI with:
  - Live per-motor status tables (volt/curr/temp/mode/fault/target/actual)
  - Buttons + sliders for drive, ramp, max-current, Kp/Ki read & write
  - One-click "Init CAN Bus" so you don't have to run the `ip link` dance by hand
  - A scrolling, colour-coded event log (also written to a .log file on disk)
  - A CSV telemetry logger (start/stop) that records every telemetry sample for
    later analysis / plotting

Protocol: Custom CAN Communication Protocol Rev.3.09b0

IMPORTANT protocol-correctness note (carried over from the curses version):
  0xB5 (ramp), 0xB3 (max current), 0xB8/0xB9 (Kp/Ki) "set" commands are broadcast-style
  in the datasheet's own examples (sent to 0x00), which means EVERY device on the bus
  applies them. With two different motor models sharing one bus, broadcasting would
  silently push Group A's tuning onto Group B (and vice versa). So every "set" command
  here is sent individually, addressed to each of that group's own 4 CAN IDs, never to
  0x00 -- including E-STOP (0xCF), which loops per-ID per group.

Requirements:
  pip install python-can

Run:
  python3 rover_gui.py                  # uses can0 @ 1 Mbps
  python3 rover_gui.py --channel can1

You will likely need passwordless sudo for `ip link` (same as before), e.g. via
/etc/sudoers.d/canif:
  yourusername ALL=(ALL) NOPASSWD: /sbin/ip
"""

import argparse
import csv
import logging
import os
import queue
import struct
import subprocess
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from datetime import datetime
from tkinter import messagebox, scrolledtext, ttk
from typing import Dict, List, Optional, Tuple

try:
    import can
except ImportError:
    print("Missing dependency: pip install python-can", file=sys.stderr)
    sys.exit(1)

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────
CHANNEL = "can0"
BITRATE = 1_000_000

POLL_HZ = 10          # telemetry request rate (round-robins across all 8 motors)
CMD_HZ = 20            # drive-command send rate
GUI_REFRESH_MS = 100   # how often the GUI redraws from motor state
HEALTH_POLL_S = 2.0    # how often we re-check `ip -details link show`

LOG_DIR = os.path.join(os.path.expanduser("~"), "rover_logs")

FAULT_BITS = {
    0: "Voltage", 1: "Current", 2: "Temperature", 3: "Encoder",
    5: "Comms", 6: "Hardware", 7: "Software",
}
RUN_MODES = {0: "OFF", 1: "VOLT", 2: "IQ", 3: "SPD", 4: "POS"}


@dataclass
class GroupConfig:
    label: str
    model: str
    motor_ids: Dict[str, int]
    direction: Dict[str, int]
    left_positions: Tuple[str, str]
    right_positions: Tuple[str, str]
    rated_speed_rpm: float
    max_speed_rpm: float
    rated_current_A: float
    peak_current_A: float
    torque_constant: float
    gear_ratio: int
    default_linear_rpm: float
    default_turn_rpm: float
    default_ramp_rpm_s: float
    default_maxcurr_A: float
    ramp_step: float
    current_step: float
    key_fwd: str
    key_bwd: str
    key_left: str
    key_right: str


GROUP_A = GroupConfig(
    label="A", model="GIM6010-36",
    motor_ids={"FL": 5, "FR": 6, "RL": 7, "RR": 8},
    direction={"FL": +1, "FR": -1, "RL": +1, "RR": -1},
    left_positions=("FL", "RL"), right_positions=("FR", "RR"),
    rated_speed_rpm=80.0, max_speed_rpm=105.0,
    rated_current_A=4.38, peak_current_A=20.05,
    torque_constant=4.30, gear_ratio=36,
    default_linear_rpm=50.0, default_turn_rpm=30.0,
    default_ramp_rpm_s=100.0, default_maxcurr_A=5.0,
    ramp_step=20.0, current_step=1.0,
    key_fwd="w", key_bwd="s", key_left="a", key_right="d",
)

GROUP_B = GroupConfig(
    label="B", model="GIM8108-9",
    motor_ids={"FL": 1, "FR": 2, "RL": 3, "RR": 4},
    direction={"FL": +1, "FR": -1, "RL": +1, "RR": -1},
    left_positions=("FL", "RL"), right_positions=("FR", "RR"),
    rated_speed_rpm=190.0, max_speed_rpm=223.0,
    rated_current_A=8.93, peak_current_A=56.01,
    torque_constant=0.72, gear_ratio=9,
    default_linear_rpm=130.0, default_turn_rpm=60.0,
    default_ramp_rpm_s=50.0, default_maxcurr_A=10.0,
    ramp_step=20.0, current_step=2.0,
    key_fwd="i", key_bwd="m", key_left="j", key_right="l",
)

# direction={"FL":+1,"FR":-1,"RL":+1,"RR":-1} is copied from the original mechanical
# mounting convention. VERIFY this against your actual chassis wiring for Group B
# (GIM8108-9) before driving -- it was not confirmed for this motor group.


# ─────────────────────────────────────────────────────────────────────────────
# LOGGING (file + in-GUI queue)
# ─────────────────────────────────────────────────────────────────────────────
class QueueLogHandler(logging.Handler):
    """Pushes formatted log records into a thread-safe queue for the GUI to drain."""

    def __init__(self, q: "queue.Queue[Tuple[str, str]]"):
        super().__init__()
        self.q = q

    def emit(self, record: logging.LogRecord):
        try:
            msg = self.format(record)
            self.q.put((record.levelname, msg))
        except Exception:
            pass


def build_logger(log_queue: "queue.Queue[Tuple[str, str]]") -> Tuple[logging.Logger, str]:
    os.makedirs(LOG_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(LOG_DIR, f"rover_{ts}.log")

    logger = logging.getLogger("rover")
    logger.setLevel(logging.DEBUG)
    logger.handlers.clear()

    fh = logging.FileHandler(log_path)
    fh.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s", "%H:%M:%S"))
    logger.addHandler(fh)

    qh = QueueLogHandler(log_queue)
    qh.setFormatter(logging.Formatter("%(asctime)s  %(message)s", "%H:%M:%S"))
    logger.addHandler(qh)

    return logger, log_path


class TelemetryCSVLogger:
    """Optional CSV data logger -- one row per decoded telemetry frame, for offline
    plotting/analysis. Start/stop from the GUI; does nothing unless enabled."""

    HEADER = ["timestamp", "group", "pos", "can_id", "voltage_V", "current_A",
              "temperature_C", "run_mode", "fault_code", "faults",
              "target_rpm", "actual_rpm"]

    def __init__(self):
        self._lock = threading.Lock()
        self._file = None
        self._writer = None
        self.path: Optional[str] = None
        self.row_count = 0

    @property
    def active(self) -> bool:
        return self._file is not None

    def start(self) -> str:
        os.makedirs(LOG_DIR, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(LOG_DIR, f"telemetry_{ts}.csv")
        with self._lock:
            self._file = open(self.path, "w", newline="")
            self._writer = csv.writer(self._file)
            self._writer.writerow(self.HEADER)
            self.row_count = 0
        return self.path

    def stop(self):
        with self._lock:
            if self._file:
                self._file.close()
            self._file = None
            self._writer = None

    def log_row(self, group_label: str, pos: str, can_id: int, state: "MotorState"):
        with self._lock:
            if not self._writer:
                return
            self._writer.writerow([
                f"{time.time():.3f}", group_label, pos, can_id,
                f"{state.voltage_V:.2f}", f"{state.current_A:.2f}",
                state.temperature_C, state.run_mode, state.fault_code,
                "|".join(state.faults()),
                f"{state.target_rpm:.1f}", f"{state.actual_rpm:.1f}",
            ])
            self.row_count += 1
            if self.row_count % 20 == 0:
                self._file.flush()


# ─────────────────────────────────────────────────────────────────────────────
# CAN INTERFACE HELPERS
# ─────────────────────────────────────────────────────────────────────────────
def setup_can_interface(channel: str, bitrate: int = BITRATE) -> Tuple[bool, str]:
    """Runs the same `ip link ... down/type can/up` dance you were doing by hand."""
    try:
        subprocess.run(["sudo", "ip", "link", "set", channel, "down"],
                        capture_output=True, timeout=5)
        time.sleep(0.2)
        subprocess.run(
            ["sudo", "ip", "link", "set", channel, "type", "can",
             "bitrate", str(bitrate), "restart-ms", "100"],
            check=True, capture_output=True, timeout=5,
        )
        subprocess.run(["sudo", "ip", "link", "set", channel, "txqueuelen", "1000"],
                        check=True, capture_output=True, timeout=5)
        subprocess.run(["sudo", "ip", "link", "set", channel, "up"],
                        check=True, capture_output=True, timeout=5)
        return True, f"{channel} up @ {bitrate} bps"
    except subprocess.CalledProcessError as e:
        err = e.stderr.decode(errors="replace").strip() if e.stderr else str(e)
        return False, f"ip link setup failed: {err}"
    except subprocess.TimeoutExpired:
        return False, "ip link setup timed out (is sudo asking for a password?)"
    except Exception as e:
        return False, f"ip link setup error: {e}"


def get_can_state(channel: str) -> str:
    try:
        out = subprocess.check_output(
            ["ip", "-details", "link", "show", channel],
            stderr=subprocess.DEVNULL, timeout=3).decode()
        for line in out.splitlines():
            if "state" in line and "berr-counter" in line:
                return line.strip().split()[2]
    except Exception:
        pass
    return "UNKNOWN"


# ─────────────────────────────────────────────────────────────────────────────
# MOTOR STATE / MIXING
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class MotorState:
    voltage_V: float = 0.0
    current_A: float = 0.0
    temperature_C: int = 0
    run_mode: int = 0
    fault_code: int = 0
    actual_rpm: float = 0.0
    target_rpm: float = 0.0
    last_rx: float = 0.0

    def faults(self) -> List[str]:
        if self.fault_code == 0:
            return ["NONE"]
        return [name for bit, name in FAULT_BITS.items() if self.fault_code & (1 << bit)]


def mix(linear: float, angular: float, cfg: GroupConfig) -> Dict[str, float]:
    left_rpm = linear - angular
    right_rpm = linear + angular
    out = {}
    for pos in cfg.left_positions:
        out[pos] = left_rpm * cfg.direction[pos]
    for pos in cfg.right_positions:
        out[pos] = right_rpm * cfg.direction[pos]
    return out


# ─────────────────────────────────────────────────────────────────────────────
# MOTOR GROUP (protocol logic -- unchanged behaviour from the curses version)
# ─────────────────────────────────────────────────────────────────────────────
class MotorGroup:
    """Holds state + issues commands for one group of 4 motors. All 'set' commands are
    sent individually addressed to this group's own CAN IDs only -- never broadcast --
    so the two groups never bleed parameters into each other."""

    def __init__(self, send_fn, cfg: GroupConfig, logger: logging.Logger,
                 telemetry: TelemetryCSVLogger):
        self.cfg = cfg
        self._send = send_fn
        self.logger = logger
        self.telemetry = telemetry
        self.states: Dict[str, MotorState] = {pos: MotorState() for pos in cfg.motor_ids}
        self.ramp = cfg.default_ramp_rpm_s
        self.maxcurr = cfg.default_maxcurr_A
        self.last_kp: Optional[float] = None
        self.last_ki: Optional[float] = None
        self.last_kt: Optional[float] = None
        self.lock = threading.Lock()

    # -- motion / safety ------------------------------------------------------
    def set_motor_speed(self, pos: str, rpm: float):
        clamped = max(-self.cfg.max_speed_rpm, min(self.cfg.max_speed_rpm, rpm))
        if abs(clamped - rpm) > 0.05:
            self.logger.debug(f"Grp{self.cfg.label} {pos} tgt {rpm:+.1f} clamped to "
                               f"{clamped:+.1f} (MAX {self.cfg.max_speed_rpm:.0f})")
        self._send(self.cfg.motor_ids[pos], bytes([0xC1]) + struct.pack("<i", int(clamped * 100)))
        with self.lock:
            self.states[pos].target_rpm = clamped

    def emergency_stop(self):
        for cid in self.cfg.motor_ids.values():
            self._send(cid, bytes([0xCF]))
        with self.lock:
            for s in self.states.values():
                s.target_rpm = 0.0
        self.logger.warning(f"Group {self.cfg.label}: HARD E-STOP (0xCF)")

    def clear_faults(self):
        for cid in self.cfg.motor_ids.values():
            self._send(cid, bytes([0xAF]))
            time.sleep(0.01)
        self.logger.info(f"Group {self.cfg.label}: faults cleared")

    # -- tunables, individually addressed --------------------------------------
    def set_ramp(self, rpm_s: float) -> float:
        rpm_s = max(0.0, rpm_s)
        for cid in self.cfg.motor_ids.values():
            self._send(cid, bytes([0xB5]) + struct.pack("<I", int(rpm_s * 100)))
        self.ramp = rpm_s
        self.logger.info(f"Group {self.cfg.label}: ramp = {rpm_s:.0f} RPM/s")
        return rpm_s

    def set_max_current(self, amps: float) -> float:
        clamped = max(0.0, min(self.cfg.peak_current_A, amps))
        for cid in self.cfg.motor_ids.values():
            self._send(cid, bytes([0xB3]) + struct.pack("<I", int(clamped * 1000)))
        self.maxcurr = clamped
        self.logger.info(f"Group {self.cfg.label}: max current = {clamped:.2f} A")
        return clamped

    def set_speed_kp(self, kp: float):
        for cid in self.cfg.motor_ids.values():
            self._send(cid, bytes([0xB8]) + struct.pack("<f", kp))
        self.logger.info(f"Group {self.cfg.label}: Kp written = {kp}")

    def set_speed_ki(self, ki: float):
        for cid in self.cfg.motor_ids.values():
            self._send(cid, bytes([0xB9]) + struct.pack("<f", ki))
        self.logger.info(f"Group {self.cfg.label}: Ki written = {ki}")

    def read_speed_kp(self):
        rep = next(iter(self.cfg.motor_ids.values()))
        self._send(rep, bytes([0xB8]))

    def read_speed_ki(self):
        rep = next(iter(self.cfg.motor_ids.values()))
        self._send(rep, bytes([0xB9]))

    def read_torque_kt(self):
        rep = next(iter(self.cfg.motor_ids.values()))
        self._send(rep, bytes([0xB0]))

    def read_status(self, pos: str):
        self._send(self.cfg.motor_ids[pos], bytes([0xAE]))

    # -- decode -----------------------------------------------------------------
    def decode(self, can_id: int, data: bytes):
        pos = None
        for p, cid in self.cfg.motor_ids.items():
            if cid == can_id:
                pos = p
                break
        if pos is None or not data:
            return
        with self.lock:
            s = self.states[pos]
            cmd = data[0]
            s.last_rx = time.monotonic()
            prev_fault = s.fault_code

            try:
                if cmd in (0xAE, 0xCF) and len(data) >= 8:
                    s.voltage_V = struct.unpack_from("<H", data, 1)[0] * 0.01
                    s.current_A = struct.unpack_from("<H", data, 3)[0] * 0.01
                    s.temperature_C = data[5]
                    s.run_mode = data[6]
                    s.fault_code = data[7]
                    self.telemetry.log_row(self.cfg.label, pos, can_id, s)
                    if s.fault_code != 0 and s.fault_code != prev_fault:
                        self.logger.error(
                            f"Group {self.cfg.label} {pos}: FAULT "
                            f"{','.join(s.faults())} (code {s.fault_code})")
                elif cmd == 0xC1 and len(data) >= 5:
                    s.actual_rpm = struct.unpack_from("<i", data, 1)[0] * 0.01
                elif cmd == 0xB8 and len(data) >= 5:
                    self.last_kp = struct.unpack_from("<f", data, 1)[0]
                    self.logger.info(f"Group {self.cfg.label} {pos}: Speed Kp = {self.last_kp:.5f}")
                elif cmd == 0xB9 and len(data) >= 5:
                    self.last_ki = struct.unpack_from("<f", data, 1)[0]
                    self.logger.info(f"Group {self.cfg.label} {pos}: Speed Ki = {self.last_ki:.5f}")
                elif cmd == 0xB0 and len(data) >= 6:
                    self.last_kt = struct.unpack_from("<f", data, 2)[0]
                    self.logger.info(
                        f"Group {self.cfg.label} {pos}: Kt = {self.last_kt:.3f} Nm/A "
                        f"(datasheet {self.cfg.torque_constant:.2f})")
            except struct.error:
                pass


# ─────────────────────────────────────────────────────────────────────────────
# ROVER (bus + both groups + rx thread)
# ─────────────────────────────────────────────────────────────────────────────
class Rover:
    def __init__(self, channel: str, logger: logging.Logger, telemetry: TelemetryCSVLogger):
        self.channel = channel
        self.logger = logger
        self._tx_lock = threading.Lock()
        self._stop = threading.Event()
        self._bus = can.interface.Bus(channel=channel, interface="socketcan",
                                       receive_own_messages=False)

        self.group_a = MotorGroup(self._send, GROUP_A, logger, telemetry)
        self.group_b = MotorGroup(self._send, GROUP_B, logger, telemetry)

        self._id_to_group: Dict[int, MotorGroup] = {}
        for cid in GROUP_A.motor_ids.values():
            self._id_to_group[cid] = self.group_a
        for cid in GROUP_B.motor_ids.values():
            self._id_to_group[cid] = self.group_b

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def _send(self, can_id: int, data: bytes) -> bool:
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        try:
            with self._tx_lock:
                self._bus.send(msg, timeout=0.02)
            return True
        except can.CanError as e:
            self.logger.error(f"CAN send failed (id {can_id}): {e}")
            return False

    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                msg = self._bus.recv(timeout=0.05)
                if msg is None:
                    continue
                grp = self._id_to_group.get(msg.arbitration_id)
                if grp:
                    grp.decode(msg.arbitration_id, bytes(msg.data))
            except Exception:
                time.sleep(0.01)

    def emergency_stop_all(self):
        self.group_a.emergency_stop()
        self.group_b.emergency_stop()

    def clear_faults_all(self):
        self.group_a.clear_faults()
        self.group_b.clear_faults()

    def close(self):
        self._stop.set()
        self._rx_thread.join(timeout=0.5)
        try:
            self._bus.shutdown()
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────────────────
# GUI
# ─────────────────────────────────────────────────────────────────────────────
STATUS_COLS = ("pos", "volt", "curr", "temp", "mode", "fault", "tgt", "act")
STATUS_HEADERS = {"pos": "Pos", "volt": "Volt", "curr": "Curr", "temp": "Temp",
                   "mode": "Mode", "fault": "Fault", "tgt": "Tgt RPM", "act": "Act RPM"}


class GroupPanel(ttk.LabelFrame):
    """One side of the GUI: settings + drive + live status table for one motor group."""

    def __init__(self, master, cfg: GroupConfig, on_apply_ramp, on_apply_maxcurr,
                 on_read_kp, on_read_ki, on_read_kt, on_write_kp, on_write_ki,
                 on_toggle_drive):
        super().__init__(master, text=f"GROUP {cfg.label} — {cfg.model} "
                                       f"({cfg.gear_ratio}:1)  IDs "
                                       f"{min(cfg.motor_ids.values())}-{max(cfg.motor_ids.values())}",
                          padding=8)
        self.cfg = cfg
        self.on_toggle_drive = on_toggle_drive
        self.drive_state = {"fwd": False, "bwd": False, "left": False, "right": False}

        # -- limits label -------------------------------------------------
        lim = ttk.Label(self, text=(
            f"Max {cfg.max_speed_rpm:.0f} RPM | Rated {cfg.rated_current_A:.2f} A | "
            f"Peak {cfg.peak_current_A:.2f} A | Kt(ds) {cfg.torque_constant:.2f} Nm/A"))
        lim.grid(row=0, column=0, columnspan=6, sticky="w", pady=(0, 6))

        # -- drive controls -------------------------------------------------
        drive = ttk.Frame(self)
        drive.grid(row=1, column=0, columnspan=6, sticky="w", pady=(0, 6))
        self.linear_var = tk.DoubleVar(value=cfg.default_linear_rpm)
        self.turn_var = tk.DoubleVar(value=cfg.default_turn_rpm)
        ttk.Label(drive, text="Linear RPM").grid(row=0, column=0)
        ttk.Spinbox(drive, from_=0, to=cfg.max_speed_rpm, increment=5,
                    textvariable=self.linear_var, width=6).grid(row=0, column=1, padx=(2, 10))
        ttk.Label(drive, text="Turn RPM").grid(row=0, column=2)
        ttk.Spinbox(drive, from_=0, to=cfg.max_speed_rpm, increment=5,
                    textvariable=self.turn_var, width=6).grid(row=0, column=3, padx=(2, 10))

        self.btn_fwd = ttk.Button(drive, text=f"FWD ({cfg.key_fwd.upper()})",
                                   command=lambda: self._toggle("fwd"))
        self.btn_bwd = ttk.Button(drive, text=f"BACK ({cfg.key_bwd.upper()})",
                                   command=lambda: self._toggle("bwd"))
        self.btn_left = ttk.Button(drive, text=f"LEFT ({cfg.key_left.upper()})",
                                    command=lambda: self._toggle("left"))
        self.btn_right = ttk.Button(drive, text=f"RIGHT ({cfg.key_right.upper()})",
                                     command=lambda: self._toggle("right"))
        self.btn_fwd.grid(row=1, column=0, padx=2, pady=2)
        self.btn_bwd.grid(row=1, column=1, padx=2, pady=2)
        self.btn_left.grid(row=1, column=2, padx=2, pady=2)
        self.btn_right.grid(row=1, column=3, padx=2, pady=2)

        # -- ramp / max current --------------------------------------------
        tune = ttk.Frame(self)
        tune.grid(row=2, column=0, columnspan=6, sticky="w", pady=(0, 6))
        self.ramp_var = tk.DoubleVar(value=cfg.default_ramp_rpm_s)
        self.maxcurr_var = tk.DoubleVar(value=cfg.default_maxcurr_A)
        ttk.Label(tune, text="Ramp RPM/s").grid(row=0, column=0)
        ttk.Spinbox(tune, from_=10, to=2000, increment=cfg.ramp_step,
                    textvariable=self.ramp_var, width=7).grid(row=0, column=1, padx=(2, 4))
        ttk.Button(tune, text="Apply", command=lambda: on_apply_ramp(self.ramp_var.get())
                   ).grid(row=0, column=2, padx=(0, 12))
        ttk.Label(tune, text="Max Current A").grid(row=0, column=3)
        ttk.Spinbox(tune, from_=0, to=cfg.peak_current_A, increment=cfg.current_step,
                    textvariable=self.maxcurr_var, width=7).grid(row=0, column=4, padx=(2, 4))
        ttk.Button(tune, text="Apply", command=lambda: on_apply_maxcurr(self.maxcurr_var.get())
                   ).grid(row=0, column=5)

        # -- PID ---------------------------------------------------------
        pid = ttk.Frame(self)
        pid.grid(row=3, column=0, columnspan=6, sticky="w", pady=(0, 6))
        self.kp_val = tk.StringVar(value="Kp: --")
        self.ki_val = tk.StringVar(value="Ki: --")
        self.kt_val = tk.StringVar(value="Kt: --")
        ttk.Label(pid, textvariable=self.kp_val, width=14).grid(row=0, column=0)
        ttk.Button(pid, text="Read Kp", command=on_read_kp).grid(row=0, column=1, padx=2)
        ttk.Label(pid, textvariable=self.ki_val, width=14).grid(row=0, column=2)
        ttk.Button(pid, text="Read Ki", command=on_read_ki).grid(row=0, column=3, padx=2)
        ttk.Label(pid, textvariable=self.kt_val, width=16).grid(row=0, column=4)
        ttk.Button(pid, text="Read Kt", command=on_read_kt).grid(row=0, column=5, padx=2)

        pid2 = ttk.Frame(self)
        pid2.grid(row=4, column=0, columnspan=6, sticky="w", pady=(0, 8))
        self.kp_entry = tk.StringVar()
        self.ki_entry = tk.StringVar()
        ttk.Entry(pid2, textvariable=self.kp_entry, width=10).grid(row=0, column=0, padx=(0, 2))
        ttk.Button(pid2, text="Write Kp",
                   command=lambda: self._write_val(self.kp_entry, on_write_kp)
                   ).grid(row=0, column=1, padx=(0, 12))
        ttk.Entry(pid2, textvariable=self.ki_entry, width=10).grid(row=0, column=2, padx=(0, 2))
        ttk.Button(pid2, text="Write Ki",
                   command=lambda: self._write_val(self.ki_entry, on_write_ki)
                   ).grid(row=0, column=3)

        # -- status table --------------------------------------------------
        self.tree = ttk.Treeview(self, columns=STATUS_COLS, show="headings", height=4)
        for c in STATUS_COLS:
            self.tree.heading(c, text=STATUS_HEADERS[c])
            self.tree.column(c, width=70 if c != "fault" else 110, anchor="center")
        self.tree.grid(row=5, column=0, columnspan=6, sticky="ew")
        self.tree.tag_configure("fault", background="#5a1f1f")
        self.tree.tag_configure("ok", background="")
        for pos in ("FL", "FR", "RL", "RR"):
            self.tree.insert("", "end", iid=f"{cfg.label}_{pos}",
                              values=(pos, "-", "-", "-", "-", "-", "-", "-"))

    def _toggle(self, key: str):
        self.drive_state[key] = not self.drive_state[key]
        if key in ("fwd", "bwd") and self.drive_state[key]:
            self.drive_state["bwd" if key == "fwd" else "fwd"] = False
        if key in ("left", "right") and self.drive_state[key]:
            self.drive_state["right" if key == "left" else "left"] = False
        self._refresh_button_styles()
        self.on_toggle_drive()

    def _refresh_button_styles(self):
        for key, btn in (("fwd", self.btn_fwd), ("bwd", self.btn_bwd),
                          ("left", self.btn_left), ("right", self.btn_right)):
            btn.state(["pressed"] if self.drive_state[key] else ["!pressed"])

    def _write_val(self, var: tk.StringVar, cb):
        txt = var.get().strip()
        if not txt:
            return
        try:
            cb(float(txt))
        except ValueError:
            messagebox.showerror("Invalid value", f"'{txt}' is not a number")

    def set_speed_from_keys(self, direction_key: str):
        """Toggle drive state programmatically (used by global keyboard handler)."""
        self._toggle(direction_key)

    def update_status(self, states: Dict[str, MotorState]):
        for pos, s in states.items():
            fault_str = ",".join(s.faults())
            tag = "fault" if s.fault_code != 0 else "ok"
            self.tree.item(f"{self.cfg.label}_{pos}", values=(
                pos, f"{s.voltage_V:.2f}V", f"{s.current_A:+.2f}A",
                f"{s.temperature_C}C", RUN_MODES.get(s.run_mode, "?"),
                fault_str, f"{s.target_rpm:+.1f}", f"{s.actual_rpm:+.1f}",
            ), tags=(tag,))

    def update_pid_labels(self, group: MotorGroup):
        if group.last_kp is not None:
            self.kp_val.set(f"Kp: {group.last_kp:.5f}")
        if group.last_ki is not None:
            self.ki_val.set(f"Ki: {group.last_ki:.5f}")
        if group.last_kt is not None:
            self.kt_val.set(f"Kt: {group.last_kt:.3f} Nm/A")


class RoverGUI(tk.Tk):
    LEVEL_COLORS = {"DEBUG": "#888888", "INFO": "#dddddd", "WARNING": "#e0a53a",
                     "ERROR": "#e05a5a", "CRITICAL": "#ff3333"}

    def __init__(self, channel: str, auto_init: bool = True):
        super().__init__()
        self.title("Rover Test Bench — Dual Motor Group Control")
        self.geometry("1280x900")
        self.channel = channel
        self.rover: Optional[Rover] = None

        self.log_queue: "queue.Queue[Tuple[str, str]]" = queue.Queue()
        self.logger, self.log_path = build_logger(self.log_queue)
        self.telemetry = TelemetryCSVLogger()

        self._build_widgets()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.bind_all("<KeyPress>", self._on_key)

        self.after(100, self._drain_log_queue)
        self.after(200, self._health_tick)

        if auto_init:
            self.after(300, self._init_can_bus)

    # ---------------------------------------------------------------- UI ---
    def _build_widgets(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")

        self.status_var = tk.StringVar(value=f"CAN: not initialized ({self.channel})")
        ttk.Label(top, textvariable=self.status_var, font=("TkDefaultFont", 10, "bold")
                  ).pack(side="left")

        ttk.Button(top, text="Init CAN Bus", command=self._init_can_bus
                   ).pack(side="left", padx=(16, 4))
        ttk.Button(top, text="Clear Faults (All)", command=self._clear_faults
                   ).pack(side="left", padx=4)
        ttk.Button(top, text="Soft Stop (All)", command=self._soft_stop
                   ).pack(side="left", padx=4)

        estop_btn = tk.Button(top, text="  HARD E-STOP (X)  ", command=self._hard_estop,
                               bg="#a02020", fg="white", font=("TkDefaultFont", 10, "bold"))
        estop_btn.pack(side="left", padx=12)

        self.telem_btn = ttk.Button(top, text="Start Telemetry Log",
                                     command=self._toggle_telemetry)
        self.telem_btn.pack(side="right", padx=4)
        self.telem_status_var = tk.StringVar(value="Telemetry: off")
        ttk.Label(top, textvariable=self.telem_status_var).pack(side="right", padx=(4, 12))

        panels = ttk.Frame(self, padding=8)
        panels.pack(fill="both", expand=False)
        panels.columnconfigure(0, weight=1)
        panels.columnconfigure(1, weight=1)

        self.panel_a = GroupPanel(
            panels, GROUP_A,
            on_apply_ramp=lambda v: self._call_group(self._group_a_or_none(), "set_ramp", v),
            on_apply_maxcurr=lambda v: self._call_group(self._group_a_or_none(), "set_max_current", v),
            on_read_kp=lambda: self._call_group(self._group_a_or_none(), "read_speed_kp"),
            on_read_ki=lambda: self._call_group(self._group_a_or_none(), "read_speed_ki"),
            on_read_kt=lambda: self._call_group(self._group_a_or_none(), "read_torque_kt"),
            on_write_kp=lambda v: self._call_group(self._group_a_or_none(), "set_speed_kp", v),
            on_write_ki=lambda v: self._call_group(self._group_a_or_none(), "set_speed_ki", v),
            on_toggle_drive=lambda: None,
        )
        self.panel_a.grid(row=0, column=0, sticky="nsew", padx=(0, 6))

        self.panel_b = GroupPanel(
            panels, GROUP_B,
            on_apply_ramp=lambda v: self._call_group(self._group_b_or_none(), "set_ramp", v),
            on_apply_maxcurr=lambda v: self._call_group(self._group_b_or_none(), "set_max_current", v),
            on_read_kp=lambda: self._call_group(self._group_b_or_none(), "read_speed_kp"),
            on_read_ki=lambda: self._call_group(self._group_b_or_none(), "read_speed_ki"),
            on_read_kt=lambda: self._call_group(self._group_b_or_none(), "read_torque_kt"),
            on_write_kp=lambda v: self._call_group(self._group_b_or_none(), "set_speed_kp", v),
            on_write_ki=lambda v: self._call_group(self._group_b_or_none(), "set_speed_ki", v),
            on_toggle_drive=lambda: None,
        )
        self.panel_b.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        legend = ttk.Label(self, padding=(8, 0), foreground="#888888", text=(
            "Keys — GLOBAL: [SPACE] soft stop  [X] hard e-stop  [F] clear faults  [Q] quit   |   "
            "GROUP A: W/S fwd/back  A/D turn   |   GROUP B: I/M fwd/back  J/L turn"))
        legend.pack(fill="x")

        log_frame = ttk.LabelFrame(self, text=f"Event Log  (file: {self.log_path})", padding=6)
        log_frame.pack(fill="both", expand=True, padx=8, pady=8)
        self.log_text = scrolledtext.ScrolledText(log_frame, height=12, bg="#1e1e1e",
                                                    fg="#dddddd", insertbackground="#dddddd",
                                                    state="disabled", wrap="word")
        self.log_text.pack(fill="both", expand=True)
        for level, color in self.LEVEL_COLORS.items():
            self.log_text.tag_configure(level, foreground=color)

    # ---------------------------------------------------------- accessors --
    def _group_a_or_none(self) -> Optional[MotorGroup]:
        return self.rover.group_a if self.rover else None

    def _group_b_or_none(self) -> Optional[MotorGroup]:
        return self.rover.group_b if self.rover else None

    def _call_group(self, group: Optional[MotorGroup], method: str, *args):
        if group is None:
            self.logger.warning("CAN bus not initialized yet -- click 'Init CAN Bus' first")
            return
        getattr(group, method)(*args)

    # -------------------------------------------------------------- CAN ----
    def _init_can_bus(self):
        self.status_var.set(f"CAN: initializing {self.channel}...")
        threading.Thread(target=self._init_can_bus_worker, daemon=True).start()

    def _init_can_bus_worker(self):
        ok, msg = setup_can_interface(self.channel, BITRATE)
        if not ok:
            self.logger.error(msg)
            self.status_var.set(f"CAN: init FAILED ({self.channel})")
            return
        try:
            if self.rover is not None:
                self.rover.close()
            self.rover = Rover(self.channel, self.logger, self.telemetry)
            self.rover.clear_faults_all()
            self.rover.group_a.set_ramp(GROUP_A.default_ramp_rpm_s)
            self.rover.group_a.set_max_current(GROUP_A.default_maxcurr_A)
            self.rover.group_b.set_ramp(GROUP_B.default_ramp_rpm_s)
            self.rover.group_b.set_max_current(GROUP_B.default_maxcurr_A)
            self.logger.info(f"Rover ready on {self.channel} ({msg})")
            self.status_var.set(f"CAN: ready ({self.channel})")
            self._start_control_loop()
        except Exception as e:
            self.logger.error(f"Failed to bring up CAN bus objects: {e}")
            self.status_var.set(f"CAN: init FAILED ({self.channel})")

    def _start_control_loop(self):
        self._poll_seq = ([(self.rover.group_a, p) for p in ("FL", "FR", "RL", "RR")] +
                           [(self.rover.group_b, p) for p in ("FL", "FR", "RL", "RR")])
        self._poll_idx = 0
        self._last_poll = 0.0
        self._last_cmd = 0.0
        self.after(50, self._control_tick)
        self.after(GUI_REFRESH_MS, self._gui_refresh_tick)

    def _control_tick(self):
        if self.rover is None:
            return
        now = time.monotonic()

        if now - self._last_poll >= 1.0 / POLL_HZ:
            grp, pos = self._poll_seq[self._poll_idx % len(self._poll_seq)]
            self._poll_idx += 1
            grp.read_status(pos)
            self._last_poll = now

        if now - self._last_cmd >= 1.0 / CMD_HZ:
            for panel, cfg, group in ((self.panel_a, GROUP_A, self.rover.group_a),
                                       (self.panel_b, GROUP_B, self.rover.group_b)):
                ds = panel.drive_state
                lin = (panel.linear_var.get() if ds["fwd"] else 0.0) - \
                      (panel.linear_var.get() if ds["bwd"] else 0.0)
                ang = (panel.turn_var.get() if ds["right"] else 0.0) - \
                      (panel.turn_var.get() if ds["left"] else 0.0)
                for pos, rpm in mix(lin, ang, cfg).items():
                    group.set_motor_speed(pos, rpm)
            self._last_cmd = now

        self.after(20, self._control_tick)

    def _gui_refresh_tick(self):
        if self.rover is None:
            return
        with self.rover.group_a.lock:
            states_a = {p: MotorState(**vars(s)) for p, s in self.rover.group_a.states.items()}
        with self.rover.group_b.lock:
            states_b = {p: MotorState(**vars(s)) for p, s in self.rover.group_b.states.items()}
        self.panel_a.update_status(states_a)
        self.panel_b.update_status(states_b)
        self.panel_a.update_pid_labels(self.rover.group_a)
        self.panel_b.update_pid_labels(self.rover.group_b)
        self.after(GUI_REFRESH_MS, self._gui_refresh_tick)

    def _health_tick(self):
        if self.rover is not None:
            state = get_can_state(self.channel)
            base = self.status_var.get().split(" | health:")[0]
            self.status_var.set(f"{base} | health: {state}")
        self.after(int(HEALTH_POLL_S * 1000), self._health_tick)

    # ------------------------------------------------------------ actions --
    def _clear_faults(self):
        if self.rover:
            self.rover.clear_faults_all()
        else:
            self.logger.warning("CAN bus not initialized yet")

    def _soft_stop(self):
        for panel in (self.panel_a, self.panel_b):
            panel.drive_state.update(fwd=False, bwd=False, left=False, right=False)
            panel._refresh_button_styles()
        self.logger.info("Soft stop (all) -- drive inputs cleared")

    def _hard_estop(self):
        self._soft_stop()
        if self.rover:
            self.rover.emergency_stop_all()
        else:
            self.logger.warning("CAN bus not initialized yet")

    def _toggle_telemetry(self):
        if self.telemetry.active:
            self.telemetry.stop()
            self.telem_btn.config(text="Start Telemetry Log")
            self.telem_status_var.set("Telemetry: off")
            self.logger.info("Telemetry CSV logging stopped")
        else:
            path = self.telemetry.start()
            self.telem_btn.config(text="Stop Telemetry Log")
            self.telem_status_var.set(f"Telemetry: {os.path.basename(path)}")
            self.logger.info(f"Telemetry CSV logging started -> {path}")

    # ------------------------------------------------------------ keyboard --
    def _on_key(self, event: tk.Event):
        # Don't hijack keystrokes while the user is typing in an Entry/Spinbox.
        if isinstance(event.widget, (tk.Entry, ttk.Entry, ttk.Spinbox, tk.Spinbox)):
            return
        key = event.keysym.lower()

        if key == "q":
            self._on_close()
        elif key == "space":
            self._soft_stop()
        elif key == "x":
            self._hard_estop()
        elif key == "f":
            self._clear_faults()
        elif key in ("w", "s", "a", "d"):
            mapping = {"w": "fwd", "s": "bwd", "a": "left", "d": "right"}
            self.panel_a.set_speed_from_keys(mapping[key])
        elif key in ("i", "m", "j", "l"):
            mapping = {"i": "fwd", "m": "bwd", "j": "left", "l": "right"}
            self.panel_b.set_speed_from_keys(mapping[key])

    # ------------------------------------------------------------- logging --
    def _drain_log_queue(self):
        while True:
            try:
                level, msg = self.log_queue.get_nowait()
            except queue.Empty:
                break
            self.log_text.configure(state="normal")
            self.log_text.insert("end", msg + "\n", (level,))
            self.log_text.see("end")
            self.log_text.configure(state="disabled")
        self.after(100, self._drain_log_queue)

    # --------------------------------------------------------------- close --
    def _on_close(self):
        try:
            if self.rover:
                self.rover.emergency_stop_all()
                self.rover.close()
        finally:
            self.telemetry.stop()
            self.destroy()


def main():
    parser = argparse.ArgumentParser(description="Rover GUI control & test bench")
    parser.add_argument("--channel", default=CHANNEL)
    parser.add_argument("--no-auto-init", action="store_true",
                         help="Don't run the CAN bus init sequence automatically on launch")
    args = parser.parse_args()

    app = RoverGUI(channel=args.channel, auto_init=not args.no_auto_init)
    app.mainloop()


if __name__ == "__main__":
    main()