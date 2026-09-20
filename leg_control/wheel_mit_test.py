#!/usr/bin/env python3
"""Single-drive MIT bench: does MIT torque control survive a continuous wheel?

Why this exists
---------------
`leg_server.py` drives the two five-bar joints with MIT frames (0x08), which
is the only CANSimple command that is *synchronous*: one frame out, one
feedback frame back, position + velocity + torque, no extra RTR round trips.
For the drive wheel we want that same property, but MIT carries position in a
16-bit field mapped over `±mit_max_pos` (12.5 rad on the output shaft by
default = ±1.99 output turns).  A wheel leaves that window after two turns.

What actually depends on the position field, though, is only:

  * the *command*: the drive computes
        tau = kp*(pos_cmd - pos) + kd*(vel_cmd - vel) + tau_ff
    so with kp = 0 the position field is multiplied by zero and cannot
    matter — that is the "pure torque over MIT" case this script runs;
  * the *feedback*: the echoed position is packed over the same range, so
    past the window it either rails at ±mit_max_pos or wraps around.

Which of those two the firmware does — and whether the drive keeps producing
the commanded torque out there, or throws an error / clamps / quietly stops —
is not in the manual.  This script finds out by measurement: it spins one
drive on pure MIT feedforward torque and compares, every cycle, the echoed
MIT feedback against the true multi-turn position from 0x09
Get_Encoder_Estimates (rotor revolutions, unbounded), then prints a verdict.

Run
---
    V=~/Personal/Bipedal__wheeled/leg_control/venv/bin/python
    $V wheel_mit_test.py --sim 1                  # emulated drive, no hardware
    $V wheel_mit_test.py --channel can0 3         # node 3 on the real bus

Nothing moves until you ask it to: the initial torque is 0.  Keys:

    + / -   torque up / down by --step        0   torque to zero
    r       reverse the torque sign           space  zero torque, stay enabled
    [ / ]   damping kd down / up              p   toggle the pos field
    q       zero torque, idle, quit           Ctrl-C  same

`p` cycles what goes into the (kp = 0, therefore ignored) position field:
`zero` -> `track` (the measured position, so you can watch it rail or wrap)
-> `sweep` (a slow ramp).  With `--kp` non-zero the field stops being
decorative and the drive really does chase it — that is the configuration
that a wheel cannot use, and `--kp` is there so you can see why.

Safety: torque slew limit, hard clamp (--tau-max, never above 6 Nm), overspeed
kill, turn limit, run-time limit, drive-error / heartbeat / feedback-timeout
watchdogs, and the drive's own break_timeout dead-man.  Any exit path zeroes
the command and puts the axis back in IDLE.
"""

from __future__ import annotations

import argparse
import csv
import math
import select
import signal
import sys
import termios
import time
import tty
from pathlib import Path
from typing import List, Optional

TWO_PI = 2.0 * math.pi
HERE = Path(__file__).resolve().parent
DEFAULT_CAN_DIR = Path.home() / "Personal/gim6010/can_control"
TAU_HARD_MAX = 6.0          # Nm output — the CLI can never exceed this


# --------------------------------------------------------------------------
# Terminal: single keypresses without blocking the control loop
# --------------------------------------------------------------------------

class Keys:
    """cbreak stdin (ISIG stays on, so Ctrl-C still works). No-op off a tty."""

    def __init__(self, enabled: bool = True):
        self.fd = sys.stdin.fileno() if enabled and sys.stdin.isatty() else None
        self._saved = None

    def __enter__(self) -> "Keys":
        if self.fd is not None:
            self._saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc) -> None:
        if self._saved is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._saved)

    def pending(self) -> str:
        if self.fd is None:
            return ""
        out = ""
        while select.select([sys.stdin], [], [], 0)[0]:
            out += sys.stdin.read(1)
        return out


# --------------------------------------------------------------------------
# The actual experiment: what does the 16-bit position field do out there?
# --------------------------------------------------------------------------

class RangeProbe:
    """Classifies the MIT feedback position against the true one.

    Every sample is scored against the two plausible firmware behaviours:

        rail  : fb pinned at ±max_pos once |true| > max_pos
        wrap  : fb = ((true + max_pos) mod 2*max_pos) - max_pos

    Inside the window both predict the same thing, so only out-of-range
    samples are counted. The verdict is whichever hypothesis the drive's
    own echo actually matched.
    """

    def __init__(self, max_pos: float):
        self.max_pos = float(max_pos)
        # a rail is exact, so judge it tightly (1% of the window); the wrap
        # hypothesis lands half a window away, so it can be judged loosely.
        # Between 0.98 and 1.02 of the window the two predictions are within
        # feedback skew of each other — those samples are simply not scored.
        self.tol_rail = 0.01 * self.max_pos
        self.tol_wrap = 0.05 * self.max_pos
        self.n_in = 0
        self.n_edge = 0
        self.n_out = 0
        self.n_rail = 0
        self.n_wrap = 0
        self.n_other = 0
        self.err_in_max = 0.0              # |fb - true| while inside
        self.err_in_at_w = 0.0
        self.last_w = 0.0
        self.true_abs_max = 0.0
        self.n_nofb = 0
        self.t_left: Optional[float] = None
        self.left_at: Optional[float] = None
        self.last_flag = "-"

    @staticmethod
    def wrapped(x: float, m: float) -> float:
        return ((x + m) % (2.0 * m)) - m

    def feed(self, t: float, true_pos: float, fb_pos: Optional[float],
             w: float = 0.0) -> str:
        self.last_w = w
        self.true_abs_max = max(self.true_abs_max, abs(true_pos))
        if fb_pos is None:
            self.n_nofb += 1
            self.last_flag = "-"
            return self.last_flag
        if abs(true_pos) > self.max_pos * 0.98 \
                and abs(true_pos) < self.max_pos * 1.02:
            self.n_edge += 1                # ambiguous band, do not score
            return self.last_flag
        if abs(true_pos) <= self.max_pos * 0.98:
            self.n_in += 1
            if abs(fb_pos - true_pos) > self.err_in_max:
                self.err_in_max = abs(fb_pos - true_pos)
                self.err_in_at_w = self.last_w
            self.last_flag = "in"
            return self.last_flag
        if self.t_left is None:
            self.t_left, self.left_at = t, true_pos
        self.n_out += 1
        rail = math.copysign(self.max_pos, true_pos)
        if abs(fb_pos - rail) <= self.tol_rail:
            self.n_rail += 1
            self.last_flag = "RAIL"
        elif abs(fb_pos - self.wrapped(true_pos, self.max_pos)) <= self.tol_wrap:
            self.n_wrap += 1
            self.last_flag = "WRAP"
        else:
            self.n_other += 1
            self.last_flag = "??"
        return self.last_flag

    def verdict(self) -> str:
        if self.n_nofb and not (self.n_in or self.n_out):
            return ("no MIT feedback in this mode — only 0x08 echoes a "
                    "position, which is exactly the trade-off being measured")
        if not self.n_out:
            return ("never left the ±%.3g rad window — inconclusive; spin it "
                    "further (raise --turns-max / hold torque longer)"
                    % self.max_pos)
        best = max((self.n_rail, "rail"), (self.n_wrap, "wrap"),
                   (self.n_other, "neither"))[1]
        share = {"rail": self.n_rail, "wrap": self.n_wrap,
                 "neither": self.n_other}[best] / self.n_out
        text = {
            "rail": "the feedback position CLAMPS at ±mit_max_pos",
            "wrap": "the feedback position WRAPS modulo 2*mit_max_pos",
            "neither": "the feedback position does neither cleanly (look at "
                       "the CSV — quantisation, a different range, or the "
                       "drive redefining its own zero)",
        }[best]
        return f"{text} ({share * 100:.0f}% of {self.n_out} out-of-range samples)"


# --------------------------------------------------------------------------
# Bench
# --------------------------------------------------------------------------

class Bench:

    HEAD = ("    t    cmd_tau  out_rad   turns    w_true  mit_pos  mit_vel "
            " mit_tau   d_pos     iq   rtt_ms  state    flags")

    def __init__(self, motor, args, gim):
        self.m = motor
        self.a = args
        self.gim = gim                       # the gim6010_can module
        self.tau_goal = float(args.torque)   # what the keys change
        self.tau_cmd = 0.0                   # slew-limited, actually sent
        self.kd = float(args.kd)
        self.kp = float(args.kp)
        self.pos_mode = args.pos_field
        self.stop_reason = ""
        self.t0 = 0.0
        self.pos0_rev: Optional[float] = None
        self.probe: Optional[RangeProbe] = None
        self.rtts: List[float] = []
        self.echo_misses = 0
        self.echo_total = 0
        self.tau_fb_out: List[float] = []    # echoed torque while out of range
        self.tau_fb_in: List[float] = []
        self.iq_out: List[float] = []
        self.iq_in: List[float] = []
        self.w_abs_max = 0.0
        self._saved_rates: dict = {}
        # (t_mono, rotor rev, rotor rev/s) captured by the RX thread the
        # instant a 0x09 frame lands: reading state.pos and state.encoder_age()
        # separately from the control loop can straddle a broadcast and pair a
        # fresh position with a stale age
        self._enc: Optional[tuple] = None
        self._csv = None
        self._csvf = None
        self._rows = 0

    # -- units --------------------------------------------------------------

    def _on_encoder(self, s) -> None:
        self._enc = (time.monotonic(), s.pos, s.vel)

    def sample(self) -> Optional[tuple]:
        """(age s, out rad absolute, out rad/s, turns since start) from the
        last 0x09 broadcast — the true, unbounded multi-turn position."""
        enc = self._enc
        if enc is None or enc[1] is None or enc[2] is None:
            return None
        t_enc, p_rev, v_rev = enc
        turns = (None if self.pos0_rev is None
                 else (p_rev - self.pos0_rev) / self.a.ratio)
        return (time.monotonic() - t_enc, p_rev / self.a.ratio * TWO_PI,
                v_rev / self.a.ratio * TWO_PI, turns)

    # -- setup --------------------------------------------------------------

    def _read_pos_rev(self, timeout: float = 1.0) -> Optional[float]:
        """Rotor revolutions: RTR first, last broadcast as the fallback."""
        try:
            return self.m.get_encoder_estimates(timeout=timeout)[0]
        except self.gim.Timeout:
            s = self.m.state
            return s.pos if s.pos is not None and s.encoder_age() < 1.0 else None

    def preflight(self) -> None:
        m, a = self.m, self.a
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            if min(m.state.heartbeat_age(), m.state.encoder_age()) < 1.0:
                break
            time.sleep(0.05)
        else:
            raise RuntimeError(
                f"node {m.node_id} is silent (no heartbeat / encoder frames). "
                "Check power, bitrate + sample point (canup.sh), termination; "
                "verify with gim_cli.py scan")
        if m.state.heartbeat_age() > 1.0:
            # axis_state only ever arrives on the heartbeat, and enable()
            # waits for it — turn the broadcast back on rather than fail later
            print("  no heartbeat frames — enabling the 100 ms broadcast")
            m.set_feedback_rates(heartbeat=100)
            t0 = time.monotonic()
            while time.monotonic() - t0 < 2.0 and m.state.heartbeat_age() > 1.0:
                time.sleep(0.05)
            if m.state.heartbeat_age() > 1.0:
                raise RuntimeError(f"node {m.node_id} never heartbeats — "
                                   "cannot track axis state or errors")
        if m.state.any_error:
            names = self.gim.decode_flags(self.gim.AXIS_ERRORS,
                                          m.state.axis_error or 0)
            raise RuntimeError(f"node {m.node_id} reports {names or 'errors'} — "
                               "clear them first (gim_cli.py clear-errors)")

        # the drive quantises every MIT field over its own mit_max_* config;
        # read the real numbers so both ends decode the same frame the same way
        lim = m.mit_limits
        for attr, ep in (("max_pos", "mit_max_pos"), ("max_vel", "mit_max_vel"),
                         ("max_kp", "mit_max_kp"), ("max_kd", "mit_max_kd"),
                         ("max_torque", "mit_max_torque")):
            val = float(m.sdo_read(ep))
            if val <= 0:
                raise RuntimeError(f"drive reports {ep} = {val} — fix the "
                                   "MIT range config first")
            setattr(lim, attr, val)
        self.probe = RangeProbe(lim.max_pos)

        info = {}
        for ep in ("axis0.motor.config.torque_constant",
                   "axis0.motor.config.current_lim",
                   "axis0.controller.config.vel_limit",
                   "axis0.controller.config.enable_vel_limit",
                   "axis0.controller.config.enable_torque_mode_vel_limit"):
            try:
                info[ep.split(".")[-1]] = m.sdo_read(ep)
            except Exception as e:
                info[ep.split(".")[-1]] = f"?({type(e).__name__})"

        temp = m.state.temperature
        print(f"node {m.node_id}: {m.state.axis_state_name()}, "
              f"{'?' if temp is None else f'{temp}C'}, mode={self.a.mode}")
        print(f"  MIT ranges   pos ±{lim.max_pos:g} rad  vel ±{lim.max_vel:g} "
              f"rad/s  kp 0..{lim.max_kp:g}  kd 0..{lim.max_kd:g}  "
              f"torque ±{lim.max_torque:g} Nm")
        print(f"  -> the position window is ±{lim.max_pos / TWO_PI:.3f} output "
              f"turns (±{lim.max_pos / TWO_PI * self.a.ratio:.1f} rotor rev); "
              f"resolution {lim.max_pos * 2 / 65535 * 1e3:.3f} mrad")
        print(f"  -> torque quantum {lim.max_torque * 2 / 4095 * 1e3:.1f} mNm "
              f"— a {self.a.tau_max:g} Nm test uses "
              f"{self.a.tau_max / (lim.max_torque * 2 / 4095):.0f} of 4095 codes")
        print(f"  drive: torque_constant={info['torque_constant']} Nm/A, "
              f"current_lim={info['current_lim']} A, "
              f"vel_limit={info['vel_limit']} rev/s "
              f"(enable={info['enable_vel_limit']}, "
              f"torque_mode={info['enable_torque_mode_vel_limit']})")

        m.on_encoder = self._on_encoder
        try:
            self._saved_rates = m.get_feedback_rates()
        except Exception as e:
            print(f"  (could not read feedback rates: {e})")
        m.set_feedback_rates(encoder=a.encoder_ms, iq=20)
        m.set_deadman(a.deadman)

    def enable(self) -> None:
        m, a = self.m, self.a
        pos_rev = self._read_pos_rev()
        if pos_rev is None:
            raise RuntimeError("no position feedback — cannot enable")
        self.pos0_rev = pos_rev
        if self._enc is None:            # RTR answered but no broadcast yet
            self._enc = (time.monotonic(), pos_rev, 0.0)
        start = pos_rev / a.ratio * TWO_PI
        print(f"  start position {start:+.3f} rad output "
              f"({start / TWO_PI:+.3f} turns, {pos_rev:+.3f} rotor rev)")
        if abs(start) > 0.8 * m.mit_limits.max_pos:
            print(f"  NOTE: already at {abs(start) / m.mit_limits.max_pos:.0%} "
                  "of the MIT position window before we start — that is fine "
                  "for a torque test, it just means you cross the edge sooner")

        if a.mode == "mit":
            # POSITION + MIT input (user manual 3.3.4): with control_mode 1 the
            # firmware ignores the MIT gains and the frame produces no torque
            m.set_controller_mode(3, 9)
            m.mit_control(pos=start, vel=0.0, kp=0.0, kd=0.0, torque=0.0,
                          wait_feedback=False)
        elif a.mode == "torque":
            m.enable_torque_mode()
        else:
            m.enable_velocity_mode()
        m.set_axis_state(8)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.5:
            if m.state.axis_state == 8:
                break
            time.sleep(0.02)
        else:
            m.idle()
            raise RuntimeError("drive did not enter closed loop (state "
                               f"{m.state.axis_state_name()})")

    # -- command ------------------------------------------------------------

    def _pos_field(self, t: float, true_pos: float) -> float:
        if self.pos_mode == "track":
            return true_pos
        if self.pos_mode == "sweep":
            return self.a.sweep_rate * t
        return 0.0

    def _send(self, t: float, true_pos: float) -> Optional[tuple]:
        """One command frame. Returns the MIT echo (pos, vel, torque) or None."""
        a = self.a
        if a.mode == "mit":
            lim = self.m.mit_limits
            fb = self.m.mit_control(
                pos=self._pos_field(t, true_pos),
                vel=0.0,
                kp=min(self.kp, lim.max_kp),
                kd=min(self.kd, lim.max_kd),
                torque=self.tau_cmd,
                wait_feedback=not a.no_echo,
                timeout=a.echo_timeout)
            if not a.no_echo:
                self.echo_total += 1
            return fb
        if a.mode == "torque":
            # Set_Input_Torque is ROTOR frame; MIT is output frame
            self.m.set_input_torque(self.tau_cmd / a.ratio)
            return None
        self.m.set_input_vel(a.vel / TWO_PI * a.ratio, self.tau_cmd / a.ratio)
        return None

    # -- keys ---------------------------------------------------------------

    def _keys(self, ks: str) -> None:
        a = self.a
        for k in ks:
            if k in "+=":
                self.tau_goal += a.step
            elif k in "-_":
                self.tau_goal -= a.step
            elif k == "0":
                self.tau_goal = 0.0
            elif k == "r":
                self.tau_goal = -self.tau_goal
            elif k == " ":
                self.tau_goal = self.tau_cmd = 0.0
            elif k == "]":
                self.kd = min(self.kd + 0.05, self.m.mit_limits.max_kd)
            elif k == "[":
                self.kd = max(self.kd - 0.05, 0.0)
            elif k == "p":
                order = ["zero", "track", "sweep"]
                self.pos_mode = order[(order.index(self.pos_mode) + 1)
                                      % len(order)]
                print(f"\n  pos field -> {self.pos_mode}")
            elif k in "qQ\x03":
                self.stop_reason = "quit"
            self.tau_goal = min(max(self.tau_goal, -a.tau_max), a.tau_max)

    # -- loop ---------------------------------------------------------------

    def run(self) -> None:
        a = self.a
        dt = 1.0 / a.rate
        self.t0 = time.monotonic()
        next_t = self.t0
        next_print = self.t0
        misses = 0
        print()

        with Keys(not a.no_keys) as keys:
            while not self.stop_reason:
                now = time.monotonic()
                t = now - self.t0
                self._keys(keys.pending())
                if self.stop_reason:
                    break

                # --- watchdogs -------------------------------------------
                s = self.m.state
                if s.any_error:
                    names = self.gim.decode_flags(self.gim.AXIS_ERRORS,
                                                  s.axis_error or 0)
                    self.stop_reason = f"drive error {names or hex(s.axis_error or 0)}"
                    break
                if s.heartbeat_age() > 0.5:
                    self.stop_reason = "heartbeat lost"
                    break
                if s.encoder_age() > 0.2:
                    self.stop_reason = "stale encoder feedback"
                    break
                smp = self.sample()
                if smp is None:
                    self.stop_reason = "no feedback"
                    break
                age, pos, w, tn = smp
                self.w_abs_max = max(self.w_abs_max, abs(w))
                if abs(w) > a.vel_max:
                    self.stop_reason = f"overspeed {w:+.1f} rad/s"
                    break
                if tn is not None and abs(tn) > a.turns_max:
                    self.stop_reason = f"turn limit reached ({tn:+.2f} turns)"
                    break
                if t > a.duration:
                    self.stop_reason = f"duration {a.duration:g}s elapsed"
                    break

                # --- command ---------------------------------------------
                step = a.ramp * dt
                self.tau_cmd += min(max(self.tau_goal - self.tau_cmd, -step),
                                    step)
                self.tau_cmd = min(max(self.tau_cmd, -a.tau_max), a.tau_max)
                t_tx = time.monotonic()
                try:
                    fb = self._send(t, pos)
                except self.gim.Timeout:
                    fb = None
                    misses += 1
                    self.echo_misses += 1
                    if misses > a.echo_misses_max:
                        self.stop_reason = (f"{misses} consecutive MIT echoes "
                                            "missing")
                        break
                except Exception as e:
                    self.stop_reason = (f"CAN TX failed: {type(e).__name__}: "
                                        f"{str(e).splitlines()[0]}")
                    break
                else:
                    if fb is not None:
                        misses = 0
                        self.rtts.append((time.monotonic() - t_tx) * 1e3)

                # --- record ----------------------------------------------
                fb_pos = fb[0] if fb else self.m.state.mit_pos
                fb_vel = fb[1] if fb else self.m.state.mit_vel
                fb_tau = fb[2] if fb else self.m.state.mit_torque
                # the echo is fresh, the 0x09 broadcast is up to encoder_ms
                # old: extrapolate it forward or the comparison measures our
                # own sampling skew (w * 5 ms = 75 mrad at 15 rad/s) instead
                # of what the 16-bit field does
                pos_now = pos + w * min(age, 0.05)
                was_out = self.probe.last_flag in ("RAIL", "WRAP", "??")
                flag = self.probe.feed(t, pos_now, fb_pos, w)
                out = flag in ("RAIL", "WRAP", "??")
                if out and not was_out:
                    self._crossing(t, pos_now, fb_pos, w)
                # only score torque once the slew ramp has caught up, or the
                # ramp itself shows up as an inside/outside difference
                if abs(self.tau_cmd) > 1e-3 \
                        and abs(self.tau_cmd - self.tau_goal) < 1e-9:
                    (self.tau_fb_out if out else self.tau_fb_in).append(
                        fb_tau if fb_tau is not None else float("nan"))
                    if self.m.state.iq_measured is not None:
                        (self.iq_out if out else self.iq_in).append(
                            self.m.state.iq_measured)
                self._log(t, pos, tn, w, fb_pos, fb_vel, fb_tau, flag,
                          pos_now, age)
                if now >= next_print:
                    next_print += 1.0 / a.print_hz
                    self._print(t, pos, tn, w, fb_pos, fb_vel, fb_tau, flag)

                next_t += dt
                delay = next_t - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_t = time.monotonic()

    def _crossing(self, t: float, pos: float, fb_pos: Optional[float],
                  w: float) -> None:
        lim = self.m.mit_limits
        print(f"\n  *** t={t:.2f}s: left the MIT position window at "
              f"{pos:+.3f} rad ({pos / TWO_PI:+.2f} turns, {w:+.2f} rad/s) — "
              f"echo now reads {fb_pos:+.3f} of ±{lim.max_pos:g}; "
              f"torque commanded {self.tau_cmd:+.3f} Nm, kp={self.kp:g}\n")
        self._rows = 0

    # -- output -------------------------------------------------------------

    def open_csv(self) -> None:
        if not self.a.csv:
            return
        self._csvf = open(self.a.csv, "w", newline="")
        self._csv = csv.writer(self._csvf)
        self._csv.writerow(
            ["t", "tau_cmd_nm", "kp", "kd", "pos_field", "out_rad_abs",
             "turns_rel", "w_true_rad_s", "mit_pos", "mit_vel", "mit_torque",
             "dpos_raw", "dpos_adj", "enc_age_ms", "iq_meas", "rtt_ms",
             "axis_state", "axis_error", "flag"])

    def _log(self, t, pos, tn, w, fb_pos, fb_vel, fb_tau, flag,
             pos_now=None, age=0.0) -> None:
        if self._csv is None:
            return
        s = self.m.state
        self._csv.writerow([
            f"{t:.4f}", f"{self.tau_cmd:.4f}", f"{self.kp:g}", f"{self.kd:g}",
            self.pos_mode, f"{pos:.6f}", "" if tn is None else f"{tn:.6f}",
            f"{w:.4f}",
            "" if fb_pos is None else f"{fb_pos:.6f}",
            "" if fb_vel is None else f"{fb_vel:.4f}",
            "" if fb_tau is None else f"{fb_tau:.4f}",
            "" if fb_pos is None else f"{fb_pos - pos:.6f}",
            "" if fb_pos is None or pos_now is None
            else f"{fb_pos - pos_now:.6f}",
            f"{age * 1e3:.2f}",
            "" if s.iq_measured is None else f"{s.iq_measured:.3f}",
            f"{self.rtts[-1]:.3f}" if self.rtts else "",
            s.axis_state, s.axis_error, flag])

    def _print(self, t, pos, tn, w, fb_pos, fb_vel, fb_tau, flag) -> None:
        if self._rows % 20 == 0:
            print(self.HEAD)
        self._rows += 1
        f = lambda v, sp="+8.3f": "       -" if v is None else format(v, sp)
        s = self.m.state
        print(f"{t:6.2f} {self.tau_cmd:+7.3f} {f(pos)} {f(tn)} {f(w)} "
              f"{f(fb_pos)} {f(fb_vel)} {f(fb_tau)} "
              f"{f(None if fb_pos is None else fb_pos - pos)} "
              f"{f(s.iq_measured, '+6.2f')} "
              f"{format(self.rtts[-1], '6.2f') if self.rtts else '     -'} "
              f"{s.axis_state_name()[:7]:7s} {flag}")

    # -- teardown -----------------------------------------------------------

    def shutdown(self) -> None:
        """Best effort and ordered: the later steps matter more, so a failing
        send must never skip them."""
        m, a = self.m, self.a
        steps = []
        if a.mode == "mit":
            steps.append(("mit zero", lambda: m.mit_control(
                0.0, 0.0, 0.0, 0.0, 0.0, wait_feedback=False)))
        steps += [("torque 0", lambda: m.set_input_torque(0.0)),
                  ("vel 0", lambda: m.set_input_vel(0.0)),
                  ("idle", m.idle)]
        if m.state.heartbeat_age() < 1.0:
            steps.append(("deadman off", lambda: m.set_deadman(0)))
            for k, v in (self._saved_rates or {}).items():
                if k in ("encoder", "iq", "heartbeat"):
                    steps.append((f"rate {k}", lambda k=k, v=v:
                                  m.set_feedback_rates(**{k: v})))
        for name, fn in steps:
            try:
                fn()
            except Exception as e:
                print(f"  shutdown: {name} failed: "
                      f"{type(e).__name__}: {str(e).splitlines()[0]}")
        if self._csvf:
            self._csvf.close()

    def summary(self) -> None:
        a, p = self.a, self.probe
        lim = self.m.mit_limits
        mean = lambda v: sum(v) / len(v) if v else float("nan")
        print("\n" + "=" * 78)
        print(f"stopped: {self.stop_reason or 'clean exit'}")
        print(f"  ran {time.monotonic() - self.t0:.1f}s in {a.mode} mode, "
              f"kp={self.kp:g} kd={self.kd:g}, |w|max {self.w_abs_max:.2f} rad/s"
              f" ({self.w_abs_max / TWO_PI * 60:.0f} output rpm)")
        if p is None:
            return
        print(f"  reached |pos| {p.true_abs_max:.3f} rad = "
              f"{p.true_abs_max / TWO_PI:.2f} output turns "
              f"(window ±{lim.max_pos:g} rad = ±{lim.max_pos / TWO_PI:.2f} turns)")
        if p.n_in:
            skew = (" (--no-echo: the echo is a cycle stale, so most of this "
                    "is sampling skew)" if a.no_echo else "")
            print(f"  inside the window the echo tracked the true position to "
                  f"{p.err_in_max * 1e3:.2f} mrad worst case at "
                  f"{p.err_in_at_w:+.1f} rad/s ({p.n_in} samples; the field's "
                  f"own quantum is {lim.max_pos * 2 / 65535 * 1e3:.3f} mrad)"
                  + skew)
        print(f"  VERDICT: {p.verdict()}")
        if self.rtts:
            r = sorted(self.rtts)
            print(f"  MIT round trip: min {r[0]:.2f} / median "
                  f"{r[len(r) // 2]:.2f} / p99 {r[int(len(r) * 0.99)]:.2f} / "
                  f"max {r[-1]:.2f} ms over {len(r)} frames, "
                  f"{self.echo_misses} missed "
                  f"({self.echo_misses / max(1, self.echo_total) * 100:.2f}%)")
        if self.tau_fb_out:
            print(f"  echoed torque: {mean(self.tau_fb_in):+.3f} Nm inside vs "
                  f"{mean(self.tau_fb_out):+.3f} Nm outside the window "
                  f"(commanded up to {a.tau_max:g})")
        if self.iq_out:
            print(f"  measured Iq:   {mean(self.iq_in):+.2f} A inside vs "
                  f"{mean(self.iq_out):+.2f} A outside — if these match, the "
                  "drive kept producing the commanded torque past the range")
        elif p.n_out and self.kp == 0.0:
            print("  (no Iq samples — enable the iq broadcast to confirm the "
                  "torque itself, not just the echo)")
        if p.n_out and self.kp == 0.0:
            print("  note: kp was 0, so the position field could not influence "
                  "torque by construction — this is the configuration a "
                  "continuous wheel would use.")
        if self.a.csv:
            print(f"  samples written to {self.a.csv}")
        print("=" * 78)


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="keys: + - 0 r space [ ] p q")
    ap.add_argument("node", type=int, help="CAN node id of the drive to test")
    ap.add_argument("--channel", "-c", default="can0")
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--sim", action="store_true",
                    help="emulated drive (sim_motor.py) instead of hardware")
    ap.add_argument("--can-dir", type=Path, default=DEFAULT_CAN_DIR,
                    help="folder containing gim6010_can.py")
    ap.add_argument("--ratio", type=float, default=8.0,
                    help="rotor:output gear ratio, only used to turn the "
                         "0x09 rotor revolutions into output radians")
    ap.add_argument("--mode", choices=("mit", "torque", "velocity"),
                    default="mit",
                    help="mit: 0x08 feedforward torque (default, synchronous); "
                         "torque: Set_Input_Torque; velocity: Set_Input_Vel — "
                         "the last two for comparison, neither echoes state")

    g = ap.add_argument_group("command")
    g.add_argument("--torque", type=float, default=0.0,
                   help="initial feedforward torque, output Nm (default 0: "
                        "nothing moves until you press +)")
    g.add_argument("--kp", type=float, default=0.0,
                   help="MIT position gain, Nm/rad. 0 (default) makes the "
                        "position field irrelevant — that is the point")
    g.add_argument("--kd", type=float, default=0.0,
                   help="MIT damping, Nm/(rad/s) — velocity control without "
                        "touching the position field")
    g.add_argument("--vel", type=float, default=0.0,
                   help="--mode velocity target, output rad/s")
    g.add_argument("--pos-field", choices=("zero", "track", "sweep"),
                   default="zero", help="what to put in the MIT position "
                                        "field (ignored while kp = 0)")
    g.add_argument("--sweep-rate", type=float, default=1.0,
                   help="--pos-field sweep ramp, rad/s")
    g.add_argument("--step", type=float, default=0.05,
                   help="torque change per +/- keypress, Nm")
    g.add_argument("--ramp", type=float, default=1.0,
                   help="torque slew limit, Nm/s")

    s = ap.add_argument_group("safety")
    s.add_argument("--tau-max", type=float, default=1.0,
                   help=f"torque clamp, output Nm (hard ceiling {TAU_HARD_MAX})")
    s.add_argument("--vel-max", type=float, default=30.0,
                   help="stop above this output speed, rad/s")
    s.add_argument("--turns-max", type=float, default=25.0,
                   help="stop after this many output turns from the start")
    s.add_argument("--duration", type=float, default=120.0,
                   help="stop after this many seconds")
    s.add_argument("--deadman", type=int, default=200,
                   help="drive-side break_timeout while running, ms (0 off)")
    s.add_argument("--echo-misses-max", type=int, default=10,
                   help="stop after this many consecutive missing MIT echoes")

    o = ap.add_argument_group("loop / output")
    o.add_argument("--rate", type=float, default=200.0, help="command Hz")
    o.add_argument("--print-hz", type=float, default=5.0)
    o.add_argument("--encoder-ms", type=int, default=5,
                   help="drive pos/vel broadcast period")
    o.add_argument("--echo-timeout", type=float, default=0.02,
                   help="how long to wait for the MIT echo, s")
    o.add_argument("--no-echo", action="store_true",
                   help="fire-and-forget MIT frames; read the echo from the "
                        "broadcast stream instead of waiting for it")
    o.add_argument("--no-keys", action="store_true",
                   help="do not put the terminal in cbreak mode")
    o.add_argument("--csv", type=Path, help="write every sample here")
    args = ap.parse_args(argv)

    if args.tau_max > TAU_HARD_MAX:
        ap.error(f"--tau-max above the {TAU_HARD_MAX} Nm ceiling")
    if abs(args.torque) > args.tau_max:
        ap.error("--torque exceeds --tau-max")

    sys.path.insert(0, str(args.can_dir))
    import can
    import gim6010_can as gim

    sim = None
    if args.sim:
        from sim_motor import SimMotor
        sim = SimMotor(can.Bus(interface="virtual", channel="wheelsim",
                               receive_own_messages=False), node_id=args.node)
        host = can.Bus(interface="virtual", channel="wheelsim",
                       receive_own_messages=False)
        print(f"SIM: emulated drive on node {args.node} "
              "(its MIT echo is the simulator's guess, not firmware)")
    else:
        host = can.Bus(channel=args.channel, interface=args.interface)

    eps = args.can_dir.parent / "manual" / "assets_protocol_manual" \
        / "endpoints_v3.json"
    bus = gim.GIM6010Bus(bus=host, endpoints=eps if eps.exists() else None)
    m = bus.motor(args.node)

    bench = Bench(m, args, gim)
    signal.signal(signal.SIGINT,
                  lambda *_: setattr(bench, "stop_reason", "SIGINT"))
    signal.signal(signal.SIGTERM,
                  lambda *_: setattr(bench, "stop_reason", "SIGTERM"))
    rc = 0
    try:
        bench.preflight()
        if bench.stop_reason:
            raise RuntimeError(f"interrupted before enabling ({bench.stop_reason})")
        bench.open_csv()
        bench.enable()
        bench.run()
    except (RuntimeError, gim.Timeout) as e:
        print(f"\nERROR: {e}", file=sys.stderr)
        bench.stop_reason = bench.stop_reason or str(e)
        rc = 1
    except Exception as e:
        print(f"\nERROR: {type(e).__name__}: {e}", file=sys.stderr)
        bench.stop_reason = bench.stop_reason or f"{type(e).__name__}: {e}"
        rc = 1
    finally:
        bench.shutdown()
        if bench.t0:
            bench.summary()
        if sim:
            sim.stop()
            try:
                sim.bus.shutdown()
            except Exception:
                pass
        bus.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
