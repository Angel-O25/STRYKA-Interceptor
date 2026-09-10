#!/usr/bin/env python3
"""STRYKA Gate 0 — first-flight command-path proof.

Proves the full command path end to end against ArduPilot SITL *before* any ROS 2
complexity exists: connect, wait for a healthy EKF, switch to GUIDED, arm, take
off, fly a 20 m square (north, east, south, west), land, disarm.

The real point of this script is the *frame check*. Our whole stack is built on
the rule that we emit world-frame velocity commands. ArduPilot speaks NED
(x=North, y=East, z=Down); ROS/MAVROS speaks ENU (x=East, y=North, z=Up). If the
two get crossed, "go north" silently becomes "go east". This script commands
each leg and then *verifies* from LOCAL_POSITION_NED that the vehicle actually
moved the way it was told. A silent wrong turn cannot pass.

Dependencies: Python 3 standard library + `pymavlink` (no ROS, no MAVROS, no
config files). Install with:  pip install pymavlink

Run:  python3 tools/first_flight.py [--connect udp:127.0.0.1:14550]

Exit code 0 on success, non-zero on any failure.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Callable, Optional

from pymavlink import mavutil

# --------------------------------------------------------------------------- #
# Tunable constants — every magic number lives here, nothing scattered below.
# --------------------------------------------------------------------------- #

DEFAULT_ENDPOINT: str = "udp:127.0.0.1:14550"

TARGET_ALTITUDE_M: float = 10.0          # take-off target above home
ALTITUDE_TOLERANCE_M: float = 0.5        # "altitude reached" band

LEG_LENGTH_M: float = 20.0              # side length of the square
LEG_SPEED_MS: float = 5.0              # cruise speed along a leg

# ArduPilot leaves the active GUIDED/offboard mode *silently* if the setpoint
# stream starves. We therefore re-send the velocity setpoint on a fixed timer
# for the entire duration of every leg (and every settle), never once per leg.
SETPOINT_RATE_HZ: float = 10.0
SETPOINT_PERIOD_S: float = 1.0 / SETPOINT_RATE_HZ

# Leg completion / settle thresholds.
ARRIVAL_TOLERANCE_M: float = 1.0         # within this of the leg end == arrived
SETTLE_SPEED_MS: float = 0.3            # ground speed that counts as "stopped"
SETTLE_TIMEOUT_S: float = 15.0

# Frame-convention verification. On a correct leg the commanded axis should
# carry almost all of the displacement and the cross axis almost none.
PROGRESS_MIN_FRACTION: float = 0.70      # commanded axis must cover >= 70% of leg
CROSS_TRACK_TOLERANCE_M: float = 5.0     # allowed drift on the *uncommanded* axis

# Phase timeouts (seconds).
HEARTBEAT_TIMEOUT_S: float = 15.0
ARMABLE_TIMEOUT_S: float = 120.0
MODE_CHANGE_TIMEOUT_S: float = 10.0
ARM_TIMEOUT_S: float = 10.0
TAKEOFF_TIMEOUT_S: float = 60.0
LEG_TIMEOUT_S: float = 40.0
LAND_TIMEOUT_S: float = 120.0

# EKF health bits we require before we are willing to arm. Poll for these
# rather than sleeping — "armable" is a state, not a duration.
EKF_FLAGS_REQUIRED: int = (
    mavutil.mavlink.EKF_ATTITUDE
    | mavutil.mavlink.EKF_VELOCITY_HORIZ
    | mavutil.mavlink.EKF_POS_HORIZ_ABS
    | mavutil.mavlink.EKF_PRED_POS_HORIZ_ABS
)
GPS_FIX_MIN: int = 3                     # 3 == 3D fix

# SET_POSITION_TARGET_LOCAL_NED type mask: ignore position, acceleration, yaw
# and yaw-rate; use velocity only. Bit set == field ignored.
VELOCITY_TYPE_MASK: int = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

# Messages we want streamed at a usable rate for the whole flight.
STREAMED_MESSAGES: tuple[tuple[int, float], ...] = (
    (mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 4.0),
    (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, SETPOINT_RATE_HZ),
    (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2.0),
    (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2.0),
    (mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 2.0),
)


# --------------------------------------------------------------------------- #
# Small helpers
# --------------------------------------------------------------------------- #

_T0: float = time.monotonic()


def log(message: str) -> None:
    """Print a wall-clock + mission-elapsed timestamped line (flushed)."""
    elapsed = time.monotonic() - _T0
    print(f"[{time.strftime('%H:%M:%S')}] [T+{elapsed:7.1f}s] {message}", flush=True)


class FlightError(RuntimeError):
    """Any condition that means Gate 0 has failed."""


@dataclass(frozen=True)
class NedSample:
    """A single LOCAL_POSITION_NED reading (metres, metres/second)."""

    north: float
    east: float
    down: float
    vnorth: float
    veast: float
    vdown: float

    @property
    def altitude(self) -> float:
        """Height above the local origin (positive up)."""
        return -self.down

    @property
    def ground_speed(self) -> float:
        """Horizontal speed magnitude."""
        return math.hypot(self.vnorth, self.veast)


@dataclass
class LegResult:
    """Outcome of one leg, kept for the final summary block."""

    name: str
    delta_north: float
    delta_east: float


# --------------------------------------------------------------------------- #
# MAVLink plumbing
# --------------------------------------------------------------------------- #


class Vehicle:
    """Thin wrapper over a pymavlink connection with the helpers Gate 0 needs."""

    def __init__(self, connection: mavutil.mavfile) -> None:
        self._conn = connection

    # -- lifecycle -------------------------------------------------------- #

    @classmethod
    def connect(cls, endpoint: str) -> "Vehicle":
        """Open the link and wait for the first heartbeat."""
        log(f"connecting to {endpoint}")
        conn = mavutil.mavlink_connection(endpoint, source_system=255)
        vehicle = cls(conn)
        vehicle._await_heartbeat()
        vehicle._request_streams()
        return vehicle

    def _await_heartbeat(self) -> None:
        hb = self._conn.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S)
        if hb is None:
            raise FlightError("no HEARTBEAT received — is SITL running?")
        autopilot = mavutil.mavlink.enums["MAV_AUTOPILOT"][hb.autopilot].name
        vtype = mavutil.mavlink.enums["MAV_TYPE"][hb.type].name
        log(
            f"heartbeat OK — autopilot={autopilot} vehicle={vtype} "
            f"system_id={self._conn.target_system} component_id={self._conn.target_component}"
        )

    def _request_streams(self) -> None:
        """Ask the autopilot to stream the messages we poll on."""
        for msg_id, rate_hz in STREAMED_MESSAGES:
            self._conn.mav.command_long_send(
                self._conn.target_system,
                self._conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(msg_id),
                1e6 / rate_hz,  # interval in microseconds
                0, 0, 0, 0, 0,
            )

    # -- reads ---------------------------------------------------------- #

    def recv(self, msg_type: str, timeout: float) -> Optional[object]:
        """Blocking recv_match for a single message type."""
        return self._conn.recv_match(type=msg_type, blocking=True, timeout=timeout)

    def latest_ned(self, timeout: float = 2.0) -> NedSample:
        """Return the freshest LOCAL_POSITION_NED, draining any backlog."""
        msg = self.recv("LOCAL_POSITION_NED", timeout)
        if msg is None:
            raise FlightError("no LOCAL_POSITION_NED — position stream lost")
        # Drain anything already queued so callers act on the newest sample.
        while True:
            newer = self._conn.recv_match(type="LOCAL_POSITION_NED", blocking=False)
            if newer is None:
                break
            msg = newer
        return NedSample(
            north=msg.x, east=msg.y, down=msg.z,
            vnorth=msg.vx, veast=msg.vy, vdown=msg.vz,
        )

    def wait_heartbeat_state(
        self,
        predicate: Callable[[object], bool],
        timeout: float,
        description: str,
    ) -> None:
        """Poll HEARTBEAT until *predicate* holds, or fail after *timeout*."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            hb = self.recv("HEARTBEAT", timeout=1.0)
            if hb is not None and predicate(hb):
                return
        raise FlightError(f"timed out waiting for: {description}")

    # -- commands ------------------------------------------------------- #

    def _send_command(self, command: int, *params: float) -> None:
        padded = list(params) + [0.0] * (7 - len(params))
        self._conn.mav.command_long_send(
            self._conn.target_system,
            self._conn.target_component,
            command,
            0,
            *padded,
        )

    def _await_ack(self, command: int, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ack = self.recv("COMMAND_ACK", timeout=1.0)
            if ack is None or ack.command != command:
                continue
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                return
            name = mavutil.mavlink.enums["MAV_RESULT"][ack.result].name
            raise FlightError(f"command {command} rejected: {name}")
        raise FlightError(f"no COMMAND_ACK for command {command}")

    def wait_until_armable(self, timeout: float) -> None:
        """Block until the EKF is healthy and GPS has a 3D fix.

        We poll for the *state* — never a fixed sleep — because "safe to arm"
        depends on filter convergence, not elapsed time.
        """
        deadline = time.monotonic() + timeout
        ekf_ok = False
        gps_ok = False
        while time.monotonic() < deadline:
            msg = self._conn.recv_match(
                type=["EKF_STATUS_REPORT", "GPS_RAW_INT"], blocking=True, timeout=1.0
            )
            if msg is None:
                continue
            if msg.get_type() == "EKF_STATUS_REPORT":
                ekf_ok = (msg.flags & EKF_FLAGS_REQUIRED) == EKF_FLAGS_REQUIRED
            elif msg.get_type() == "GPS_RAW_INT":
                gps_ok = msg.fix_type >= GPS_FIX_MIN
            if ekf_ok and gps_ok:
                log("EKF healthy and GPS 3D fix — vehicle is armable")
                return
        raise FlightError(
            f"vehicle not armable within {timeout:.0f}s (ekf_ok={ekf_ok}, gps_ok={gps_ok})"
        )

    def set_mode(self, mode_name: str, timeout: float) -> None:
        """Command a flight-mode change and confirm it via HEARTBEAT read-back."""
        mapping = self._conn.mode_mapping() or {}
        if mode_name not in mapping:
            raise FlightError(f"autopilot has no mode {mode_name!r}")
        mode_id = mapping[mode_name]
        log(f"requesting mode {mode_name}")
        self._conn.mav.set_mode_send(
            self._conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        self.wait_heartbeat_state(
            lambda hb: hb.custom_mode == mode_id,
            timeout,
            f"mode == {mode_name}",
        )
        log(f"mode confirmed: {mode_name}")

    def arm(self, timeout: float) -> None:
        """Arm and confirm the armed flag is set in HEARTBEAT."""
        log("arming")
        self._send_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0)
        self._await_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout)
        self.wait_heartbeat_state(
            lambda hb: bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
            timeout,
            "armed state",
        )
        log("armed (confirmed via HEARTBEAT)")

    def wait_disarm(self, timeout: float) -> None:
        """Block until the autopilot reports disarmed."""
        self.wait_heartbeat_state(
            lambda hb: not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
            timeout,
            "disarmed state",
        )
        log("disarmed (confirmed via HEARTBEAT)")

    def takeoff(self, altitude_m: float, timeout: float) -> None:
        """Take off to *altitude_m* and wait until we are within tolerance."""
        log(f"takeoff to {altitude_m:.1f} m")
        self._send_command(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude_m
        )
        self._await_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, timeout)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            sample = self.latest_ned()
            if abs(sample.altitude - altitude_m) <= ALTITUDE_TOLERANCE_M:
                log(f"altitude reached: {sample.altitude:.2f} m")
                return
        raise FlightError(f"did not reach {altitude_m:.1f} m within {timeout:.0f}s")

    def land(self, timeout: float) -> None:
        """Command LAND and wait for disarm."""
        log("landing")
        self.set_mode("LAND", MODE_CHANGE_TIMEOUT_S)
        self.wait_disarm(timeout)

    # -- setpoint streaming ------------------------------------------- #

    def send_velocity(self, vnorth: float, veast: float, vdown: float) -> None:
        """Emit one world-frame (NED) velocity setpoint.

        World-frame velocity only — never attitude, thrust or motor commands.
        This is the governing design rule for the whole STRYKA stack.
        """
        self._conn.mav.set_position_target_local_ned_send(
            0,                                   # time_boot_ms (autopilot ignores)
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            VELOCITY_TYPE_MASK,
            0.0, 0.0, 0.0,                        # position (ignored)
            vnorth, veast, vdown,                 # velocity (used)
            0.0, 0.0, 0.0,                        # acceleration (ignored)
            0.0, 0.0,                             # yaw, yaw_rate (ignored)
        )

    def stream_velocity_until(
        self,
        vnorth: float,
        veast: float,
        vdown: float,
        done: Callable[[NedSample], bool],
        timeout: float,
    ) -> NedSample:
        """Publish (vnorth, veast, vdown) at SETPOINT_RATE_HZ until *done*.

        The stream must not stop: ArduPilot drops out of GUIDED without warning
        if setpoints starve, so we re-send on every tick for the whole leg.
        """
        deadline = time.monotonic() + timeout
        last: Optional[NedSample] = None
        while time.monotonic() < deadline:
            self.send_velocity(vnorth, veast, vdown)
            last = self.latest_ned()
            if done(last):
                return last
            time.sleep(SETPOINT_PERIOD_S)
        raise FlightError(
            f"leg/settle timed out after {timeout:.0f}s "
            f"(last sample: {last})"
        )


# --------------------------------------------------------------------------- #
# The square, and the frame check that is the point of it
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class Leg:
    """One side of the square: a name, a velocity vector, and what to verify."""

    name: str
    vnorth: float
    veast: float
    axis: str          # "north" or "east" — the axis that should dominate
    sign: int          # +1 or -1 — expected direction on that axis


def build_legs() -> tuple[Leg, ...]:
    """North, then east, then south, then west."""
    v = LEG_SPEED_MS
    return (
        Leg("north", +v, 0.0, "north", +1),
        Leg("east", 0.0, +v, "east", +1),
        Leg("south", -v, 0.0, "north", -1),
        Leg("west", 0.0, -v, "east", -1),
    )


def verify_leg(leg: Leg, start: NedSample, end: NedSample) -> LegResult:
    """Assert the vehicle moved the way it was told; raise loudly if not.

    A silent wrong turn is the exact failure mode Gate 0 exists to catch, so
    this check is deliberately strict: the commanded axis must carry the great
    majority of the motion in the expected direction, and the cross axis must
    stay near zero.
    """
    delta_north = end.north - start.north
    delta_east = end.east - start.east

    primary = delta_north if leg.axis == "north" else delta_east
    cross = delta_east if leg.axis == "north" else delta_north
    cross_axis = "east" if leg.axis == "north" else "north"

    log(
        f"leg {leg.name}: measured NED delta = "
        f"dNorth {delta_north:+.2f} m, dEast {delta_east:+.2f} m"
    )

    expected_progress = PROGRESS_MIN_FRACTION * LEG_LENGTH_M
    directed_progress = primary * leg.sign  # positive if we went the right way

    wrong_direction = directed_progress < expected_progress
    excess_cross_track = abs(cross) > CROSS_TRACK_TOLERANCE_M

    if wrong_direction or excess_cross_track:
        want = f"{'+' if leg.sign > 0 else '-'}{leg.axis}"
        print("", flush=True)
        print("=" * 72, flush=True)
        print("  FRAME-CONVENTION FAULT SUSPECTED — ENU/NED AXIS CONFUSION", flush=True)
        print("=" * 72, flush=True)
        print(f"  Leg            : {leg.name}", flush=True)
        print(f"  Commanded      : velocity N={leg.vnorth:+.1f} E={leg.veast:+.1f} m/s"
              f"  (expect motion {want})", flush=True)
        print(f"  Measured delta : dNorth {delta_north:+.2f} m, dEast {delta_east:+.2f} m",
              flush=True)
        if wrong_direction:
            print(f"  --> commanded {leg.axis} axis moved {directed_progress:+.2f} m, "
                  f"need >= {expected_progress:.2f} m in the commanded direction.", flush=True)
        if excess_cross_track:
            print(f"  --> uncommanded {cross_axis} axis moved {cross:+.2f} m, "
                  f"tolerance is +/-{CROSS_TRACK_TOLERANCE_M:.2f} m.", flush=True)
        print("", flush=True)
        print("  Most likely cause: NED (x=North, ArduPilot) has been crossed with", flush=True)
        print("  ENU (x=East, ROS/MAVROS). A commanded 'north' has become 'east'", flush=True)
        print("  (or a sign has flipped). Fix the frame mapping before trusting any", flush=True)
        print("  velocity command downstream.", flush=True)
        print("=" * 72, flush=True)
        raise FlightError(f"leg {leg.name}: frame-convention verification failed")

    log(f"leg {leg.name}: frame check PASSED "
        f"(commanded axis {directed_progress:+.2f} m, cross-track {cross:+.2f} m)")
    return LegResult(leg.name, delta_north, delta_east)


def fly_leg(vehicle: Vehicle, leg: Leg) -> LegResult:
    """Fly one leg: record start, cruise until the distance is made, settle."""
    start = vehicle.latest_ned()
    log(f"leg {leg.name} START at NED north={start.north:.2f} east={start.east:.2f} "
        f"alt={start.altitude:.2f}")

    def far_enough(sample: NedSample) -> bool:
        moved = math.hypot(sample.north - start.north, sample.east - start.east)
        return moved >= (LEG_LENGTH_M - ARRIVAL_TOLERANCE_M)

    vehicle.stream_velocity_until(
        leg.vnorth, leg.veast, 0.0, far_enough, LEG_TIMEOUT_S
    )

    # Stop and let the vehicle settle — still streaming zero velocity so we do
    # not drop out of GUIDED between legs.
    def stopped(sample: NedSample) -> bool:
        return sample.ground_speed <= SETTLE_SPEED_MS

    end = vehicle.stream_velocity_until(0.0, 0.0, 0.0, stopped, SETTLE_TIMEOUT_S)
    log(f"leg {leg.name} END at NED north={end.north:.2f} east={end.east:.2f} "
        f"alt={end.altitude:.2f}")

    return verify_leg(leg, start, end)


# --------------------------------------------------------------------------- #
# Orchestration
# --------------------------------------------------------------------------- #


def run(endpoint: str) -> int:
    """Execute the full Gate 0 sequence. Return a process exit code."""
    flight_start = time.monotonic()
    legs_flown: list[LegResult] = []

    try:
        vehicle = Vehicle.connect(endpoint)

        vehicle.wait_until_armable(ARMABLE_TIMEOUT_S)
        vehicle.set_mode("GUIDED", MODE_CHANGE_TIMEOUT_S)
        vehicle.arm(ARM_TIMEOUT_S)
        vehicle.takeoff(TARGET_ALTITUDE_M, TAKEOFF_TIMEOUT_S)

        # Prime the setpoint stream before the first leg.
        for _ in range(int(SETPOINT_RATE_HZ)):
            vehicle.send_velocity(0.0, 0.0, 0.0)
            time.sleep(SETPOINT_PERIOD_S)

        for leg in build_legs():
            legs_flown.append(fly_leg(vehicle, leg))

        vehicle.land(LAND_TIMEOUT_S)

    except FlightError as exc:
        log(f"FAILURE: {exc}")
        _print_summary(flight_start, legs_flown, passed=False)
        return 1
    except KeyboardInterrupt:
        log("FAILURE: interrupted by user")
        return 130

    _print_summary(flight_start, legs_flown, passed=True)
    return 0


def _print_summary(
    flight_start: float, legs: list[LegResult], passed: bool
) -> None:
    """Print the Gate 0 evidence block."""
    total = time.monotonic() - flight_start
    print("", flush=True)
    print("-" * 72, flush=True)
    print("  STRYKA GATE 0 — FIRST FLIGHT SUMMARY", flush=True)
    print("-" * 72, flush=True)
    print(f"  Total flight time : {total:.1f} s", flush=True)
    print("  Measured leg deltas (NED, metres):", flush=True)
    for result in legs:
        print(
            f"    {result.name:<6} dNorth {result.delta_north:+7.2f}   "
            f"dEast {result.delta_east:+7.2f}",
            flush=True,
        )
    for missing in ("north", "east", "south", "west"):
        if missing not in {r.name for r in legs}:
            print(f"    {missing:<6} (not flown)", flush=True)
    print("-" * 72, flush=True)
    print(f"  RESULT: {'PASS' if passed else 'FAIL'}", flush=True)
    print("-" * 72, flush=True)


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="STRYKA Gate 0 first-flight command-path proof (SITL)."
    )
    parser.add_argument(
        "--connect",
        default=DEFAULT_ENDPOINT,
        help=f"MAVLink endpoint (default: {DEFAULT_ENDPOINT})",
    )
    return parser.parse_args(argv)


def main() -> None:
    args = parse_args()
    sys.exit(run(args.connect))


if __name__ == "__main__":
    main()
