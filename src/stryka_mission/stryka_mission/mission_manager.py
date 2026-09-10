"""STRYKA mission manager, v1 -- the Gate 0 square, flown from a ROS 2 node.

This node owns the highest-risk interface in the system: the boundary between
our autonomy and the flight controller. It is built first, while it is the only
moving part, before any perception exists.

What it does in v1
------------------
No target, no guidance, no detector. It arms, takes off, flies a 20 m square
(north, east, south, west) using world-frame velocity setpoints through MAVROS,
then lands. Every state of the full mission state machine is implemented even
though ``ENGAGE`` has nothing to engage yet.

Three behaviours are load-bearing and implemented now, not later:

1. If the flight controller leaves GUIDED, the pilot has taken over. We stop
   publishing setpoints immediately and drop to ``IDLE``. Checked on every
   ``/mavros/state`` message -- never on a timer. Never fight the pilot.
2. Setpoints are published from a dedicated timer at ``mission.setpoint_rate_hz``
   whenever we are commanding the vehicle. Below the flight controller's minimum
   rate ArduPilot exits autonomous mode silently.
3. There is exactly one place in this file that calls ``publish`` on the
   setpoint topic (:meth:`_setpoint_timer_cb`), and it only fires when the state
   machine has authorised it. The state machine cannot be bypassed.

The ENU/NED trap, second appearance
-----------------------------------
``tools/first_flight.py`` spoke raw MAVLink in NED: x is North. MAVROS speaks
ENU: x is East, y is North. This node flies the same square and the first leg
must still go north. :meth:`_verify_leg` records local position at the start and
end of every leg and asserts the commanded axis dominated in the commanded
direction. It fails loudly on a wrong turn; it cannot pass while turning wrong.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from stryka_common.node_logging import NodeMonitor

# --------------------------------------------------------------------------- #
# Gate 0 square geometry. These mirror tools/first_flight.py exactly -- the
# whole point is that the two implementations fly the same path. Values that
# also appear in params.yaml (setpoint rate, abort range floor) are read from
# there, not duplicated here.
# --------------------------------------------------------------------------- #

TARGET_ALTITUDE_M: float = 10.0
ALTITUDE_TOLERANCE_M: float = 0.5

LEG_LENGTH_M: float = 20.0
LEG_SPEED_MS: float = 5.0

ARRIVAL_TOLERANCE_M: float = 1.0        # within this of the leg end == arrived
SETTLE_SPEED_MS: float = 0.3           # ground speed that counts as "stopped"

# Frame-convention verification thresholds.
PROGRESS_MIN_FRACTION: float = 0.70     # commanded axis must cover >= 70% of the leg
CROSS_TRACK_TOLERANCE_M: float = 5.0    # allowed drift on the uncommanded axis

# Timing.
CONTROL_RATE_HZ: float = 10.0          # state-machine tick rate
HOLD_SETTLE_S: float = 3.0            # quiet hold before starting the square
ABORT_HOLD_S: float = 3.0            # hold still after an abort before landing
SERVICE_RETRY_S: float = 3.0            # re-issue a mode/arm request after this long

# Phase timeouts (seconds).
TAKEOFF_TIMEOUT_S: float = 60.0
LEG_TIMEOUT_S: float = 40.0
LAND_TIMEOUT_S: float = 120.0

# SET_POSITION_TARGET type mask: command velocity only. Bit set == field ignored.
VELOCITY_TYPE_MASK: int = (
    PositionTarget.IGNORE_PX
    | PositionTarget.IGNORE_PY
    | PositionTarget.IGNORE_PZ
    | PositionTarget.IGNORE_AFX
    | PositionTarget.IGNORE_AFY
    | PositionTarget.IGNORE_AFZ
    | PositionTarget.IGNORE_YAW
    | PositionTarget.IGNORE_YAW_RATE
)

GUIDED_MODE: str = "GUIDED"
LAND_MODE: str = "LAND"


class MissionState(Enum):
    """The full mission state machine. All states exist in v1."""

    IDLE = auto()      # on the ground, or the pilot has control; not commanding
    ARMED = auto()     # GUIDED confirmed, arming requested/confirmed
    TAKEOFF = auto()   # climbing to the target altitude
    HOLD = auto()      # holding position, commanding zero velocity
    ENGAGE = auto()    # flying the square (later: closing on a target)
    ABORT = auto()     # engagement given up; hold, then land
    LAND = auto()      # descending; waiting for disarm


class LegPhase(Enum):
    CRUISE = auto()    # driving toward the end of the current leg
    SETTLE = auto()    # commanded stop, waiting for the vehicle to stop moving


@dataclass(frozen=True)
class Leg:
    """One side of the square.

    Velocities are ENU (x=East, y=North) because that is what MAVROS wants on
    ``setpoint_raw/local``. ``axis``/``sign`` describe the world-frame motion we
    must observe for the leg to count as correct.
    """

    name: str
    vx_east: float
    vy_north: float
    axis: str          # "north" or "east" -- the axis that must dominate
    sign: int          # +1 or -1 -- expected direction on that axis


def build_legs() -> tuple[Leg, ...]:
    """North, then east, then south, then west."""
    v = LEG_SPEED_MS
    return (
        Leg("north", 0.0, +v, "north", +1),
        Leg("east", +v, 0.0, "east", +1),
        Leg("south", 0.0, -v, "north", -1),
        Leg("west", -v, 0.0, "east", -1),
    )


@dataclass
class LegResult:
    name: str
    delta_north: float
    delta_east: float


@dataclass
class SquareRun:
    """Mutable state of the square being flown in ENGAGE."""

    legs: tuple[Leg, ...]
    index: int = 0
    phase: LegPhase = LegPhase.CRUISE
    start_east: float = 0.0
    start_north: float = 0.0
    leg_started_at: float = 0.0
    results: list[LegResult] = field(default_factory=list)

    @property
    def current(self) -> Leg:
        return self.legs[self.index]

    @property
    def done(self) -> bool:
        return self.index >= len(self.legs)


class MissionManager(Node):
    """Gate 0 mission manager node."""

    def __init__(self) -> None:
        super().__init__("mission_manager")

        self._load_config()

        # --- state ---------------------------------------------------- #
        self._state: MissionState = MissionState.IDLE
        self._state_entered_at: float = self._now()
        self._mission_complete: bool = False

        self._latest_fcu: Optional[State] = None
        self._pose: Optional[PoseStamped] = None
        self._velocity: Optional[TwistStamped] = None
        self._external_vel_cmd: Optional[TwistStamped] = None  # stub, unused in v1

        # The single authorised setpoint. Written only by the state machine,
        # read only by the setpoint timer.
        self._setpoint_authorised: bool = False
        self._cmd_east: float = 0.0
        self._cmd_north: float = 0.0
        self._cmd_up: float = 0.0

        # Service-request guards so we issue each command once (with retry).
        self._expect_guided: bool = False
        self._guided_requested_at: Optional[float] = None
        self._arm_requested_at: Optional[float] = None
        self._takeoff_requested: bool = False
        self._land_requested: bool = False

        self._square: Optional[SquareRun] = None

        # --- observability (STRYKA logging standard) ------------------ #
        self._monitor = NodeMonitor(self, lambda: self._state.name)

        # --- interfaces ---------------------------------------------- #
        self._setpoint_pub = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", qos_profile_system_default
        )
        self.create_subscription(
            State, "/mavros/state", self._on_fcu_state, qos_profile_system_default
        )
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._on_pose,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistStamped,
            "/mavros/local_position/velocity_local",
            self._on_velocity,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistStamped, "/stryka/velocity_cmd", self._on_external_vel_cmd, 10
        )

        self._set_mode = self.create_client(SetMode, "/mavros/set_mode")
        self._arming = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._takeoff = self.create_client(CommandTOL, "/mavros/cmd/takeoff")

        # --- timers ------------------------------------------------- #
        # Setpoint stream: fixed rate, its own timer, the one publish choke point.
        self._setpoint_timer = self.create_timer(
            1.0 / self._setpoint_rate_hz, self._setpoint_timer_cb
        )
        # State machine: everything that decides *what* to command.
        self._control_timer = self.create_timer(
            1.0 / CONTROL_RATE_HZ, self._control_tick
        )

        self.get_logger().info(
            f"mission_manager up: setpoint_rate={self._setpoint_rate_hz} Hz, "
            f"abort_range_floor={self._abort_range_floor_m} m, "
            f"target_alt={TARGET_ALTITUDE_M} m, leg={LEG_LENGTH_M} m"
        )

    # ------------------------------------------------------------------ #
    # Config
    # ------------------------------------------------------------------ #

    def _load_config(self) -> None:
        """Load params.yaml. Nothing that lives in the config is hard-coded.

        The launch file passes ``config_file`` explicitly. For a bare
        ``ros2 run`` we fall back to the copy installed by ``stryka_bringup``
        (resolved at runtime -- there is deliberately no package dependency on
        bringup, so the two do not form a build cycle).
        """
        config_file = (
            self.declare_parameter("config_file", "").get_parameter_value().string_value
        )
        if not config_file:
            config_file = os.path.join(
                get_package_share_directory("stryka_bringup"), "config", "params.yaml"
            )

        with open(config_file, "r", encoding="utf-8") as handle:
            config = yaml.safe_load(handle)

        mission = config["mission"]
        self._setpoint_rate_hz: float = float(mission["setpoint_rate_hz"])
        self._abort_range_floor_m: float = float(mission["abort_range_floor_m"])
        self.get_logger().info(f"loaded config from {config_file}")

    # ------------------------------------------------------------------ #
    # Time helpers
    # ------------------------------------------------------------------ #

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _time_in_state(self) -> float:
        return self._now() - self._state_entered_at

    # ------------------------------------------------------------------ #
    # Subscriptions
    # ------------------------------------------------------------------ #

    def _on_fcu_state(self, msg: State) -> None:
        """Track FCU state and enforce 'never fight the pilot' on every message."""
        self._latest_fcu = msg

        # Behaviour 1: if we expected to be flying GUIDED and we are not, the
        # pilot has taken over. Stop now -- do not wait for the next tick.
        flying = self._state in (
            MissionState.TAKEOFF,
            MissionState.HOLD,
            MissionState.ENGAGE,
            MissionState.ABORT,
        )
        if self._expect_guided and flying and msg.mode != GUIDED_MODE:
            self._setpoint_authorised = False
            self._transition(
                MissionState.IDLE,
                f"mode left GUIDED (now {msg.mode!r}) -- pilot has control",
            )

    def _on_pose(self, msg: PoseStamped) -> None:
        self._pose = msg

    def _on_velocity(self, msg: TwistStamped) -> None:
        self._velocity = msg

    def _on_external_vel_cmd(self, msg: TwistStamped) -> None:
        # Stub for v1. A later week's ENGAGE will consume this instead of the
        # hard-coded square; keeping the subscription here fixes the contract.
        self._external_vel_cmd = msg

    # ------------------------------------------------------------------ #
    # Local position helpers (ENU: x=East, y=North, z=Up)
    # ------------------------------------------------------------------ #

    def _east(self) -> float:
        assert self._pose is not None
        return self._pose.pose.position.x

    def _north(self) -> float:
        assert self._pose is not None
        return self._pose.pose.position.y

    def _altitude(self) -> float:
        assert self._pose is not None
        return self._pose.pose.position.z

    def _ground_speed(self) -> float:
        if self._velocity is None:
            return math.inf
        v = self._velocity.twist.linear
        return math.hypot(v.x, v.y)

    def _have_pose(self) -> bool:
        return self._pose is not None

    # ------------------------------------------------------------------ #
    # The one setpoint publish choke point
    # ------------------------------------------------------------------ #

    def _setpoint_timer_cb(self) -> None:
        """Publish the authorised velocity setpoint at the configured rate.

        This is the only call to ``self._setpoint_pub.publish`` in the file.
        It emits nothing unless the state machine has authorised a setpoint,
        and it never computes one -- it only ships the cached command.
        """
        if not self._setpoint_authorised:
            return

        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        # MAVROS re-frames ENU -> NED for the FCU; we supply ENU here.
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = VELOCITY_TYPE_MASK
        msg.velocity.x = self._cmd_east
        msg.velocity.y = self._cmd_north
        msg.velocity.z = self._cmd_up
        self._setpoint_pub.publish(msg)

    def _command_velocity(self, east: float, north: float, up: float = 0.0) -> None:
        """Set the cached setpoint. Reachable only from the state machine."""
        self._cmd_east = east
        self._cmd_north = north
        self._cmd_up = up

    # ------------------------------------------------------------------ #
    # State machine
    # ------------------------------------------------------------------ #

    def _transition(self, new_state: MissionState, reason: str) -> None:
        """The single choke point for state changes. Logs trigger + runs entry."""
        if new_state is self._state:
            return
        old = self._state
        self._monitor.record_transition(old.name, new_state.name, reason)
        self._state = new_state
        self._state_entered_at = self._now()

        # Entry actions.
        if new_state is MissionState.IDLE:
            self._setpoint_authorised = False
            self._expect_guided = False
            self._guided_requested_at = None
            self._arm_requested_at = None
            self._takeoff_requested = False
            self._square = None
        elif new_state is MissionState.ARMED:
            self._setpoint_authorised = False
        elif new_state is MissionState.TAKEOFF:
            self._setpoint_authorised = False
        elif new_state in (MissionState.HOLD, MissionState.ABORT):
            self._command_velocity(0.0, 0.0, 0.0)
            self._setpoint_authorised = True
        elif new_state is MissionState.ENGAGE:
            self._command_velocity(0.0, 0.0, 0.0)
            self._setpoint_authorised = True
            self._square = SquareRun(legs=build_legs())
            self._begin_leg()
        elif new_state is MissionState.LAND:
            self._setpoint_authorised = False
            self._land_requested = False

    def _control_tick(self) -> None:
        """Advance the state machine. Never publishes; only decides commands."""
        handler = {
            MissionState.IDLE: self._tick_idle,
            MissionState.ARMED: self._tick_armed,
            MissionState.TAKEOFF: self._tick_takeoff,
            MissionState.HOLD: self._tick_hold,
            MissionState.ENGAGE: self._tick_engage,
            MissionState.ABORT: self._tick_abort,
            MissionState.LAND: self._tick_land,
        }[self._state]
        handler()

    # -- IDLE ---------------------------------------------------------- #

    def _tick_idle(self) -> None:
        if self._mission_complete:
            return
        if self._latest_fcu is None or not self._latest_fcu.connected:
            self._log_throttled("waiting for FCU connection (/mavros/state)")
            return
        if not self._have_pose():
            self._log_throttled("waiting for /mavros/local_position/pose")
            return
        self._transition(MissionState.ARMED, "FCU connected and localised")

    # -- ARMED ------------------------------------------------------- #

    def _tick_armed(self) -> None:
        fcu = self._latest_fcu
        assert fcu is not None

        if fcu.mode != GUIDED_MODE:
            self._request_guided()
            return

        self._expect_guided = True

        if not fcu.armed:
            self._request_arm()
            return

        if not self._takeoff_requested:
            self._request_takeoff()
            return

        self._transition(MissionState.TAKEOFF, "armed in GUIDED, takeoff commanded")

    # -- TAKEOFF --------------------------------------------------- #

    def _tick_takeoff(self) -> None:
        if self._time_in_state() > TAKEOFF_TIMEOUT_S:
            self._transition(MissionState.ABORT, "takeoff timed out")
            return
        if not self._have_pose():
            return
        error = abs(self._altitude() - TARGET_ALTITUDE_M)
        if error <= ALTITUDE_TOLERANCE_M:
            self.get_logger().info(f"altitude reached: {self._altitude():.2f} m")
            self._transition(MissionState.HOLD, "reached target altitude")

    # -- HOLD ----------------------------------------------------- #

    def _tick_hold(self) -> None:
        self._command_velocity(0.0, 0.0, 0.0)
        if self._time_in_state() >= HOLD_SETTLE_S:
            self._transition(MissionState.ENGAGE, "hold stable -- starting square")

    # -- ENGAGE (the square) ------------------------------------- #

    def _begin_leg(self) -> None:
        square = self._square
        assert square is not None
        square.phase = LegPhase.CRUISE
        square.start_east = self._east()
        square.start_north = self._north()
        square.leg_started_at = self._now()
        leg = square.current
        self.get_logger().info(
            f"leg {leg.name} START at ENU east={square.start_east:.2f} "
            f"north={square.start_north:.2f} alt={self._altitude():.2f}"
        )

    def _tick_engage(self) -> None:
        square = self._square
        assert square is not None

        if not self._have_pose() or self._velocity is None:
            self._command_velocity(0.0, 0.0, 0.0)
            return

        if self._now() - square.leg_started_at > LEG_TIMEOUT_S:
            self._transition(
                MissionState.ABORT, f"leg {square.current.name} timed out"
            )
            return

        leg = square.current
        travelled = math.hypot(
            self._east() - square.start_east, self._north() - square.start_north
        )

        if square.phase is LegPhase.CRUISE:
            self._command_velocity(leg.vx_east, leg.vy_north, 0.0)
            if travelled >= LEG_LENGTH_M - ARRIVAL_TOLERANCE_M:
                square.phase = LegPhase.SETTLE
            return

        # SETTLE: commanded stop, wait for the vehicle to actually stop.
        self._command_velocity(0.0, 0.0, 0.0)
        if self._ground_speed() > SETTLE_SPEED_MS:
            return

        result = self._verify_leg(leg, square)
        if result is None:
            self._transition(
                MissionState.ABORT,
                f"leg {leg.name}: frame-convention verification FAILED",
            )
            return

        square.results.append(result)
        square.index += 1
        if square.done:
            self._print_summary(square)
            self._transition(MissionState.LAND, "square complete")
        else:
            self._begin_leg()

    def _verify_leg(self, leg: Leg, square: SquareRun) -> Optional[LegResult]:
        """Assert the vehicle moved the way it was told. Loud failure, no silent pass.

        Returns the leg result on success, ``None`` on a frame-convention fault.
        """
        delta_east = self._east() - square.start_east
        delta_north = self._north() - square.start_north

        primary = delta_north if leg.axis == "north" else delta_east
        cross = delta_east if leg.axis == "north" else delta_north
        cross_axis = "east" if leg.axis == "north" else "north"

        self.get_logger().info(
            f"leg {leg.name} END: measured ENU delta = "
            f"dNorth {delta_north:+.2f} m, dEast {delta_east:+.2f} m"
        )

        needed = PROGRESS_MIN_FRACTION * LEG_LENGTH_M
        directed_progress = primary * leg.sign
        wrong_direction = directed_progress < needed
        excess_cross_track = abs(cross) > CROSS_TRACK_TOLERANCE_M

        if wrong_direction or excess_cross_track:
            want = f"{'+' if leg.sign > 0 else '-'}{leg.axis}"
            log = self.get_logger()
            log.error("=" * 68)
            log.error("FRAME-CONVENTION FAULT SUSPECTED -- ENU/NED AXIS CONFUSION")
            log.error("=" * 68)
            log.error(f"leg            : {leg.name}")
            log.error(
                f"commanded      : ENU velocity x(E)={leg.vx_east:+.1f} "
                f"y(N)={leg.vy_north:+.1f} m/s  (expect motion {want})"
            )
            log.error(
                f"measured delta : dNorth {delta_north:+.2f} m, dEast {delta_east:+.2f} m"
            )
            if wrong_direction:
                log.error(
                    f"--> commanded {leg.axis} axis moved {directed_progress:+.2f} m, "
                    f"need >= {needed:.2f} m in the commanded direction"
                )
            if excess_cross_track:
                log.error(
                    f"--> uncommanded {cross_axis} axis moved {cross:+.2f} m, "
                    f"tolerance is +/-{CROSS_TRACK_TOLERANCE_M:.2f} m"
                )
            log.error(
                "likely cause: MAVROS is ENU (x=East, y=North); first_flight.py was "
                "NED (x=North). A crossed axis turns 'north' into 'east', or a sign "
                "flip reverses a leg. Fix the frame mapping before trusting guidance."
            )
            log.error("=" * 68)
            return None

        self.get_logger().info(
            f"leg {leg.name}: frame check PASSED "
            f"(commanded axis {directed_progress:+.2f} m, cross-track {cross:+.2f} m)"
        )
        return LegResult(leg.name, delta_north, delta_east)

    # -- ABORT --------------------------------------------------- #

    def _tick_abort(self) -> None:
        self._command_velocity(0.0, 0.0, 0.0)
        if self._time_in_state() >= ABORT_HOLD_S:
            self._transition(MissionState.LAND, "abort hold complete -- landing")

    # -- LAND ---------------------------------------------------- #

    def _tick_land(self) -> None:
        fcu = self._latest_fcu
        if not self._land_requested:
            self._request_land()
            return
        if fcu is not None and not fcu.armed:
            self.get_logger().info("disarmed after landing")
            self._mission_complete = True
            self._transition(MissionState.IDLE, "landed and disarmed -- mission complete")
            return
        if self._time_in_state() > LAND_TIMEOUT_S:
            self.get_logger().error("LAND timed out waiting for disarm")
            self._transition(MissionState.IDLE, "land timed out")

    # ------------------------------------------------------------------ #
    # Service requests (fire-and-forget; progress is judged from /mavros/state)
    # ------------------------------------------------------------------ #

    def _request_guided(self) -> None:
        if not self._should_reissue(self._guided_requested_at):
            return
        if not self._set_mode.service_is_ready():
            self._log_throttled("waiting for /mavros/set_mode")
            return
        req = SetMode.Request()
        req.custom_mode = GUIDED_MODE
        self._set_mode.call_async(req).add_done_callback(
            lambda f: self._log_service_result("set_mode GUIDED", f, "mode_sent")
        )
        self._guided_requested_at = self._now()
        self.get_logger().info("requested mode GUIDED")

    def _request_arm(self) -> None:
        if not self._should_reissue(self._arm_requested_at):
            return
        if not self._arming.service_is_ready():
            self._log_throttled("waiting for /mavros/cmd/arming")
            return
        req = CommandBool.Request()
        req.value = True
        self._arming.call_async(req).add_done_callback(
            lambda f: self._log_service_result("arming", f, "success")
        )
        self._arm_requested_at = self._now()
        self.get_logger().info("requested arm")

    def _request_takeoff(self) -> None:
        if not self._takeoff.service_is_ready():
            self._log_throttled("waiting for /mavros/cmd/takeoff")
            return
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = float(TARGET_ALTITUDE_M)
        self._takeoff.call_async(req).add_done_callback(
            lambda f: self._log_service_result("takeoff", f, "success")
        )
        self._takeoff_requested = True
        self.get_logger().info(f"requested takeoff to {TARGET_ALTITUDE_M:.1f} m")

    def _request_land(self) -> None:
        if not self._set_mode.service_is_ready():
            self._log_throttled("waiting for /mavros/set_mode (LAND)")
            return
        req = SetMode.Request()
        req.custom_mode = LAND_MODE
        self._set_mode.call_async(req).add_done_callback(
            lambda f: self._log_service_result("set_mode LAND", f, "mode_sent")
        )
        self._expect_guided = False  # we are deliberately leaving GUIDED now
        self._land_requested = True
        self.get_logger().info("requested mode LAND")

    def _should_reissue(self, last_at: Optional[float]) -> bool:
        return last_at is None or (self._now() - last_at) > SERVICE_RETRY_S

    def _log_service_result(self, label: str, future, ok_attr: str) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 -- log whatever the client raised
            self.get_logger().warning(f"{label} call failed: {exc}")
            return
        ok = getattr(response, ok_attr, None)
        self.get_logger().info(f"{label} response: {ok_attr}={ok}")

    def _log_throttled(self, message: str) -> None:
        self.get_logger().info(message, throttle_duration_sec=2.0)

    # ------------------------------------------------------------------ #
    # Summary
    # ------------------------------------------------------------------ #

    def _print_summary(self, square: SquareRun) -> None:
        log = self.get_logger()
        log.info("-" * 68)
        log.info("STRYKA mission_manager -- square summary")
        log.info("-" * 68)
        for result in square.results:
            log.info(
                f"  {result.name:<6} dNorth {result.delta_north:+7.2f}   "
                f"dEast {result.delta_east:+7.2f}"
            )
        passed = len(square.results) == len(square.legs)
        log.info("-" * 68)
        log.info(f"  RESULT: {'PASS' if passed else 'FAIL'}")
        log.info("-" * 68)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
