# STRYKA — State Diagnostic

**Written:** 2026-09-10 (filename dated 2026-09-08 as requested — the state
described is the working tree as of the last modification, 2026-09-08 17:41).
**Workspace:** `~/Documents/STRYKA`
**Git HEAD:** `d7162031287b82dcc0fabbaf7138c2e516a7d32b` on `main`
**Method:** read-only. Every claim below is from reading a file, running a
search, or a throwaway build in `/tmp`. Nothing in the repository was modified
except the creation of this file.

---

## 0. Two corrections to the framing of this diagnostic

Before anything else, two premises in the brief do not match the disk.

### 0.1 Checkpoint 7 HAS been run, and it FAILED

The brief states Checkpoint 7 "has **not** been run." It has.
`docs/gates/checkpoint7.md` is a 281-line gate record dated 2026-09-08 ~09:40Z
against this exact commit, and its first line reads:

> **Result: FAIL** — the square flies correctly (north first, frame convention
> clean), but the stand-down test fails: on a mode change out of GUIDED the node
> stands down for ~0.5 s and then **forces the vehicle back into GUIDED on its
> own and resumes the mission**. It fights the pilot.

The raw artefacts it cites are present in `scratch/checkpoint7/` (29 files:
node logs, independently-recorded pose/setpoint/state CSVs, MAVProxy tlogs,
analysis scripts). This is not a stale or aspirational document.

**This matters more than anything else in this report.** I read
`mission_manager.py` cold and independently derived the same defect (§3.2). The
code analysis and the flight evidence agree. Checkpoint 7's blocking failure is
real, reproduced, and understood.

### 0.2 The entire codebase is untracked

`git ls-files` returns exactly two files: `.gitignore` and `README.md`. All of
`src/`, `tools/` and `docs/` — including the 739-line `mission_manager.py`, the
585-line `first_flight.py` and the Checkpoint 7 gate record — is **untracked
working-tree state**. Nothing has ever been pushed. See §5.

Given that "a previous session was lost," this is the single largest
unmitigated risk in the workspace right now.

---

## 1. What is present and working

### 1.1 Packages under `src/`

| Package | Build type | Status | Evidence |
|---|---|---|---|
| `stryka_msgs` | `ament_cmake` | **Real** | `msg/TargetState.msg` (18 lines), `CMakeLists.txt` invokes `rosidl_generate_interfaces`; builds and renders (§2.4) |
| `stryka_common` | `ament_python` | **Real** | `node_logging.py`, 87 lines: `NodeMonitor` (heartbeat + periodic state log + transition log) |
| `stryka_mission` | `ament_python` | **Real** | `mission_manager.py`, 739 lines; entry point `mission_manager` declared in `setup.py` |
| `stryka_bringup` | `ament_python` | **Real** (config/launch only) | `config/params.yaml` (32 lines), `launch/sim.launch.py` (39 lines); no console scripts, correctly |
| `stryka_guidance` | `ament_python` | **Scaffolding only** | `__init__.py` is 0 bytes; `entry_points.console_scripts` empty; `package.xml` description `TODO: Package description` |
| `stryka_perception` | `ament_python` | **Scaffolding only** | identical — 0-byte `__init__.py`, no entry points, TODO description |
| `stryka_tracking` | `ament_python` | **Scaffolding only** | identical |

The three scaffolding packages are unmodified `ros2 pkg create` output. They
contain no nodes and no logic of any kind. Confirmed by reading, not inferred
from size: `find src -name '__init__.py' | xargs wc -l` reports 0 for all of
them.

### 1.2 `tools/`

- `first_flight.py` — 585 lines, real and complete. Standalone `pymavlink` Gate
  0 command-path proof. Speaks **NED** (`send_velocity(vnorth, veast, vdown)`,
  `MAV_FRAME_LOCAL_NED`), by design and clearly documented. Contains its own
  frame verification with the same 70 % / 5 m thresholds as the ROS node.
- `tools/__pycache__/` — build artefact, correctly gitignored.

### 1.3 `docs/`

- `docs/gates/checkpoint7.md` — the gate record described in §0.1.
- `docs/.gitkeep`
- `docs/state/` — **did not exist**; created to hold this file.

### 1.4 Working end-to-end, with evidence

- The Gate 0 square flies correctly through MAVROS. Four legs, first leg north,
  cross-track < 0.4 m on every leg, square closes 0.30 m E / 0.32 m S of origin
  (`checkpoint7.md` §2, independently re-derived from `/mavros/local_position/pose`).
- Setpoint rate holds: 720 setpoints over 35.9 s = **20.00 Hz mean**, rolling
  1-s median 20 Hz, against a configured 20.0 Hz (`checkpoint7.md` §3).
- `type_mask` on every setpoint measured as `3527` — velocity-only. I verified
  this constant independently: `IGNORE_PX|PY|PZ|AFX|AFY|AFZ|YAW|YAW_RATE` = 3527.
- The ENU/NED verification works and passed on real flight data.

---

## 2. Build health

### 2.1 Is `build/` / `install/` / `log/` current with respect to sources?

**For four of seven packages, yes. For three, they have never been built at all.**

The last build was 2026-09-08 17:31:41. From `log/build_2026-09-08_17-31-41/logger_all.log`:

```
Command line arguments: ['/usr/bin/colcon', 'build', '--packages-select',
  'stryka_msgs', 'stryka_common', 'stryka_bringup', 'stryka_mission',
  '--symlink-install']
```

and from `events.log`:

```
[0.000561] (-) JobUnselected: {'identifier': 'stryka_guidance'}
[0.000655] (-) JobUnselected: {'identifier': 'stryka_perception'}
[0.000741] (-) JobUnselected: {'identifier': 'stryka_tracking'}
```

That build exited 0 with zero bytes of stderr for all four packages. Newest
source file is `mission_manager.py` at 17:01, thirty minutes before the build,
so the four built packages are current. `--symlink-install` means subsequent
Python edits take effect without rebuilding.

`build/` and `install/` therefore contain four packages, not seven. A plain
`colcon build` has never been run in this workspace.

### 2.2 A clean `colcon build` FAILS in the default shell

This is a real finding, not an artefact of my environment. I built into a
throwaway directory (`--build-base` / `--install-base` / `--log-base` all under
`/tmp`); the existing `build/`, `install/` and `log/` were untouched, verified
by mtime before and after.

**Inside the workspace `.venv` (which is auto-activated in the VS Code / Claude
Code terminal — `VIRTUAL_ENV` is set and `.venv/bin` is first on `PATH`):**

```
--- stderr: stryka_msgs
ModuleNotFoundError: No module named 'em'
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:130
  CMakeLists.txt:13 (rosidl_generate_interfaces)
Failed   <<< stryka_msgs [1.37s, exited with code 1]
Summary: 0 packages finished
  1 package failed: stryka_msgs
  4 packages aborted: stryka_common stryka_guidance stryka_perception stryka_tracking
```

**Outside the venv, same command, same sources:**

```
Summary: 7 packages finished [4.02s]
```

Root cause, confirmed:

```
$ cat .venv/pyvenv.cfg
include-system-site-packages = false
$ /usr/bin/python3 -c "import em"        # OK, empy 3.3.4 (python3-empy, apt)
$ .venv/bin/python -c "import em"        # ModuleNotFoundError
```

`rosidl` needs `empy` at build time. The system Python has it; the venv is
sealed off from system site-packages and does not. So whether `colcon build`
works depends entirely on whether the venv happens to be active — and in the
editor terminal, it is. The 09-08 build succeeded because it was run from a
shell without the venv.

This is a latent trap. It will present as "the message package suddenly stopped
building" to whoever hits it next, most likely Mel.

### 2.3 `COLCON_IGNORE`

All four required markers are present:

```
PRESENT : .venv/COLCON_IGNORE
PRESENT : ardupilot/COLCON_IGNORE
PRESENT : scratch/COLCON_IGNORE
PRESENT : detached/COLCON_IGNORE
```

Also present, correctly, in `build/`, `install/` and `log/`.

**Anything else colcon would walk that it shouldn't:** `terrain/` and `.vscode/`
have no `COLCON_IGNORE`. Neither contains a `package.xml`, `setup.py` or
`CMakeLists.txt`, so colcon finds nothing in them — I verified with
`find . -maxdepth 4 -name package.xml -not -path './src/*'`, which returns
nothing. Cosmetic only; a wasted crawl, not a build hazard.

### 2.4 `ros2 interface show stryka_msgs/msg/TargetState`

Renders correctly. Full output verified: comments preserved,
`std_msgs/Header` expanded to `builtin_interfaces/Time`, `geometry_msgs/Point`
and `Vector3` expanded to their `float64 x/y/z`, `float64[9]
position_covariance` intact, and:

```
float64  bearing_error         # angle off boresight, RADIANS -- never pixels
```

`mavros_msgs` is installed and importable (126 interfaces).

---

## 3. `mission_manager` against its requirements

File: `src/stryka_mission/stryka_mission/mission_manager.py` (739 lines).

### 3.1 State machine — **MET**

All seven states exist as enum members (lines 101–110):

```python
class MissionState(Enum):
    IDLE = auto()      # on the ground, or the pilot has control; not commanding
    ARMED = auto()     # GUIDED confirmed, arming requested/confirmed
    TAKEOFF = auto()   # climbing to the target altitude
    HOLD = auto()      # holding position, commanding zero velocity
    ENGAGE = auto()    # flying the square (later: closing on a target)
    ABORT = auto()     # engagement given up; hold, then land
    LAND = auto()      # descending; waiting for disarm
```

Every transition goes through one function, which logs the trigger (lines 385–392):

```python
def _transition(self, new_state: MissionState, reason: str) -> None:
    """The single choke point for state changes. Logs trigger + runs entry."""
    if new_state is self._state:
        return
    old = self._state
    self._monitor.record_transition(old.name, new_state.name, reason)
```

and `NodeMonitor.record_transition` (node_logging.py:85–87):

```python
def record_transition(self, old: str, new: str, reason: str) -> None:
    """Log a state transition with the trigger that caused it."""
    self._node.get_logger().info(f"STATE {old} -> {new}  ({reason})")
```

I found no assignment to `self._state` outside `_transition`. Every state is
reachable: `ABORT` from takeoff timeout (line 470), leg timeout (line 510) and
frame-check failure (line 533). The flight log in `checkpoint7.md` §2 shows the
real transition trace.

### 3.2 Never fight the pilot — **NOT MET. This is the blocking defect.**

**The detection half is correct.** It is on the state callback, not a timer,
exactly as required (lines 295–312):

```python
def _on_fcu_state(self, msg: State) -> None:
    """Track FCU state and enforce 'never fight the pilot' on every message."""
    self._latest_fcu = msg
    flying = self._state in (
        MissionState.TAKEOFF, MissionState.HOLD,
        MissionState.ENGAGE, MissionState.ABORT,
    )
    if self._expect_guided and flying and msg.mode != GUIDED_MODE:
        self._setpoint_authorised = False
        self._transition(
            MissionState.IDLE,
            f"mode left GUIDED (now {msg.mode!r}) -- pilot has control",
        )
```

**The stand-down is not latched, and is undone within one control tick.**
Entering `IDLE` clears every guard that would prevent restarting (lines 395–401):

```python
if new_state is MissionState.IDLE:
    self._setpoint_authorised = False
    self._expect_guided = False
    self._guided_requested_at = None
    self._arm_requested_at = None
    self._takeoff_requested = False
    self._square = None
```

and `_tick_idle`, running at 10 Hz, restarts the mission unconditionally
(lines 433–442):

```python
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
```

The only gate is `self._mission_complete`, and a pilot takeover does not set it
— it is set in exactly one place, `_tick_land` after a successful disarm (line
623). So 100 ms after standing down, the node enters `ARMED`, and `_tick_armed`
sees a non-GUIDED mode and **commands the aircraft back into GUIDED** (lines
450–451):

```python
if fcu.mode != GUIDED_MODE:
    self._request_guided()
    return
```

`checkpoint7.md` §4 measured exactly this, on a real SITL flight:

| T (s) | Event |
|---:|---|
| 0.000 | **LOITER** appears on `/mavros/state` |
| +0.001 | `STATE ENGAGE -> IDLE (mode left GUIDED (now 'LOITER') -- pilot has control)` |
| +0.036 | `STATE IDLE -> ARMED (FCU connected and localised)` |
| +0.137 | `requested mode GUIDED` + `set_mode GUIDED response: mode_sent=True` |
| **+0.145** | **GUIDED re-appears on `/mavros/state` — the node has overridden the pilot** |
| +0.487 | node resumes publishing setpoints |
| +3.54 | node re-enters ENGAGE and re-flies the square |

195 setpoints were published after the override; drift reached 36 m before the
harness killed the node. The node held `IDLE` for 36 milliseconds.

**Secondary gap in the same requirement.** `flying` excludes `MissionState.ARMED`.
Between arming and takeoff the vehicle is armed on the ground with props
spinning, and a pilot mode change in that window is not caught by
`_on_fcu_state` at all — instead `_tick_armed` will actively re-request GUIDED.
The stand-down guard does not cover the armed-on-ground window.

### 3.3 Setpoint rate from a dedicated timer — **MET, with a caveat**

Dedicated timer, rate from YAML (lines 239–245):

```python
self._setpoint_timer = self.create_timer(
    1.0 / self._setpoint_rate_hz, self._setpoint_timer_cb
)
self._control_timer = self.create_timer(
    1.0 / CONTROL_RATE_HZ, self._control_tick
)
```

`_setpoint_rate_hz` comes from `params.yaml` (`mission.setpoint_rate_hz: 20.0`).
Measured at 20.00 Hz mean over the Checkpoint 7 flight.

**Caveat:** the node runs on the default `SingleThreadedExecutor` —
`rclpy.spin(node)` at line 730 is the only executor construct in the codebase;
there are no callback groups anywhere. The setpoint timer therefore shares one
thread with the control tick, three subscriptions, the heartbeat timer and the
state-log timer, with no scheduling isolation. This was harmless on a desktop
with no perception running. It is unproven on a Pi 5 with the detector and
tracker in the graph, which `checkpoint7.md` §6 itself lists as untested.

### 3.4 One choke point — **MET**

Verified by search across the whole package, not by reading one function:

```
$ grep -rn "\.publish(" src/
src/stryka_common/stryka_common/node_logging.py:78:  self._heartbeat_pub.publish(msg)
src/stryka_mission/stryka_mission/mission_manager.py:373: self._setpoint_pub.publish(msg)

$ grep -rn "create_publisher" src/
src/stryka_common/stryka_common/node_logging.py:58: node.create_publisher(String, HEARTBEAT_TOPIC, 10)
src/stryka_mission/stryka_mission/mission_manager.py:211: self.create_publisher(PositionTarget, "/mavros/setpoint_raw/local", ...)
```

Exactly one publish on the setpoint topic. The other is `std_msgs/String` on
`~/heartbeat` — not a setpoint. The choke point is authorisation-gated
(lines 354–373):

```python
def _setpoint_timer_cb(self) -> None:
    if not self._setpoint_authorised:
        return
    msg = PositionTarget()
    ...
    msg.type_mask = VELOCITY_TYPE_MASK
    msg.velocity.x = self._cmd_east
    msg.velocity.y = self._cmd_north
    msg.velocity.z = self._cmd_up
    self._setpoint_pub.publish(msg)
```

It computes nothing; it ships a cached command. `_setpoint_authorised` is
written only inside `_transition` — I checked every assignment. The state
machine cannot be bypassed. This requirement is implemented as advertised.

### 3.5 ENU/NED verification — **MET**

Position is recorded at leg start (lines 488–494):

```python
def _begin_leg(self) -> None:
    square.phase = LegPhase.CRUISE
    square.start_east = self._east()
    square.start_north = self._north()
    square.leg_started_at = self._now()
```

with the ENU accessors correct for MAVROS (lines 329–335) — `x` is East, `y` is
North:

```python
def _east(self) -> float:
    return self._pose.pose.position.x
def _north(self) -> float:
    return self._pose.pose.position.y
```

and checked at leg end (lines 552–599), failing loudly:

```python
directed_progress = primary * leg.sign
wrong_direction = directed_progress < needed
excess_cross_track = abs(cross) > CROSS_TRACK_TOLERANCE_M

if wrong_direction or excess_cross_track:
    log.error("FRAME-CONVENTION FAULT SUSPECTED -- ENU/NED AXIS CONFUSION")
    ...
    return None
```

**It cannot reach a PASS while turning the wrong way.** A `None` return drives
`_tick_engage` to `ABORT` (lines 531–537), and `square.index` is incremented
only after a non-`None` result (lines 539–540), so `square.done` — the sole
precondition for `_print_summary` — is unreachable if any leg fails.

Two weaknesses worth recording, neither of which breaks the requirement:

1. The PASS/FAIL line in `_print_summary` is a tautology (line 720):
   `passed = len(square.results) == len(square.legs)`. Since the summary is only
   called when all four legs verified, `passed` is always `True` and the `FAIL`
   branch is dead code. The real gate is the abort path, which works — but the
   printed "RESULT: PASS" carries no independent information.
2. `_verify_leg` checks the two horizontal axes only. A leg that climbed or
   descended significantly would still pass. Altitude is never cross-checked.

### 3.6 Config from YAML — **PARTIALLY MET**

Only two values are loaded (lines 276–278):

```python
mission = config["mission"]
self._setpoint_rate_hz: float = float(mission["setpoint_rate_hz"])
self._abort_range_floor_m: float = float(mission["abort_range_floor_m"])
```

Everything that shapes the flight is a module-level constant (lines 61–83):
`TARGET_ALTITUDE_M = 10.0`, `ALTITUDE_TOLERANCE_M = 0.5`, `LEG_LENGTH_M = 20.0`,
`LEG_SPEED_MS = 5.0`, `ARRIVAL_TOLERANCE_M = 1.0`, `SETTLE_SPEED_MS = 0.3`,
`PROGRESS_MIN_FRACTION = 0.70`, `CROSS_TRACK_TOLERANCE_M = 5.0`,
`CONTROL_RATE_HZ = 10.0`, `HOLD_SETTLE_S`, `ABORT_HOLD_S`, `SERVICE_RETRY_S`,
`TAKEOFF_TIMEOUT_S`, `LEG_TIMEOUT_S`, `LAND_TIMEOUT_S`.

Target altitude, leg speed, and the acceptance tolerances are exactly the values
you will want to change between SITL and a first hardware flight without editing
a source file. The module docstring's claim is narrower than the requirement:

> Values that also appear in params.yaml (setpoint rate, abort range floor) are
> read from there, not duplicated here.

That is true, but only two values appear in `params.yaml` in the first place.

**Also: `abort_range_floor_m` is loaded but never used.** It appears three
times in the workspace — the YAML, the assignment above, and a startup log line
(line 249). It drives no logic. It is currently decorative config.

### 3.7 Design rule check — **both rules hold**

**World-frame velocity only.** A case-insensitive search for
`attitude|thrust|actuator|rc_override|motor|AttitudeTarget` across `src/` and
`tools/` returns two hits, both benign:

```
tools/first_flight.py:74:   mavutil.mavlink.EKF_ATTITUDE      (an EKF health flag)
tools/first_flight.py:351:  "World-frame velocity only — never attitude, thrust or motor commands."
```

Nothing publishes `AttitudeTarget`, `ActuatorControl`, `OverrideRCIn` or
anything motor-level. The one setpoint publisher emits `PositionTarget` with
`type_mask = 3527`, which I verified independently ignores position,
acceleration, yaw and yaw-rate, leaving velocity as the only commanded field.

**Guidance never sees pixels.** No pixel-valued quantity crosses into tracking
or guidance — and the strong reason is that no tracking or guidance code exists
yet (§1.1). Every `_px` hit in the tree is either a `params.yaml` camera entry
(`focal_px`, `model_input_px`, `sensor_width_px`, `detection_floor_px`), a
comment asserting the rule, or `PositionTarget.IGNORE_PX` — a MAVROS bitmask
constant meaning "ignore position X", unrelated to pixels. `TargetState.msg`
declares `bearing_error` in radians and documents the boundary. The contract is
correctly specified; it is not yet exercised by any code.

---

## 4. The known open item — auto-arm on launch

**Confirmed: this is still the behaviour.** Launching the node against a
connected FCU arms the aircraft and takes off, with no human action.

The trigger path, in full:

1. `_control_tick` at 10 Hz dispatches to `_tick_idle` (lines 418–429).
2. `_tick_idle` (lines 433–442) requires only three things — mission not already
   complete, `/mavros/state` reporting `connected`, and a pose received:
   ```python
   self._transition(MissionState.ARMED, "FCU connected and localised")
   ```
   There is no human-input condition anywhere in that function.
3. `_tick_armed` (lines 446–464) then runs the whole sequence unprompted:
   ```python
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
   ```
4. `_request_arm` sends `CommandBool(value=True)`; `_request_takeoff` sends
   `CommandTOL(altitude=10.0)`.

The Checkpoint 7 log shows it taking 0.9 s from node start to takeoff command:
`IDLE -> ARMED` at 203.47, `ARMED -> TAKEOFF` at 204.37.

### Proposed smallest change (NOT implemented)

The important insight is that **this and the §3.2 pilot-fight defect are the
same missing mechanism.** Both exist because `_tick_idle` restarts the mission
whenever the FCU is merely connected. One latch fixes both, and fixing auto-arm
without fixing the latch would leave the pilot-fight bug alive in a new form.

Add a single authorisation flag, consulted by `_tick_idle` and cleared
permanently by a pilot takeover:

- **New field:** `self._launch_authorised: bool`, initialised from config, and
  critically **not reset** by the `IDLE` entry action (unlike every other guard
  there).
- **New config** in `params.yaml` under `mission:`:
  ```yaml
  auto_arm_on_connect: true    # SITL convenience. MUST be false on hardware.
  ```
  Initialise `self._launch_authorised = bool(mission["auto_arm_on_connect"])`.
  The simulator workflow is then unchanged: `ros2 launch stryka_bringup
  sim.launch.py` still flies the square with no extra step.
- **New trigger:** a `std_srvs/Trigger` service `~/authorise_launch` whose
  callback sets `self._launch_authorised = True`. On hardware, with
  `auto_arm_on_connect: false`, nothing happens until someone calls it.
- **One new guard** at the top of `_tick_idle`, after the `_mission_complete`
  check:
  ```python
  if not self._launch_authorised:
      self._log_throttled("waiting for launch authorisation (~/authorise_launch)")
      return
  ```
- **The latch, in `_on_fcu_state`:** set `self._launch_authorised = False`
  alongside `self._setpoint_authorised = False` when a pilot takeover is
  detected. Because the `IDLE` entry action does not touch this field, the node
  stands down and *stays* down until a human re-authorises. That is precisely
  what Checkpoint 7 §4 demands.

Cost: roughly fifteen lines plus one YAML key. It converts the failed
stand-down criterion into a passing one and removes the hardware auto-arm
hazard in the same edit.

Two things to decide before implementing, which are yours not mine:

- Should `auto_arm_on_connect: true` be permitted at all in a file that also
  ships to the aircraft? A safer variant is to key it off the launch file
  (`sim.launch.py` passes the parameter, hardware bringup does not) so the
  aircraft cannot inherit a simulator default by accident.
- Should the `ARMED` window (§3.2, armed on the ground pre-takeoff) be added to
  `flying` in the same change? I would say yes — otherwise a mode change in that
  window still gets fought.

---

## 5. Git state

| Item | Value |
|---|---|
| Branch | `main` |
| HEAD | `d7162031287b82dcc0fabbaf7138c2e516a7d32b` |
| Tracks | `origin/main` → `https://github.com/Angel-O25/STRYKA-Interceptor.git` |
| Ahead / behind | `0 0` — exactly in sync |
| Tracked files | **2** — `.gitignore`, `README.md` |

### What is uncommitted

Everything. `git add -An` would stage 56 files: all seven packages, both real
source files, `first_flight.py`, `params.yaml`, the launch file, the Checkpoint
7 gate record, and the `.gitignore` modification.

**Should any of it be committed? All of it, immediately.** The `.gitignore`
diff is a strict improvement (adds `scratch/`, and the SITL run artefacts
`eeprom.bin`, `mav.parm`, `mav.tlog*`, `terrain/`, `logs/`,
`install_geographiclib_datasets.sh` that ArduPilot dumps into the repo root).
There is no reason to hold any of it back. The Checkpoint 7 record itself flags
this — it lists the working tree as DIRTY and notes the code under test was
never committed, which means **the artefacts in `scratch/checkpoint7/` currently
attest to a code state that exists in exactly one place on one disk.**

### Nothing large or generated is tracked

Trivially true, since only two small text files are tracked. More usefully, I
verified `git check-ignore` succeeds for every candidate:

```
build/  install/  log/  ardupilot/  .venv/  scratch/  detached/
eeprom.bin  mav.tlog  mav.parm  terrain/  tools/__pycache__/*.pyc   → all IGNORED
```

`git add -An` confirms it: the 56 files it would stage are all text sources,
manifests and markdown. No images, weights (`*.pt`, `*.onnx`, `*.hef`,
`*.engine` are all ignored), bags, or binaries.

### `.gitignore` issues

- **Nothing present that should not be.** Every rule is justified.
- **One gap:** only `.vscode/settings.json` is ignored, not `.vscode/`.
  `git check-ignore .vscode/launch.json` → **NOT IGNORED**. `settings.json`
  currently exists and contains a machine-specific absolute path
  (`"cmake.sourceDirectory": "/home/orion/Documents/STRYKA/src/stryka_msgs"`),
  so the current rule is doing useful work — but any other VS Code file added
  later will be picked up. Recommend broadening to `.vscode/` with an explicit
  `!.vscode/extensions.json` if you ever want to share one.
- **Minor:** `*.tlog` and `mav.tlog*` and `*.tlog.raw` overlap. Harmless.

---

## 6. Requirements from Task 3 that are NOT met

*The most important section. Ordered by severity.*

### R2 — "Never fight the pilot" — **NOT MET. Blocking.**

Detection is correct and on the right callback. The **latch is absent**. Entering
`IDLE` clears every guard, and `_tick_idle` unconditionally restarts the mission
on the next 10 Hz tick, re-commanding GUIDED 145 ms after the pilot took it away.
Measured on a real SITL flight (`checkpoint7.md` §4): 195 setpoints published
after the override, 36 m of drift, sustained stand-down **never achieved**.
Full evidence in §3.2. This is the defect that failed Checkpoint 7.

*Recommendation:* implement the `_launch_authorised` latch in §4. Do not fix
auto-arm separately — it is the same mechanism.

### R6 — "Config from YAML, nothing hard-coded that should be tunable" — **PARTIALLY MET.**

Two values come from YAML. Target altitude (10 m), leg speed (5 m/s), leg length
(20 m), all acceptance tolerances and all phase timeouts are module constants.
`abort_range_floor_m` is loaded, logged at startup, and used by nothing.
Evidence in §3.6.

*Recommendation:* promote the flight-shaping values — altitude, speed, tolerances,
timeouts — into `params.yaml` before the first hardware flight, since those are
exactly the values that must differ between SITL and a real aircraft. Either wire
`abort_range_floor_m` into the abort logic or delete it; config that does nothing
is worse than absent config, because it reads as implemented.

### R3 — "Setpoint rate maintained" — **MET, but unproven where it matters.**

20.00 Hz measured, dedicated timer, rate from YAML. But the node uses the default
single-threaded executor with no callback groups (§3.3), and the measurement was
taken on a desktop with no perception stack. `checkpoint7.md` §6 lists Pi 5
behaviour under load as untested.

*Recommendation:* not a code change yet. Re-measure on the Pi 5 with the full
graph running before trusting it; if it degrades, the fix is a
`MutuallyExclusiveCallbackGroup` for the setpoint timer on a `MultiThreadedExecutor`.

### R5 — "ENU/NED verification" — **MET.** Two cosmetic weaknesses noted in §3.5

(the tautological PASS line, and no altitude cross-check). Neither compromises
the requirement — a wrong turn genuinely cannot reach a PASS summary.

### R1 (state machine) and R4 (one choke point) — **MET.** No action.

### Design rules — **both hold.** No action. See §3.7.

---

## 7. Present but incomplete

- **`stryka_perception`, `stryka_tracking`, `stryka_guidance`** — package
  skeletons with zero implementation. The entire detector → tracker → guidance
  chain is unwritten. Three months to delivery.
- **`stryka_bringup`** — brings up only `mission_manager`. MAVROS is launched by
  hand (`sim.launch.py` docstring: "MAVROS is launched separately for now").
  There is no single-command bringup and no hardware bringup file.
- **`params.yaml`** — honest and well-commented, but by its own annotations
  largely unmeasured: `focal_px` is `MEASURE AND REPLACE`, `detection_floor_px`
  is `ASSUMED from literature`, `latency_compensation_s` is `MEASURE with
  latency_probe.py at W10`, `pn_gain` is `UNSET -- tune in simulation`.
- **Leg geometry** — every leg overshoots ~36 % (20 m commanded, 27.2–27.5 m
  flown; `checkpoint7.md` A2). Velocity-only control with no deceleration lead;
  at 5 m/s the vehicle coasts ~8 m past the 19 m arrival threshold. The same
  logic is in `first_flight.py`, so Gate 0 overshot identically.
- **Takeoff altitude gate** — `checkpoint7.md` A3: "altitude reached" fired at
  9.54 m against a 10.0 ± 0.5 m band, satisfied by passing through the band
  during the climb rather than settling in it.
- **Licensing** — every `package.xml` and `setup.py` says
  `TODO: License declaration`. No `LICENSE` file.
- **`README.md`** — one line: `# STRYKA-Interceptor`.

## 8. Missing

- Any detector, tracker or guidance node.
- `docs/state/` (created by this report). No `docs/` planning material in the
  repo at all beyond the one gate record — the execution plan, timeline, handoff
  and Mel's onboarding documents are all in `detached/`, which is gitignored and
  therefore not backed up either.
- Any test beyond the three `ament_copyright` / `ament_flake8` / `ament_pep257`
  boilerplate files per package. There is not one unit test of the state machine.
  Note that `stryka_common/test/` is empty — it has no lint tests at all, unlike
  its siblings.
- `latency_probe.py`, referenced by `params.yaml` for W10.
- Hardware bringup launch file.
- Checkpoint 7 iterations 2–4 (correctly not run after iteration 1 failed).
- Empty leftover directories from `ros2 pkg create`: `src/stryka_msgs/src/`,
  `src/stryka_msgs/include/stryka_msgs/`.

## 9. Open items, ordered by risk to the 18 December delivery

| # | Item | Risk | Recommendation |
|---|---|---|---|
| 1 | **Nothing is committed or pushed.** 56 files, all real work, exist only on this disk. A session was already lost once. | **Catastrophic / imminent.** Total loss of the project at any moment. | Commit everything and push today, before any code change. It costs minutes. |
| 2 | **Pilot-fight defect (R2).** Node countermands the pilot in 145 ms. | **Safety-critical.** On hardware this is a fly-away with a pilot who believes they have control. Blocks Checkpoint 7 and gates G3. | Implement the `_launch_authorised` latch (§4). Include the `ARMED` window fix. Then re-run Checkpoint 7 iterations 1–4. |
| 3 | **Auto-arm on launch.** Starting a ROS node arms an aircraft. | **Safety-critical**, same mechanism as #2. | Same change. Do not treat as separate work. |
| 4 | **Perception / tracking / guidance are empty.** Three of seven packages, and the entire sensing chain, unwritten with ~14 weeks left. | **Highest schedule risk.** Everything downstream of the detector is unstarted. | Not a fix — a planning item. This should drive the next milestone, and `focal_px` must be measured before the detector boundary can be written correctly. |
| 5 | **Clean build fails inside `.venv`** (missing `empy`). | **High friction, low severity.** Will block Mel or a fresh clone, and presents as a confusing rosidl error. | Either `apt install python3-empy` into the venv equivalent (`.venv/bin/pip install empy`), or recreate the venv with `--system-site-packages`, or document "do not build from inside the venv." Pick one and write it in the README. |
| 6 | **Only 4 of 7 packages have ever been built.** | **Medium.** Masks breakage in the three scaffolds; a plain `colcon build` has never been validated in-place. | Now verified working (7/7 in 4.02 s outside the venv). Run a real full build once #5 is settled. |
| 7 | **Leg overshoot ~36 %** (A2). | **Medium.** Harmless in an open SITL field; matters once flying a real geometry near obstacles or a real target. | Add a deceleration lead or switch to position targets for leg ends. Before hardware, not before Checkpoint 7 re-run. |
| 8 | **Flight parameters hard-coded (R6).** | **Medium.** Forces source edits to change altitude/speed between SITL and hardware — exactly when you least want to edit code. | Promote to `params.yaml` alongside the #2/#3 change. |
| 9 | **Setpoint rate unproven on Pi 5 under load** (R3). | **Medium, deferred.** Silent mode exit if it starves. | Measure on target hardware with the full graph. Revisit executor threading only if it degrades. |
| 10 | **Takeoff altitude band satisfied in transit** (A3). | **Low.** | Require altitude in band *and* vertical speed near zero. |
| 11 | **No unit tests of the state machine.** | **Low now, compounding.** The state machine is the safety-critical component and is verified only by flying it. | After #2, add tests for the transition table — especially that a takeover latches. A test would have caught this defect without a simulator. |
| 12 | **`abort_range_floor_m` is dead config; TODO licences; one-line README; `.vscode/` gitignore gap.** | **Low.** | Housekeeping. Bundle into the #1 commit. |

---

## 10. Confidence and limits

What I could not determine, stated plainly:

- **I did not run the code.** No SITL was started, no node was launched. All
  runtime claims here are either from reading the code or quoted from
  `checkpoint7.md` and its artefacts, attributed at each point.
- **I did not verify Checkpoint 7's CSVs independently.** I confirmed the 29
  artefact files exist with plausible sizes and timestamps, and that the gate
  record's root-cause analysis matches what I derived from the source. I did not
  re-parse `rec.iter1.setpoint.csv` to recount the 195 post-override setpoints.
- **The `.venv` build failure is reproducible but its origin is unclear.**
  The venv is auto-activated in this terminal, but not by `~/.bashrc` or an
  `.envrc` — most likely the VS Code Python extension. Whether it activates in
  *your* normal shell, I cannot tell from here.
- **Why the three scaffold packages were excluded from the last build is not
  recorded** anywhere I could find. I have assumed convenience, not a known
  breakage — and they do build (7/7 clean).
