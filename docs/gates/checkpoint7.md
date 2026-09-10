# Checkpoint 7 — Gate Record

**Result: FAIL** — the square flies correctly (north first, frame convention
clean), but the stand-down test fails: on a mode change out of GUIDED the node
stands down for ~0.5 s and then **forces the vehicle back into GUIDED on its
own and resumes the mission**. It fights the pilot. See
[Stand-down results](#stand-down-results).

Per the checkpoint constraints, `mission_manager` was **not modified** to make
the test pass, and iterations 2–4 of the stand-down test were **not run** after
iteration 1 failed conclusively (the defect is a single deterministic code path
common to every point in the square).

---

## 1. Run identification

| Item | Value |
|---|---|
| Date / time (UTC) | 2026-09-08, ~09:40Z |
| Workspace git commit | `d7162031287b82dcc0fabbaf7138c2e516a7d32b` ("Merge remote repository (README) into initial workspace") |
| Working tree state | **DIRTY.** `src/`, `tools/`, `docs/` are untracked; `.gitignore` modified. The code under test is uncommitted working-tree state on top of `d716203`. It has not been committed pending review of this record. |
| ROS 2 distro | Humble |
| MAVROS | 2.14.0 |
| ArduPilot SITL | ArduCopter, `ArduPilot-4.6.0-beta1-8348-gb9439efde1`; FCU reports `V4.8.0-dev (b9439efd)` |
| pymavlink (recorder/probe) | 2.4.49 |
| Vehicle model | SITL `+` quad, home 35.363261 / 149.165230, EKF3 |
| Raw logs | `scratch/checkpoint7/` (gitignored) — see [Artefacts](#artefacts) |

### Deviation from the written procedure

`sim_vehicle.py -v ArduCopter --console --map` could not be used: in this
environment its MAVProxy child exits immediately (terminal-window spawn path).
The stack was instead brought up as individual processes, which is functionally
equivalent for this test:

- `arducopter` SITL binary directly (TCP `:5760`)
- `mavproxy.py --master tcp:127.0.0.1:5760 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --map --console --daemon`
- `ros2 launch mavros apm.launch fcu_url:=udp://:14551@`
- `ros2 launch stryka_bringup sim.launch.py`

`sim_vehicle.py`'s default MAVProxy only forwards `14550`; `14551` was added
explicitly for MAVROS. Readiness between layers was gated on observed
conditions (TCP `:5760` listening → MAVProxy "Received 1427 parameters" → EKF3
IMU0+IMU1 using GPS, GPS `fix_type=6`, 10 sats, EKF flags `0x033F` → `/mavros/state`
`connected: true`), never on a fixed sleep. No `VER: broadcast request timeout`
was seen.

---

## 2. Square result

**Node self-assessment: PASS. Independent check from `/mavros/local_position/pose`:
first leg moved NORTH, all four legs correct.**

The node's four `leg … END` deltas were re-derived independently from the raw
`/mavros/local_position/pose` stream (recorded by a separate node on a common
wall clock) at the node's reported leg start/end timestamps. The independent
numbers match the node's, and leg 1 is unambiguously +North.

### Commanded vs measured leg deltas (ENU, metres)

| Leg | Cmd ΔN | Cmd ΔE | Meas ΔN | Meas ΔE | Along-axis travel | Cross-track error |
|---|---:|---:|---:|---:|---:|---:|
| 1 — north | +20.0 | 0.0 | **+27.23** | −0.02 | 27.23 | **0.02** |
| 2 — east  | 0.0 | +20.0 | −0.34 | **+27.45** | 27.45 | **0.34** |
| 3 — south | −20.0 | 0.0 | **−27.51** | −0.35 | 27.51 | **0.35** |
| 4 — west  | 0.0 | −20.0 | +0.35 | **−27.53** | 27.53 | **0.35** |

Square closes: end position 0.30 m E / 0.32 m S of the start point.

### Timing (wall clock, from node log)

| Phase | Duration |
|---|---:|
| ARMED → disarmed (total flight time) | **55.8 s** |
| takeoff command → altitude reached | 8.6 s |
| ENGAGE → RESULT PASS (the square) | 32.9 s |
| per leg (N / E / S / W) | 8.2 / 8.2 / 8.3 / 8.2 s |
| LAND command → disarm | 10.3 s |

### State transitions (from `node.square.log`, epoch seconds)

```
1788860203.467  STATE IDLE -> ARMED       (FCU connected and localised)
1788860204.365  STATE ARMED -> TAKEOFF    (armed in GUIDED, takeoff commanded)
1788860212.965  altitude reached: 9.54 m
1788860212.966  STATE TAKEOFF -> HOLD     (reached target altitude)
1788860216.065  STATE HOLD -> ENGAGE      (hold stable -- starting square)
1788860216.066  leg north START  ENU e=-0.02 n=0.02  alt=9.98
1788860224.265  leg north END    dNorth +27.23  dEast -0.02   frame check PASSED
1788860224.267  leg east  START  ENU e=-0.04 n=27.26 alt=10.03
1788860232.466  leg east  END    dNorth -0.34   dEast +27.45  frame check PASSED
1788860232.467  leg south START  ENU e=27.40 n=26.92 alt=10.02
1788860240.766  leg south END    dNorth -27.51  dEast -0.35   frame check PASSED
1788860240.769  leg west  START  ENU e=27.05 n=-0.60 alt=10.02
1788860248.965  leg west  END    dNorth +0.35   dEast -27.53  frame check PASSED
1788860248.970  RESULT: PASS
1788860248.971  STATE ENGAGE -> LAND      (square complete)
1788860259.266  disarmed after landing
1788860259.268  STATE LAND -> IDLE        (landed and disarmed -- mission complete)
```

---

## 3. Measured setpoint rate

| | Value |
|---|---|
| Configured (`mission.setpoint_rate_hz`) | 20.0 Hz |
| Measured, mean over the flight | **20.00 Hz** (720 setpoints on `/mavros/setpoint_raw/local` over 35.9 s) |
| Measured, rolling 1-s window | median 20 Hz, steady at 20 Hz for the whole flight |
| `type_mask` on every setpoint | `3527` (velocity-only: position + accel + yaw + yaw-rate ignored) — correct |

**Not materially below configured.** No risk of ArduPilot dropping autonomous
mode from setpoint starvation in this run.

---

## 4. Stand-down results

Procedure per iteration: bring up node, let it fly to the target point,
`ros2 service call /mavros/set_mode "{custom_mode: 'LOITER'}"`, measure. The
override time is taken as the first sample on `/mavros/state` showing a non-GUIDED
mode (recorded on the same wall clock as the setpoint stream).

### Iteration 1 — during the first leg (north)

Fired ~4.7 s into leg north (vehicle ~23 m north of start, mid-leg, setpoints
confirmed streaming at 20 Hz immediately before).

Timeline, relative to LOITER first seen on `/mavros/state` (T0 = 1788860444.3132):

| T (s) | Event | Source |
|---:|---|---|
| −0.016 | last GUIDED-mission setpoint published (`vy=+5.0`) | `setpoint.csv` |
| 0.000 | **LOITER** appears on `/mavros/state` | `state.csv` |
| +0.001 | node logs `STATE ENGAGE -> IDLE (mode left GUIDED (now 'LOITER') -- pilot has control)` | `node.iter1.log` |
| +0.036 | node logs `STATE IDLE -> ARMED (FCU connected and localised)` | `node.iter1.log` |
| +0.137 | node logs `requested mode GUIDED` + `set_mode GUIDED response: mode_sent=True` | `node.iter1.log` |
| **+0.145** | **GUIDED re-appears on `/mavros/state` — the node has overridden the pilot** | `state.csv` |
| +0.487 | node resumes publishing setpoints (`vx=vy=0`, from re-entered HOLD) | `setpoint.csv` |
| +3.54 | node logs `STATE HOLD -> ENGAGE (hold stable -- starting square)` + `leg north START` again | `node.iter1.log` |
| +10.19 | last setpoint (test harness kills the node here) | `setpoint.csv` |

| Criterion | Result |
|---|---|
| Setpoint publishing stops promptly | Partial — halted within ~1 control tick (≤50 ms); 0.50 s gap in the stream |
| Setpoint publishing stops **without exception** | **FAIL** — resumed 0.487 s later and continued (195 setpoints published after the override) |
| Transitioned to `IDLE` | Yes — momentarily (held IDLE for ~36 ms) |
| Log named the mode change as the trigger | Yes — `mode left GUIDED (now 'LOITER') -- pilot has control` |
| Aircraft held position | **FAIL** — node re-armed, re-took-off, resumed the square; position drift 36 m by the time the harness intervened |
| Node did **not** attempt to re-enter GUIDED | **FAIL** — node commanded GUIDED 0.145 s after the pilot selected LOITER; `/mavros/state`: `…GUIDED@444.236 → LOITER@444.313 → GUIDED@444.458` |
| **Stand-down latency (LOITER on /state → last setpoint)** | **not achieved** — the stream never stayed down. Momentary halt ≈ 1 control tick; sustained stand-down: **never** in a 10 s window. |

**Iteration 1 verdict: FAIL.**

### Iterations 2, 3, 4 — not run

Stopped after iteration 1 per the checkpoint constraint ("If it fails, report
the failure with evidence and stop"). The failure is not position-dependent:
the offending path (`IDLE` immediately restarts the mission) executes
identically regardless of where in the square the mode change occurs.

### Root cause (from observed behaviour — no code changed)

Behaviour 1 ("never fight the pilot") is implemented for *detection* but not as
a *latch*:

- `_on_fcu_state` correctly detects `mode != GUIDED` while flying and calls
  `_transition(IDLE, "mode left GUIDED …")`. Setpoints de-authorised. Good.
- But entering `IDLE` **clears** the service-request guards
  (`_expect_guided=False`, `_guided_requested_at=None`, …) and sets nothing to
  remember that a pilot takeover happened.
- `_tick_idle` restarts the mission whenever `/mavros/state` is `connected` and
  a pose is available, gated **only** by `self._mission_complete` — which a
  pilot takeover does not set.
- Net effect: on the very next 10 Hz control tick the node goes
  `IDLE → ARMED`, re-requests GUIDED, re-commands takeoff, and re-flies the
  square. It countermands the pilot in ~145 ms.

A correct implementation needs a terminal/latched state on pilot takeover
(e.g. `IDLE` after a takeover must require an explicit external re-arm, or a
dedicated `STANDDOWN` state that never self-exits). This is a design change to
`mission_manager` and is out of scope for this checkpoint run.

---

## 5. Anomalies

### A1 — Stand-down failure (see §4). Blocking.

### A2 — Leg overshoot ~36 %. Non-blocking for this gate, must fix before hardware.

Every leg was commanded as 20 m and measured 27.2–27.5 m. Cause: velocity-only
control with arrival detected at `LEG_LENGTH − ARRIVAL_TOLERANCE = 19 m`, then a
stop-and-settle phase. At 5 m/s the vehicle coasts ~8 m before the settle phase
arrests it — there is no deceleration lead or position target. Cross-track
stayed < 0.4 m so the frame-convention verification is unaffected and the square
still closes, but the flown geometry is not the commanded geometry. The same
logic exists in `tools/first_flight.py`, so the Gate 0 MAVLink run is expected
to overshoot identically.

### A3 — Takeoff altitude band is marginal.

`altitude reached` logged at 9.54 m (square run) and 9.66 m (stand-down run)
against a 10.0 m target with a ±0.5 m acceptance band — inside the band by
0.04–0.16 m. The band is being satisfied by the arrival of the vehicle from
below during the climb, not by settling at target. Worth tightening the check
(require altitude within band *and* vertical speed near zero) so "reached 10 m"
means it.

### A4 — `ros2 service call` client setup latency.

The one-shot `ros2 service call` used to force the mode change took ~1.6 s to
bind to `/mavros/set_mode` ("waiting for service to become available"). This is
a property of the *test harness*, not the node, and does not affect the
stand-down measurement (which is taken from when LOITER appears on
`/mavros/state`, downstream of the service call). Noted so the ~1.6 s in the raw
`mark.iter1.txt` is not misread as node latency.

---

## 6. Open items — what this run did not prove

- **Sustained stand-down / no-fight-the-pilot behaviour.** Failed; must be
  re-tested after the `mission_manager` design fix.
- **Stand-down latency as a number.** Cannot be reported — the stream never
  stayed down. Re-measure once A1 is fixed.
- **Iterations 2–4** (mid-second-leg, at a corner, final leg). Not run.
- **Leg geometry accuracy.** The square passes the frame gate but overshoots
  ~36 % (A2). Not addressed here.
- **Behaviour under a real mode change from an RC transmitter** — see below.
- **`ENGAGE` with an actual target**, guidance, detector — out of scope this week.
- **Setpoint rate under CPU load / on the Pi 5** — this was a desktop SITL run
  with the FCU in the loop at 20 Hz and no perception stack running.

---

## What this does not test

- **This used a software-commanded mode change** (`ros2 service call
  /mavros/set_mode`), **not a pilot moving a mode switch on an RC transmitter.**
  The real override path — RC → receiver → FCU → mode change → MAVLink heartbeat
  → MAVROS → our node — has more latency and more failure modes (RC link loss,
  failsafe interaction, mode-switch debounce). The authoritative override test
  is **G3 on hardware**. Passing Checkpoint 7 does not anticipate G3, and G3
  must be run with the safety pilot in command on a real radio.
- SITL physics are idealised: no wind gusts beyond the sim model, perfect EKF
  convergence, no GPS glitching, no vibration, RTK-fixed GPS from t=0.
- The companion computer here is a desktop, not the Pi 5. Scheduling jitter,
  thermal behaviour and the setpoint rate with the full ROS graph running are
  not represented.
- MAVROS ran on the same host as SITL over loopback UDP — no real serial/radio
  link latency or dropouts.

---

## Artefacts (in `scratch/checkpoint7/`, gitignored)

| File | Contents |
|---|---|
| `node.square.log` | mission_manager stdout, square run |
| `square.{pose,setpoint,state}.csv` | independent recorder, square run |
| `analyze_square.py` | leg-delta + rate post-processor |
| `node.iter1.log` | mission_manager stdout, stand-down iteration 1 |
| `rec.iter1.{pose,setpoint,state}.csv` | independent recorder, iteration 1 |
| `mark.iter1.txt` | service-call transcript + timestamps, iteration 1 |
| `analyze_standdown.py` | stand-down latency post-processor |
| `recorder.py`, `probe.py`, `standdown_iter.sh` | test tooling |
| `sitl.log`, `arducopter.log`, `mavproxy.log`, `mavros.log` | stack bring-up logs |
| `mav.tlog`, `mav.tlog.raw` | MAVProxy telemetry logs |

---

## Teardown

Stack shut down in reverse order (node → MAVROS → MAVProxy → SITL). Confirmed no
`mission_manager`, `mavros_node`, `mavproxy`, `arducopter` or `sim_vehicle`
processes remain; ports 5760 / 5501 / 14550 / 14551 all free.
