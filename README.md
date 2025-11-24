# KC1 — Kayak Controller (Arduino Uno)

> **Disclaimer:** This project is developed purely via vibe-coding. No manual code edits or fixes are performed; the code isn't always reviewed. Despite that, the controller is used on a real kayak on the water.

> **Physical Project:** There is also a physical project where we build the motors with a controller. This will be published later.

> **Android App:** An Android companion app is available that allows you to monitor the controller status, configure parameters on the fly, and control the kayak directly from your phone. Link will be added later.

API Version: **0.4.3.0**

Differential‑thrust kayak controller with Normal, Air, and dedicated Heading (compass) modes, runtime serial configuration/telemetry API, and safety‑first arming & failsafe behaviors.

## Current Hardware / Pin Map

- RC PWM inputs via Pin Change Interrupts (PORTD D4–D7 + PORTB D8 for heading-hold toggle)
  - CH1 Yaw: D4
  - CH2 Throttle: D5
  - CH3 Normal toggle: D6
  - CH4 Air toggle: D7
  - CH5 Heading Hold toggle: D8
- ESC outputs (Servo / Timer1): Left D9, Right D10
- BNO055: I2C (SDA=A4, SCL=A5). Auto address detect (0x28 then 0x29 fallback)
- Optional HC‑06 Bluetooth on hardware UART (shared with USB). Recommended: HC‑06 TX → D0 through 1k series; D1 → HC‑06 RX via divider.

## Modes & Control

| Mode     | How to Enter                                       | How to Exit                                                   | Behavior Summary                                                                                                     |
| -------- | -------------------------------------------------- | ------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| DISARMED | Power‑up, `RESET`, stale RC                        | Arm via CH3 (Normal) or CH4 (Air) toggle with sticks centered | Motors neutral (1500 µs)                                                                                             |
| NORMAL   | CH3 toggle while disarmed or from other armed mode | Switch to AIR (CH4), HEADING (CH5/HEAD ON) or DISARM          | Direct differential mix (L=Thr+Yaw, R=Thr−Yaw)                                                                       |
| AIR      | CH4 toggle while disarmed or from other armed mode | Toggle CH4 again (neutralize), go NORMAL/HEADING, or DISARM   | Latched setpoints (`air_gain_per_cycle` per loop at full stick) for cruise-like trimming                             |
| HEADING  | CH5 toggle or `HEAD ON` (armed + BNO055 OK)        | `HEAD OFF` (to NORMAL) or CH3/CH4 to NORMAL/AIR or DISARM     | Holds captured heading with yaw PID; forward speed is incremental cruise adjusted by throttle (`hdg_gain_per_cycle`) |

Heading mode particulars:

- Entering from AIR preserves forward speed using the latched Air throttle setpoint; from NORMAL it derives from current motor outputs.
- Throttle stick center ≈ hold; forward increases cruise speed; back decreases (never reverses below zero).
- Yaw stick is ignored for turning—PID provides yaw corrections.
- Exiting via `HEAD OFF` resets heading speed to zero (NORMAL mode).

## Runtime Configuration & Persistence

`ConfigStore` provides named float parameters persisted in EEPROM. Runtime changes via serial API `CFG SET` auto‑save immediately. Factory defaults recoverable with `CFG RESET`.

Key parameters:

Motion / shaping:

- `reverseLeft`, `reverseRight`, `motorExpoL`, `motorExpoR`

Heading & yaw correction:

- `headingHoldEnabled`, `headingDeadbandDeg`
- `headKp`, `headKi`, `headKd`, `headCmdMax`
- `speedZeroThresh`, `speedHighFrac`, `spinCmdMin`, `spinCmdMax`
- `hdg_gain_per_cycle` (Heading mode forward speed increment per loop at full stick)
- `head_boost_trigger_mult` (Multiplier of deadband where adaptive yaw boost begins; default 2.0)
- `head_max_boost` (Maximum adaptive yaw boost multiplier; default 1.35)

Motor trim / thrust balancing:

- `motor_start_us_l`, `motor_start_us_r` (µs-equivalent start offsets applied to small magnitude commands so both motors begin spinning together; default 0)
- `motor_scale_l`, `motor_scale_r` (per-motor scalar applied to command magnitude after expo & reversal; default 1.0; clamped internally ~0.5–1.5)
- `motor_start_region` (command-domain magnitude threshold under which start offsets are applied; default 150)

Cruise / setpoint:

- `air_gain_per_cycle` (Air mode incremental gain)

Failsafe & input quality:

- `deadCenter`, `staleTimeoutMs`, `failsafeStuckThr`, `stuckDeltaUs`, `stuckCycles`

Use `CFG LIST` to view current values (one `name=value` per line).

## Serial API (single hardware UART, 9600 baud by default)

Commands are case‑insensitive, line terminated by CR or LF. Responses end with a new prompt `> `. Errors: `ERR: message`.

Core commands:

```
HELP
CFG LIST
CFG GET <name>
CFG SET <name> <value>
CFG RESET
CFG SAVE
TELEM STATUS
TELEM RC
TELEM MOTORS
TELEM HEADING
TELEM ALL
HEAD ON
HEAD OFF
HEAD SET <deg>
HEAD TARGET
RESET
VERSION
```

- `CFG META JSON` prints a JSON array of parameter metadata objects with fields:
  `name,type,min,max,step,default,units,desc`.

Telemetry line formats (examples):

- `STATUS mode=HDG armed=1 bno=1 fsHold=0 cmdL=300 cmdR=300 usL=1680 usR=1680`
- `RC yawUs=1510 thrUs=1500 nUs=1000 aUs=1000 hUs=1000 yaw=20 thr=0 valid=1`
- `MOTORS cmdL=... cmdR=... usL=... usR=...`
- `HEADING bno=1 cur=123.45 hold=1 tgt=125.00 err=-1.55`
- `ALL ...` (STATUS + RC + MOTORS + optional heading details consolidated)

Heading mode / compass control:

- `HEAD ON` (or CH5) enters heading mode, capturing current heading & initial speed.
- `HEAD OFF` leaves heading mode (returns to NORMAL, zero speed).
- `HEAD SET <deg>` sets absolute target heading; resets PID integrator.
- `HEAD TARGET` prints current heading target.

Reset:

- `RESET` disarms, neutralizes outputs, clears heading hold.

Version / identity:

- `VERSION` prints: `VERSION api=<semver> git=<hash> build=<DATE>T<TIME>`.
  - `git` hash can be injected at build via PlatformIO `build_flags` (e.g. `-DKC1_GIT_HASH=\"abcd123\"`).
  - If not supplied, shows `unknown`.

## Safety & Failsafes

- RC stale detection: lack of fresh Yaw/Throttle pulses beyond `staleTimeoutMs` (ms) → disarm.
- Stuck-signal heuristic: if both Yaw & Throttle remain within `stuckDeltaUs` for `stuckCycles` and `failsafeStuckThr` enabled → motors held neutral (armed state preserved) to avoid runaway if receiver freezes outputs.
- Disarm always sets motors to 1500 µs and clears heading hold integrators.

## Debug Output (optional)

Compile‑time flags:

- `DEBUG_ENABLED` (0/1)
- `DEBUG_STYLE` 0=verbose per loop line, 1=pseudo‑graphics single updating line.
  Pseudo‑graphics includes mode, armed, buttons, motor bars, raw input microseconds, and concise heading snapshot.

## Build

Environment: `[env:uno]` (default and only). Typical commands:

```
pio run -e uno            # build
pio run -e uno -t upload  # build + flash
```

## Bluetooth (HC‑06) Notes

- Factory default baud is usually 9600; firmware currently uses 9600 to match.
- To change module baud manually: disconnect any active BT pairing, talk at its current baud, send `AT` (expect `OK`), then e.g. `AT+BAUD7` (57600) or desired code, and re-open Serial at the new rate. A temporary minimal passthrough/echo sketch can be used if needed.
- Use a 1k series resistor HC‑06 TX → D0 to limit contention with onboard USB interface; Arduino TX → HC‑06 RX through a divider if module RX is not 5V tolerant.
- Do not open the PC Serial Monitor while also using a Bluetooth terminal (shared UART collisions cause garbled commands).

## Heading Mode Implementation Highlights

- Shortest wrap‑aware angular difference (−180..+180) for error.
- PID: integral decay inside half deadband; integrator reset on target change; mild anti‑windup near saturation.
- Adaptive linear yaw authority boost: begins at `headingDeadbandDeg * head_boost_trigger_mult`, ramps to `head_max_boost` by ~90° error (unless already near max authority) so large deviations correct faster without overshoot.
- Speed regime logic: stationary spin (bounded by `spinCmdMin/Max`), blended differential, or high‑speed one‑sided reduction.
- Separate incremental forward cruise gain: `hdg_gain_per_cycle` (distinct from `air_gain_per_cycle`).

## Motor Trim & Thrust Balancing

Electric kayak drives often exhibit asymmetry: one ESC+motor may not start until a higher PWM value or may produce slightly more/less thrust for the same command. KC1 provides lightweight per‑motor trim parameters that act only at the output stage (they do not alter reported command-domain telemetry):

1. Start Offsets (`motor_start_us_l`, `motor_start_us_r`)

- Applied symmetrically (forward & reverse) when 0 < |command| < `motor_start_region`.
- Conceptually adds a few microseconds worth of “kick” (in command-domain approximation) so a sluggish motor begins spinning as early as the other.
- Not applied at exact neutral (command = 0).

2. Scaling (`motor_scale_l`, `motor_scale_r`)

- Multiplies the (possibly offset-adjusted) magnitude to balance sustained thrust.
- Final magnitude is clamped to ±1000 before mapping to 1000–2000 µs.

3. Start Region (`motor_start_region`)

- Defines the command magnitude window (default 150) considered “near start” where offsets are applied.
- Increasing widens the zone if a motor still lags at slightly higher low throttle.

Telemetry Integrity: `cmdL`, `cmdR` shown in STATUS/ALL remain the post-trim commands (after scaling & offset) because they reflect actual thrust intent. RC inputs and heading/yaw setpoints are never modified by trim; only the mapping to final microseconds is influenced.

### Suggested Calibration Procedure

1. Ensure both `motor_scale_*` = 1.0 and `motor_start_us_*` = 0; leave `motor_start_region` at 150 initially.
2. With the craft secure, slowly raise throttle in NORMAL mode from neutral and note the command value (approx yaw=0) where each motor first spins.
3. If (for example) right motor starts noticeably later, increment `motor_start_us_r` (e.g. +10) and re-test until both begin within a small stick movement.
4. Perform a low to mid (e.g. command ~400–500) forward thrust with yaw centered. If kayak drifts/yaws without yaw input, reduce scale of the stronger side or increase scale of the weaker side (`motor_scale_*` adjustments in small steps, e.g. 0.02–0.05).
5. After changes, verify high-throttle symmetry; avoid pushing scale outside 0.8–1.2 unless hardware disparity is large.
6. If a motor still lags slightly above the start offset window, raise `motor_start_region` (e.g. 200) and re-evaluate.

Safety: Always perform calibration out of the water or firmly restrained. Small increments reduce risk of sudden yaw.

## Future / TODO (abridged)

- Add yaw inversion runtime config (currently fixed inversion in code).
- Add extended `TELEM HEADING` with raw error & applied yawCmd.
- Alternate soft console avoiding SoftwareSerial ISR conflicts (e.g. bit‑bang TX only).

## Quick Start

1. Wire RC inputs (D4–D7 + optional D8 for heading hold toggle), ESCs (D9/D10), BNO055 (A4/A5), Bluetooth (optional) and common ground.
2. Flash `[env:uno]`. Open Serial at 9600. Prompt appears: `KC1 - Kayak Controller (Uno)` then `> `.
3. Send `HELP` to list commands.
4. Arm: center sticks, toggle CH3 (Normal) or CH4 (Air).
5. Enter heading mode: `HEAD ON` (BNO055 required) or CH5 toggle.
6. Adjust heading-mode cruise speed with throttle stick (center hold / forward increase / back decrease).

## Parameter Quick Reference (Selected)

| Category | Parameter                           | Default          | Notes                                         |
| -------- | ----------------------------------- | ---------------- | --------------------------------------------- |
| Heading  | heading_deadband_deg                | 5.0              | Error band where corrections stop / decay     |
| Heading  | head_kp / head_ki / head_kd         | 3.5 / 0.05 / 0.8 | PID gains                                     |
| Heading  | head_cmd_max                        | 400              | Max yaw command magnitude                     |
| Heading  | head_boost_trigger_mult             | 2.0              | Multiplier of deadband where boost starts     |
| Heading  | head_max_boost                      | 1.35             | Max adaptive boost factor                     |
| Heading  | hdg_gain_per_cycle                  | 15               | Forward speed increment per loop (full stick) |
| Air Mode | air_gain_per_cycle                  | 40               | Setpoint increment per loop (full stick)      |
| Motors   | motor_expo_l / motor_expo_r         | 400              | Output shaping (0..1000)                      |
| Motors   | motor_start_us_l / motor_start_us_r | 0                | Start offset (µs equiv)                       |
| Motors   | motor_scale_l / motor_scale_r       | 1.0              | Per-motor thrust scalar                       |
| Motors   | motor_start_region                  | 150              | Command magnitude zone for start offsets      |
| Failsafe | stale_timeout_ms                    | 100              | Channel stale detection                       |
| Failsafe | stuck_delta_us / stuck_cycles       | 1 / 75           | Stuck-signal heuristic                        |
| Mixing   | spin_cmd_min / spin_cmd_max         | 180 / 700        | Stationary spin bounds                        |
| Mixing   | speed_zero_thresh                   | 50               | Below -> treat as stationary                  |
| Mixing   | speed_high_frac                     | 0.80             | Above -> high-speed yaw strategy              |

Full list: `CFG LIST`.

## Versioning Policy

KC1 now uses a 4-part semantic version: MAJOR.MINOR.PATCH.HOTFIX

- MAJOR: Manual bump for fundamental/architectural or backward-incompatible changes.
- MINOR: Significant feature additions (e.g. new mode, large subsystem) that remain backward compatible.
- PATCH: Incremented for smaller features, enhancements, new parameters, internal refactors that preserve compatibility.
- HOTFIX: Urgent post-release corrections (regressions, critical bug) applied without other changes.

Example progression: 0.3.1.0 (small enhancement) → 0.3.2.0 (another minor feature) → 0.3.2.1 (hotfix) → 0.4.0.0 (larger feature set) → 1.0.0.0 (stable milestone / external interface freeze candidate).

## CHANGELOG (abridged)

### 0.4.3.0

- **Major BNO055 Compass Reliability Improvements**:
  - Set NDOF operation mode explicitly for proper sensor fusion
  - Added magnetometer calibration status monitoring (requires mag cal ≥2 for reliable heading)
  - Implemented stale/stuck data detection (rejects readings unchanged for >30s)
  - Added range validation (0-360°) and NaN checking
  - Increased I2C clock to 400kHz for better communication reliability
  - Added sensor health checking with automatic error recovery
  - Enhanced `TELEM HEADING` with calibration status, failure count, and timestamp
  - Added startup calibration guidance messages
- Fixes erratic compass behavior caused by uncalibrated magnetometer and missing operation mode configuration.

### 0.4.2.0

- Added `compass_correction_deg` parameter (0..359) to correct for BNO055 mounting orientation.
- Compass correction is automatically applied to all heading readings and target heading when entering heading hold mode.
- Added `TEST CONFIG` and `TEST SETDEBUG` commands for ConfigStore debugging.
- **EEPROM Compatible**: New parameter added at end of structure, existing settings preserved on upgrade.

### 0.4.0.0

- Added schema/metadata API: `CFG META <name|ALL|JSON>`, `CFG SET` now clamps values to schema and snaps to step.
- JSON export for client settings UIs.

### 0.3.1.0

- Introduced formal 4-part version scheme (MAJOR.MINOR.PATCH.HOTFIX) and bumped PATCH for this documentation/identity enhancement.
- `VERSION` now reports the 4-part string (e.g. `0.3.1.0`).

### 0.3.0

- Added per-motor trim & scaling: `motor_start_us_l`, `motor_start_us_r`, `motor_scale_l`, `motor_scale_r`, `motor_start_region`.
- Exposed configurable `motor_start_region` and integrated into output stage.
- Documentation: Motor trim & thrust balancing section, quick reference table, changelog.

### 0.2.x

- Added adaptive heading boost (`head_boost_trigger_mult`, `head_max_boost`).
- Dedicated Heading mode (MODE_HEADING) with incremental cruise speed (`hdg_gain_per_cycle`).

### 0.1.x

- Initial Normal & Air modes, RC capture, EEPROM config, basic heading hold.

## Disclaimer

Hobby / experimental firmware. Validate motor direction, limits, and failsafes on the bench before water trials. Use at your own risk.

---

Authored with assistance from GitHub Copilot / GPT tooling.
