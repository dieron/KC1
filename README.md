# KC1 — Kayak Controller (Arduino Uno)

API Version: **0.2.0**

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
```

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

## Disclaimer

Hobby / experimental firmware. Validate motor direction, limits, and failsafes on the bench before water trials. Use at your own risk.

---

Authored with assistance from GitHub Copilot / GPT tooling.
