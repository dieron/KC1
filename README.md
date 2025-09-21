# KC1 — Kayak Controller (Arduino Uno)

Differential‑thrust kayak controller with Normal & Air modes, optional heading hold (BNO055), runtime serial configuration/telemetry API, and safety‑first arming & failsafe behaviors.

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

- DISARMED: Neutral outputs (1500 µs). Transition to armed only when yaw & throttle centered and a CH3/CH4 toggle event (>500 µs jump) occurs.
- NORMAL: Direct differential mix (L=Thr+Yaw, R=Thr−Yaw).
- AIR: Latched setpoints increment toward stick deflection each loop (trim‑style “cruise” control). Retoggling Air while already in Air instantly neutralizes (one-cycle skip flag).
- HEADING HOLD: When enabled (HEAD ON command or CH5 toggle), captures current compass heading, applies PID yaw correction with speed‑aware strategy:
  - Near zero throttle: spin in place (opposite sign motors within configured min/max)
  - High speed: subtract from one side only
  - Mid speeds: differential add/subtract on both sides
  - Small error inside deadband: integrator decay + base commands restored (no bias)

## Runtime Configuration & Persistence

`ConfigStore` provides named float parameters persisted in EEPROM. Runtime changes via serial API `CFG SET` auto‑save immediately. Factory defaults recoverable with `CFG RESET`.

Key parameters (names exactly as used by API):

- Motion / shaping: `reverseLeft`, `reverseRight`, `motorExpoL`, `motorExpoR`
- Heading: `headKp`, `headKi`, `headKd`, `headCmdMax`, `headingDeadbandDeg`, `speedZeroThresh`, `speedHighFrac`, `spinCmdMin`, `spinCmdMax`, `headingHoldEnabled`
- Failsafe & input quality: `deadCenter`, `staleTimeoutMs`, `airGainPerCycle`, `failsafeStuckThr`, `stuckDeltaUs`, `stuckCycles`

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

- `STATUS mode=NRM armed=1 hold=0 bno=1 fsHold=0 cmdL=120 cmdR=100 usL=1560 usR=1550`
- `RC yawUs=1510 thrUs=1500 nUs=1000 aUs=1000 hUs=1000 yaw=20 thr=0 valid=1`
- `MOTORS cmdL=... cmdR=... usL=... usR=...`
- `HEADING bno=1 cur=123.45 hold=1 tgt=125.00 err=-1.55`
- `ALL ...` (STATUS + RC + MOTORS + optional heading details consolidated)

Heading control:

- `HEAD ON` captures current heading if sensor OK; starts correction.
- `HEAD OFF` disables correction.
- `HEAD SET 270` sets absolute target (normalized 0–359.99).
- `HEAD TARGET` prints current target.

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

## Heading Hold Implementation Highlights

- Angular difference via shortest signed path (wrap‑aware −180..+180).
- PID with integral decay inside half the deadband and integrator reset when `HEAD SET` used.
- Sign inversion applied so positive heading error (target CCW ahead) yields correct physical turn direction according to differential mix conventions.
- Speed regime logic picks spin / balanced / one‑sided reduction strategy.

## Future / TODO (abridged)

- Add yaw inversion runtime config (currently fixed inversion in code).
- Add extended `TELEM HEADING` with raw error & applied yawCmd.
- Alternate soft console avoiding SoftwareSerial ISR conflicts (e.g. bit‑bang TX only).

## Quick Start

1. Wire RC inputs (D4–D7 + optional D8 for heading hold toggle), ESCs (D9/D10), BNO055 (A4/A5), Bluetooth (optional) and common ground.
2. Flash `[env:uno]`. Open Serial at 9600. Prompt appears: `KC1 - Kayak Controller (Uno)` then `> `.
3. Send `HELP` to list commands.
4. Arm: center sticks, toggle CH3 or CH4.
5. Enable heading hold: `HEAD ON` (with BNO055 present) or CH5 toggle.

## Disclaimer

Hobby / experimental firmware. Validate motor direction, limits, and failsafes on the bench before water trials. Use at your own risk.

---

Authored with assistance from GitHub Copilot / GPT tooling.
