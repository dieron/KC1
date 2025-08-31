# KC1 — Kayak Controller for Arduino Uno

A safety‑minded, differential‑thrust controller for an RC kayak using an Arduino Uno. It reads 4 RC PWM channels, provides two drive modes (Normal and Air), and outputs two low‑jitter 50 Hz ESC signals.

Note: All code in this repository was authored with the help of GitHub Copilot and GPT‑5 (preview).

## Features

- 4‑channel RC input via Pin Change Interrupts (D4–D7)
  - CH1: Yaw (D4)
  - CH2: Throttle (D5)
  - CH3: Normal mode button/switch (D6)
  - CH4: Air mode button/switch (D7)
- Input mapping: 1000–2000 µs → −1000..+1000 with center deadband
- Modes
  - Normal: Direct differential mixing (L = Thr + Yaw, R = Thr − Yaw)
  - Air: Latched setpoints that adjust incrementally each loop from stick deflection (great for trimming while hands‑off)
  - Air “click to neutral”: Clicking the Air button again while in Air instantly sets both motors to neutral (1500 µs) for one cycle
- ESC outputs at 50 Hz using Servo (Timer1), on pins:
  - Left ESC: D9
  - Right ESC: D10
- Output shaping
  - Per‑motor direction reversal flags
  - Adjustable exponential response curve (0 = linear, 1000 = strong expo)
- Safety
  - Powers up Disarmed and drives both ESCs at neutral (1500 µs)
  - Arming requires sticks centered and a CH3/CH4 toggle event (> ~500 µs change)
  - While armed, CH3/CH4 toggles switch modes
  - RC‑stale detection with debounce disarms to neutral
  - Built‑in “neutral‑hold” heuristic for receivers that keep outputting fixed failsafe values (see Failsafe notes)
- Debug
  - Compile‑time selectable debug: verbose or single‑line pseudo‑graphics
  - Pseudo‑graphics prints mode/armed/buttons/motor bars and raw channel microseconds on a single terminal line
- Timing: Steady ~20 ms loop using millis() (about 50 Hz)

## Wiring

- Receiver PWM inputs to Arduino:
  - CH1 (Yaw) → D4
  - CH2 (Throttle) → D5
  - CH3 (Normal btn/switch) → D6
  - CH4 (Air btn/switch) → D7
- ESC signal wires:
  - Left ESC → D9
  - Right ESC → D10
- Common ground between Arduino, receiver, and ESC BEC(s) is required.

## Quick manual

- Power‑up: Controller is Disarmed. Both ESC outputs are 1500 µs.
- Arm:
  - Center yaw and throttle sticks (deadband around 1500).
  - Toggle CH3 (Normal) or CH4 (Air). A single toggle event (> ~500 µs change) arms.
  - If CH3 toggled: arms in Normal mode. If CH4 toggled: arms in Air mode.
- While armed:
  - Toggle CH3 or CH4 to switch modes.
  - In Air mode, clicking CH4 again returns both motors to neutral immediately for one cycle.
- Disarm:
  - Automatic on RC‑stale detection (see Failsafe), or power cycle.

## Failsafe notes

- RC‑stale: If CH1/CH2 pulses stop updating, the controller disarms to neutral (debounced to avoid spurious trips).
- Receivers that output fixed failsafe values: Some PWM receivers continue outputting “default” values when the transmitter is off, so true loss can’t be inferred from pulse presence alone. Options:
  1. Configure the receiver to put a dedicated channel at a unique failsafe value (e.g., < 950 µs) and wire it for explicit disarm.
  2. Use the built‑in neutral‑hold heuristic that holds both motors at 1500 µs when yaw+throttle are essentially unchanged for an extended time, without disarming. This is enabled by default and can be turned off via a build flag (see below).
- Protocols like SBUS/IBUS/CRSF provide an explicit failsafe bit and are more robust; can be added later.

## Debugging output

- Build flags control debug:
  - `DEBUG_ENABLED=1` enables debug prints.
  - `DEBUG_STYLE=0` verbose multi‑line; `DEBUG_STYLE=1` pseudo‑graphics single‑line.
- Pseudo‑graphics renders one compact line refreshed with a carriage return. It shows mode, armed state, button states, motor bars, and raw Y/T/N/A microseconds.

Current defaults in `platformio.ini` set `DEBUG_ENABLED=1` and `DEBUG_STYLE=1` (pseudo‑graphics on by default). Adjust as needed.

## Build and upload (PlatformIO)

- Requirements: PlatformIO (VS Code extension) with an Arduino Uno environment.
- This repo includes `platformio.ini` with the Servo library dependency.
- Steps (VS Code):
  1. Open the workspace folder.
  2. PlatformIO: Project Tasks → Build.
  3. Connect the Uno → Project Tasks → Upload.
  4. Open the Serial Monitor at 115200 baud for debug (if enabled).

## Tuning

- Deadband, Air mode gain per cycle, output shaping, and failsafe thresholds are defined as macros in `src/main.cpp` and can be overridden via PlatformIO build flags.

### Configuration via build flags

You can change behavior without editing code by updating `build_flags` in `platformio.ini`.

- Debug
  - `DEBUG_ENABLED` = 0/1 (default 0 in code; currently 1 in `platformio.ini`)
  - `DEBUG_STYLE` = 0 verbose, 1 pseudo‑graphics (default 0; currently 1)
- Output shaping
  - `REVERSE_LEFT`, `REVERSE_RIGHT` = 0 normal, 1 reverse (default 0; currently both 1)
  - `MOTOR_EXPO` = 0..1000 global expo (default 400)
  - `MOTOR_EXPO_L`, `MOTOR_EXPO_R` override per‑motor expo (default inherit `MOTOR_EXPO`)
- Failsafe heuristic for constant outputs
  - `FAILSAFE_STUCK_THR` = 0/1 enable neutral‑hold (default 1 = enabled)
  - `STUCK_DELTA_US` = microsecond tolerance to treat as “unchanged” (default 1)
  - `STUCK_CYCLES` = consecutive 20 ms cycles before neutral‑hold engages (default ~75 ≈ 1.5 s)

Example `platformio.ini` snippet (current):

```ini
[env:uno]
platform = atmelavr
board = uno
framework = arduino
lib_deps = arduino-libraries/Servo
build_flags = \
  -D DEBUG_ENABLED=1 \
  -D DEBUG_STYLE=1 \
  -D REVERSE_LEFT=1 \
  -D REVERSE_RIGHT=1 \
  -D MOTOR_EXPO=400
```

## Disclaimer

This is hobby software for an RC kayak. Always test on the bench first, verify directions and failsafe behavior, and use proper safety practices on the water.

---

Authored with the help of GitHub Copilot and GPT‑5 (preview).
