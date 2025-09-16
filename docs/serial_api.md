# KC1 Serial Configuration & Telemetry API (v1)

Baud: 115200 8N1
Line endings: LF or CRLF (both terminate). Prompt: `>` after each processed line.
Commands: Case-insensitive. Tokens separated by one or more spaces or tabs.

Responses:

- Success (state change or info): Specific line or `OK`
- Error: `ERR: <reason>` (generic reasons: `unknown`, `missing`, `bad number`, `feature disabled`)

## Command Summary

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

## Configuration Parameters

Names correspond exactly to internal storage fields (snake_case). Types drive coercion & clamping.

| Name                 | Type  | Stored Range (enforced) | Practical / Recommended Range | Description                                            |
| -------------------- | ----- | ----------------------- | ----------------------------- | ------------------------------------------------------ | --- | ---------------------------- |
| reverse_left         | u8    | 0..255                  | 0 or 1                        | Invert left motor output sign                          |
| reverse_right        | u8    | 0..255                  | 0 or 1                        | Invert right motor output sign                         |
| motor_expo_l         | u16   | 0..65535                | 0..1000                       | Expo shaping left motor (0 linear)                     |
| motor_expo_r         | u16   | 0..65535                | 0..1000                       | Expo shaping right motor                               |
| heading_hold_en      | u8    | 0..255                  | 0 or 1                        | Master enable for heading hold feature                 |
| heading_deadband_deg | float | IEEE32                  | 0.5..15 (typ 5)               | Error band disabling corrections / bleeding integrator |
| head_kp              | float | IEEE32                  | 0.5..8 (typ 3.5)              | Proportional gain deg -> command units                 |
| head_ki              | float | IEEE32                  | 0.0..0.5 (typ 0.05)           | Integral gain deg\*s -> command                        |
| head_kd              | float | IEEE32                  | 0.0..2.0 (typ 0.8)            | Derivative gain deg/s -> command                       |
| head_cmd_max         | float | IEEE32                  | 50..800 (typ 400)             | Absolute clamp on PID output (command units)           |
| speed_zero_thresh    | u16   | 0..65535                | 0..300 (typ 50)               | Avg                                                    | cmd | below => treat as stationary |
| speed_high_frac      | float | IEEE32                  | 0.2..1.0 (typ 0.80)           | Fraction of full output considered "high speed"        |
| spin_cmd_min         | u16   | 0..65535                | 50..400 (typ 180)             | Minimum spin command magnitude at rest                 |
| spin_cmd_max         | u16   | 0..65535                | 200..1000 (typ 700)           | Maximum spin command magnitude at rest                 |
| failsafe_stuck_thr   | u8    | 0..255                  | 0 or 1                        | Enable stuck-signal neutral hold heuristic             |
| stuck_delta_us       | u16   | 0..65535                | 1..10 (typ 1)                 | Max pulse width delta to count as unchanged            |
| stuck_cycles         | u16   | 0..65535                | 20..200 (typ 75)              | Consecutive stable cycles to trigger hold              |
| dead_center          | i16   | -32768..32767           | 0..200 (typ 50)               | Stick deadband in command units                        |
| stale_timeout_ms     | u16   | 0..65535                | 60..250 (typ 100)             | Channel stale threshold (ms)                           |
| air_gain_per_cycle   | i16   | -32768..32767           | 10..100 (typ 40)              | Air mode setpoint increment scale                      |

Example LIST output snippet:

```
reverse_left=0
reverse_right=0
motor_expo_l=400
motor_expo_r=400
heading_hold_en=1
... (remaining)
```

### CFG GET

```
CFG GET head_kp
CFG head_kp=3.5000
```

### CFG SET

```
CFG SET head_kp 4.2
OK head_kp=4.2000
```

Value is clamped to the stored type range; reply shows the final stored value.

### CFG RESET / CFG SAVE

- RESET: Restores all parameters to compiled defaults and persists (reply `OK`).
- SAVE: Forces EEPROM write of any pending change (usually unnecessary because SET auto-saves).

## Telemetry

All telemetry commands return a single line except `TELEM ALL` (single line aggregate too). Keys are fixed, order stable.

### TELEM STATUS

Format:

```
STATUS mode=<DIS|NRM|AIR> armed=<0|1> hold=<0|1> bno=<0|1> fsHold=<0|1> cmdL=<int> cmdR=<int> usL=<int> usR=<int>
```

### TELEM RC

```
RC yawUs=<us> thrUs=<us> nUs=<us> aUs=<us> hUs=<us> yaw=<-1000..1000> thr=<-1000..1000> valid=<0|1>
```

### TELEM MOTORS

```
MOTORS cmdL=<int> cmdR=<int> usL=<int> usR=<int>
```

### TELEM HEADING

```
HEADING bno=<0|1> cur=<deg?> hold=<0|1> [tgt=<deg> err=<deg>]
```

`cur`, `tgt`, `err` only when heading available (`bno=1` and read succeeds). Target/err only when hold active.

### TELEM ALL

Combines STATUS, RC, MOTORS, HEADING into one line:

```
ALL mode=<..> armed=.. hold=.. bno=.. fsHold=.. yawUs=.. thrUs=.. nUs=.. aUs=.. hUs=.. yaw=.. thr=.. cmdL=.. cmdR=.. usL=.. usR=.. [curH=.. [tgtH=.. errH=..]]
```

## Heading Commands

| Command        | Effect                                                                  |
| -------------- | ----------------------------------------------------------------------- |
| HEAD ON        | Enable hold if feature & IMU present; capture current heading as target |
| HEAD OFF       | Disable hold (target retained)                                          |
| HEAD SET <deg> | Normalize degree (wrap 0–360) and set target (resets integrator)        |
| HEAD TARGET    | Report current target                                                   |

Errors:

- `ERR: feature disabled` when `heading_hold_en=0`
- `ERR: no heading` when IMU read fails
- `ERR: bad number` when SET argument invalid

## RESET Command

`RESET` disarms, outputs neutral (1500 µs), clears heading-hold state: reply `OK`.

## Examples

```
> cfg set head_kp 4.0
OK head_kp=4.0000
> head on
HEAD ON target=42.15
> telem heading
HEADING bno=1 cur=42.15 hold=1 tgt=42.15 err=0.00
> head set 90
HEAD TARGET=90.00
> telem heading
HEADING bno=1 cur=42.30 hold=1 tgt=90.00 err=47.70
> telem all
ALL mode=AIR armed=1 hold=1 bno=1 fsHold=0 yawUs=1496 thrUs=1502 nUs=1000 aUs=1900 hUs=1010 yaw=0 thr=0 cmdL=0 cmdR=0 usL=1500 usR=1500 curH=42.3 tgtH=90.0 errH=47.7
```

## Implementation Notes

- Parser runs every loop; non-blocking buffered line input (64 bytes max).
- Backspace not currently implemented (simple append/terminate model); send a newline to reset buffer.
- `CFG SET` triggers immediate EEPROM write only when the value actually changes.
- Use modest polling (≤10 Hz) for `TELEM ALL` to conserve bandwidth.

## Future

Potential future command groups: `CAL` (calibration), `LOG`, `PROFILE` (parameter sets).

---

KC1 Serial API v1 (updated)
