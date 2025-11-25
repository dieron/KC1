# KC1 Serial Configuration & Telemetry API (v2)

Baud: 9600 8N1
Line endings: LF or CRLF (both terminate). Prompt: `>` after each processed line.
Commands: Case-insensitive. Tokens separated by one or more spaces or tabs.

Responses:

- Success (state change or info): Specific line or `OK`
- Error: `ERR: <reason>` (generic reasons: `unknown`, `missing`, `bad number`, `feature disabled`)

## Command Summary

```
HELP
VERSION
CFG LIST
CFG GET <name>
CFG META <name|ALL|JSON>
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

## VERSION Command

Returns firmware version, optional git hash, and build date/time:

```
VERSION api=0.4.0.0 git=6c61ac7 build=Oct  3 2025T12:34:56
```

## Configuration Parameters

All parameters now have schema metadata including min/max bounds, step size, units, and descriptions. `CFG SET` automatically clamps values to valid ranges and quantizes to step increments.

### Parameter Schema Discovery

- `CFG META <name>` - Get metadata for one parameter
- `CFG META ALL` - Get metadata for all parameters (text format)
- `CFG META JSON` - Get metadata as JSON array for client UIs

Example metadata output:

```
META head_kp type=F32 def=3.5000 min=0.0000 max=10.0000 step=0.0500 units= desc="Heading PID gain"
```

JSON format returns an array of objects with fields: `name`, `type`, `min`, `max`, `step`, `default`, `units`, `desc`.

### Complete Parameter List

Names correspond exactly to internal storage fields (snake_case). All parameters now enforce schema constraints.

| Name                    | Type | Min  | Max    | Step | Default | Description                                            |
| ----------------------- | ---- | ---- | ------ | ---- | ------- | ------------------------------------------------------ |
| reverse_left            | U8   | 0    | 1      | 1    | 0       | Invert left motor output sign                          |
| reverse_right           | U8   | 0    | 1      | 1    | 0       | Invert right motor output sign                         |
| motor_expo_l            | U16  | 0    | 1000   | 1    | 400     | Expo shaping left motor (0 linear)                     |
| motor_expo_r            | U16  | 0    | 1000   | 1    | 400     | Expo shaping right motor                               |
| heading_hold_en         | U8   | 0    | 1      | 1    | 1       | Master enable for heading hold feature                 |
| compass_correction_deg  | I16  | 0    | 359    | 1    | 0       | Compass mounting correction in degrees                 |
| heading_deadband_deg    | F32  | 0.0  | 30.0   | 0.1  | 5.0     | Error band disabling corrections / bleeding integrator |
| head_kp                 | F32  | 0.0  | 10.0   | 0.05 | 3.5     | Proportional gain deg -> command units                 |
| head_ki                 | F32  | 0.0  | 2.0    | 0.01 | 0.05    | Integral gain deg\*s -> command                        |
| head_kd                 | F32  | 0.0  | 5.0    | 0.01 | 0.8     | Derivative gain deg/s -> command                       |
| head_cmd_max            | F32  | 50.0 | 1000.0 | 5.0  | 400.0   | Absolute clamp on PID output (command units)           |
| speed_zero_thresh       | U16  | 0    | 200    | 1    | 50      | Avg cmd below => treat as stationary                   |
| speed_high_frac         | F32  | 0.0  | 1.0    | 0.01 | 0.80    | Fraction of full output considered "high speed"        |
| spin_cmd_min            | U16  | 0    | 1000   | 1    | 180     | Minimum spin command magnitude at rest                 |
| spin_cmd_max            | U16  | 0    | 1000   | 1    | 700     | Maximum spin command magnitude at rest                 |
| failsafe_stuck_thr      | U8   | 0    | 1      | 1    | 1       | Enable stuck-signal neutral hold heuristic             |
| stuck_delta_us          | U16  | 0    | 50     | 1    | 1       | Max pulse width delta to count as unchanged            |
| stuck_cycles            | U16  | 1    | 1000   | 1    | 75      | Consecutive stable cycles to trigger hold              |
| dead_center             | I16  | 0    | 500    | 1    | 50      | Stick deadband in command units                        |
| stale_timeout_ms        | U16  | 20   | 2000   | 10   | 100     | Channel stale threshold (ms)                           |
| air_gain_per_cycle      | I16  | 0    | 200    | 1    | 40      | Air mode setpoint increment scale                      |
| hdg_gain_per_cycle      | I16  | 0    | 200    | 1    | 15      | Heading mode forward speed increment scale             |
| head_boost_trigger_mult | F32  | 1.0  | 5.0    | 0.1  | 2.0     | Boost trigger x deadband                               |
| head_max_boost          | F32  | 1.0  | 2.0    | 0.01 | 1.35    | Max boost multiplier                                   |
| motor_start_us_l        | U16  | 0    | 200    | 1    | 0       | Motor start offset left (µs equiv)                     |
| motor_start_us_r        | U16  | 0    | 200    | 1    | 0       | Motor start offset right (µs equiv)                    |
| motor_scale_l           | F32  | 0.5  | 1.5    | 0.01 | 1.0     | Motor scale factor left                                |
| motor_scale_r           | F32  | 0.5  | 1.5    | 0.01 | 1.0     | Motor scale factor right                               |
| motor_start_region      | U16  | 0    | 400    | 1    | 150     | Start region threshold                                 |

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

Value is automatically clamped to schema min/max bounds and quantized to step increments. Reply shows the final stored value after validation.

### CFG RESET / CFG SAVE

- RESET: Restores all parameters to compiled defaults and persists (reply `OK`).
- SAVE: Forces EEPROM write of any pending change (usually unnecessary because SET auto-saves).

## Telemetry

All telemetry commands return a single line except `TELEM ALL` (single line aggregate too). Keys are fixed, order stable.

### TELEM STATUS

Format:

```
STATUS mode=<DIS|NRM|AIR|HDG> armed=<0|1> bno=<0|1> fsHold=<0|1> cmdL=<int> cmdR=<int> usL=<int> usR=<int>
```

Mode values: DIS (disarmed), NRM (normal), AIR (air mode), HDG (heading mode).

**Air Mode (AIR) Controls:**

- **Cruise mode** (default): Joystick incrementally adjusts latched speed/yaw setpoints
- **Click Air button**: Toggles manual override—joystick acts as **offset** from cruise (centered=maintain, deflection=temporary adjustment)
- Click again to resume cruise at the original setpoints

**Heading Mode (HDG) Controls:**

- **Throttle**: Incrementally adjusts forward cruise speed (center=hold, forward=accelerate, back=decelerate)
- **Yaw <80%**: Ignored—PID handles all yaw corrections automatically
- **Yaw >80%**: Manually adjusts target heading by 3° per trigger (immediate + every 1s while held). Right=clockwise, Left=counter-clockwise.

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
HEADING bno=<0|1> cal=<0-3> fails=<uint> lastMs=<uint> [cur=<deg>] hold=<0|1> [tgt=<deg> err=<deg>]
```

- `bno`: 1 if BNO055 detected, 0 if not found
- `cal`: Magnetometer calibration status (0=uncalibrated, 1=poor, 2=good, 3=excellent). Requires ≥2 for reliable heading hold.
- `fails`: Consecutive read failure count (0=healthy, >0 indicates communication issues)
- `lastMs`: Milliseconds since boot of last successful heading read
- `cur`: Current corrected heading in degrees (0-360), only when valid reading available
- `hold`: 1 if heading hold active, 0 if inactive
- `tgt`, `err`: Target heading and error in degrees, only when hold active

### TELEM ALL

Combines STATUS, RC, MOTORS, HEADING into one line:

```
ALL mode=<DIS|NRM|AIR|HDG> armed=<0|1> hold=<0|1> bno=<0|1> fsHold=<0|1> yawUs=<us> thrUs=<us> nUs=<us> aUs=<us> hUs=<us> yaw=<-1000..1000> thr=<-1000..1000> cmdL=<int> cmdR=<int> usL=<int> usR=<int> [curH=<deg> [tgtH=<deg> errH=<deg>]]
```

**Note**: The compact `ALL` format does not include the new `cal`, `fails`, or `lastMs` fields. Use `TELEM HEADING` for full diagnostic information.

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
> version
VERSION api=0.4.0.0 git=6c61ac7 build=Oct  3 2025T12:34:56
> cfg meta head_kp
META head_kp type=F32 def=3.5000 min=0.0000 max=10.0000 step=0.0500 units= desc="Heading PID gain"
> cfg set head_kp 4.0
OK head_kp=4.0000
> cfg meta json
[{"name":"reverse_left","type":"U8","min":0.0000,"max":1.0000,"step":1.0000,"default":0.0000,"units":"bool","desc":"Invert motor direction"},...]
> head on
HEAD MODE ON tgt=42.15
> telem heading
HEADING bno=1 cal=2 fails=0 lastMs=45320 cur=42.15 hold=1 tgt=42.15 err=0.00
> head set 90
HEAD TARGET=90.00
> telem heading
HEADING bno=1 cal=2 fails=0 lastMs=47850 cur=42.30 hold=1 tgt=90.00 err=47.70
> telem all
ALL mode=HDG armed=1 hold=1 bno=1 fsHold=0 yawUs=1496 thrUs=1502 nUs=1000 aUs=1900 hUs=1010 yaw=0 thr=0 cmdL=0 cmdR=0 usL=1500 usR=1500 curH=42.3 tgtH=90.0 errH=47.7
```

## Implementation Notes

- Parser runs every loop; non-blocking buffered line input (64 bytes max).
- Backspace not currently implemented (simple append/terminate model); send a newline to reset buffer.
- `CFG SET` triggers immediate EEPROM write only when the value actually changes.
- Use modest polling (≤10 Hz) for `TELEM ALL` to conserve bandwidth.

## Future

Potential future command groups: `CAL` (calibration), `LOG`, `PROFILE` (parameter sets).

---

KC1 Serial API v2 - Updated for firmware 0.4.0.0 with parameter schema validation and JSON export
