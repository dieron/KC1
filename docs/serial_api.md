# KC1 Serial Configuration & Telemetry API

Baud: 115200 8N1
Line endings: Send `\n` (CR optional, ignored). Controller replies with lines ending in `\r\n`.
Prompt: A literal `>` is emitted after each processed command (after newline). Partial lines are ignored until newline.
Backspace: Supported (DEL / BS).

All commands are case-insensitive. Tokens are space separated. Unknown commands return:

```
ERR: UNKNOWN
```

Successful state-changing commands return:

```
OK
```

Error forms use `ERR:` prefix with a short reason token (e.g. `ERR: NAME`).

## Summary of Commands

```
HELP
CFG LIST
CFG GET <name>
CFG SET <name> <value>
CFG RESET        (reset all parameters to compiled defaults and persist)
CFG SAVE         (force EEPROM save)
TELEM STATUS
TELEM RC
TELEM MOTORS
TELEM HEADING
TELEM ALL        (STATUS+RC+MOTORS+HEADING)
HEAD ON          (enable heading hold, captures current heading)
HEAD OFF         (disable heading hold)
HEAD SET <deg>   (set target heading without changing active state)
HEAD TARGET      (report current target heading)
RESET            (alias: same as CFG RESET)
SAVE             (alias: same as CFG SAVE)
```

## Configuration Parameters

Names map 1:1 with `ConfigStore`. Use `CFG LIST` to view (values printed in deterministic order). Example snippet:

```
reverseLeft=0
reverseRight=0
motorExpoL=35
motorExpoR=35
headingHoldEnabled=1
headingDeadbandDeg=5.00
headKp=3.500000
headKi=0.050000
headKd=0.800000
headCmdMax=600.000000
speedZeroThresh=50
speedHighFrac=0.800000
spinCmdMin=220
spinCmdMax=700
failsafeStuckThr=1
stuckDeltaUs=1
stuckCycles=75
deadCenter=50
staleTimeoutMs=100
airGainPerCycle=40
```

### GET

`CFG GET headKp` -> `headKp=3.500000`

### SET

`CFG SET headKp 4.2` -> `OK`
Immediate effect: subsequent control loop iteration uses new value.
Invalid name -> `ERR: FAIL` or `ERR: NAME`

### RESET / SAVE

`CFG RESET` resets RAM to defaults (compile-time fallback macros in `ConfigStore.cpp`), writes EEPROM, replies `OK`.
`CFG SAVE` forces EEPROM write (values already live) and replies `OK`.

## Telemetry

Values are single-line, key=value pairs separated by spaces.

### STATUS

Fields:

- mode: numeric (0 DISARMED,1 NORMAL,2 AIR)
- armed: 0/1
- hold: heading hold active flag
- bno: IMU detected (1) or not (0)
- fs_hold: stuck-signal failsafe (neutral hold) active flag

Example:
`mode=2 armed=1 hold=1 bno=1 fs_hold=0`

### RC

Raw and scaled RC inputs at time of request (fresh read):
`yaw_us thr_us nrm_us air_us hold_us yaw thr valid`
Example:
`yaw_us=1500 thr_us=1500 nrm_us=1100 air_us=1900 hold_us=1020 yaw=0 thr=0 valid=1`

### MOTORS

Last commanded signed values and microsecond outputs:
`cmdL=<signed> cmdR=<signed> usL=<microseconds> usR=<microseconds>`
Example: `cmdL=320 cmdR=280 usL=1660 usR=1640`

### HEADING

`hold=<0|1> bno=<0|1> cur=<deg> tgt=<deg> err=<deg>` (cur/tgt/err only if IMU & heading readable)
Example: `hold=1 bno=1 cur=123.4 tgt=125.0 err= -1.60`

### ALL

Concatenates STATUS, RC, MOTORS, HEADING in that order (four lines).

## Heading Control Commands

- `HEAD ON`: Enables heading hold (if IMU present) and sets target to current heading.
- `HEAD OFF`: Disables heading hold (target retained internally).
- `HEAD SET <deg>`: Sets target to given angle (normalized 0..360). If hold is active, new target used immediately.
- `HEAD TARGET`: Returns `target=<deg>`.

Error tokens:

- `NOIMU`: IMU not detected
- `NOHDG`: Heading read failed
- `VAL`: Missing or invalid value
- `SUBCMD`: Unknown subcommand

## Examples

```
>cfg list
(reverseLeft=0 ...)
>cfg set headKp 4.0
OK
>head on
OK
>telem heading
hold=1 bno=1 cur=42.1 tgt=42.1 err=0.00
>cfg set headingDeadbandDeg 3
OK
>head set 90
OK
>telem heading
hold=1 bno=1 cur=42.3 tgt=90.0 err=47.70
```

## Implementation Notes

- Parsing is non-blocking; processed once per control loop (~50 Hz). Latency negligible for configuration.
- `CFG SET` auto-persists immediately (ConfigStore::set triggers save on change). `CFG SAVE` rarely needed.
- Telemetry values are snapshots; for streaming, poll at desired rate externally.
- Avoid flooding with `TELEM ALL` faster than ~10 Hz to keep Serial bandwidth low.

## Future Extensions

Reserved command prefixes for future mobile app integration: `CAL` (calibration), `PROFILE`, `LOG`.

---

KC1 Serial API v1
