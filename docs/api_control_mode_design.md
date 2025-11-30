# API Control Mode Design (EXT Mode)

## Overview

Add a new mode (`MODE_EXT`) where motor throttle values are set directly via API commands, emulating the behavior of AIR mode but with API input instead of RC.

## Core Concept

- **Simple API**: Single command sets both motor values
- **Transient values**: If no command received within timeout, motors go to 0
- **Emulates AIR mode**: Uses same internal motor conversion logic
- **No complex state machine**: Just receive values, apply them, timeout if silent

## Safety

**Automatic failsafe**: Motors stop if no `EXT` command received within timeout (5 seconds).

---

## API Commands

### Motor Control

| Command              | Response | Description                                |
| -------------------- | -------- | ------------------------------------------ |
| `EXT <left> <right>` | `OK`     | Set motor values, each -1.0 to 1.0 (float) |

Examples:

```
EXT 0.5 0.5      # Both motors forward 50%
EXT 1.0 1.0     # Full forward
EXT -0.5 -0.5   # Reverse 50%
EXT 0.3 -0.3    # Spin right (left forward, right reverse)
EXT 0 0         # Stop
```

### Mode Entry/Exit

| Command       | Response | Description                                         |
| ------------- | -------- | --------------------------------------------------- |
| `MODE EXT`    | `OK`     | Enter EXT mode (requires disarmed or will auto-arm) |
| `MODE DISARM` | `OK`     | Exit any mode, disarm                               |

Or simply: First `EXT` command auto-enters EXT mode, timeout auto-exits.

---

## Behavior

### On receiving `EXT <L> <R>`:

1. If not in EXT mode, switch to EXT mode
2. Store values as current motor targets
3. Reset watchdog timer
4. Apply values through existing motor output logic (expo, scaling, etc.)

### On watchdog timeout (no EXT command for 5 seconds):

1. Set both motors to 0
2. Remain in EXT mode waiting for next command
3. Use RESET command to disarm and exit

### On RC button press:

1. Exit EXT mode
2. Return to normal RC control

---

## Implementation

### New globals:

```cpp
static float g_extMotorL = 0.0f;  // -1.0 to 1.0
static float g_extMotorR = 0.0f;  // -1.0 to 1.0
static uint32_t g_lastExtCmdMs = 0;
```

### In command parser:

```cpp
if (icmp(cmd, "EXT") == 0) {
    float l, r;
    // parse two floats
    g_extMotorL = constrain(l, -1.0f, 1.0f);
    g_extMotorR = constrain(r, -1.0f, 1.0f);
    g_lastExtCmdMs = millis();
    if (g_mode != MODE_EXT) {
        g_mode = MODE_EXT;
        // arm if needed
    }
    reply("OK");
}
```

### In loop() MODE_EXT case:

```cpp
case MODE_EXT: {
    // Check watchdog
    if (millis() - g_lastExtCmdMs > 5000) {
        g_extMotorL = 0;
        g_extMotorR = 0;
    }
    // Convert -1..1 to command units (-1000..1000)
    int16_t cmdL = (int16_t)(g_extMotorL * 1000.0f);
    int16_t cmdR = (int16_t)(g_extMotorR * 1000.0f);
    // Use existing motor output path
    outputMotors(cmdL, cmdR);
    break;
}
```

---

## Configuration Parameters

| Parameter   | Type  | Default | Description                |
| ----------- | ----- | ------- | -------------------------- |
| (hardcoded) | const | 5000    | Timeout before motors stop |

Note: Watchdog timeout is currently a compile-time constant (`EXT_WATCHDOG_MS = 5000`).

---

## Example Session

```
> TELEM STATUS
STATUS mode=DIS armed=0 ...

> EXT 0.3 0.3
OK

> TELEM STATUS
STATUS mode=EXT armed=1 ...

> EXT 0.5 0.5
OK

> EXT 0.5 0.4
OK

# (phone stops sending for 5 seconds)
# Motors automatically go to 0

> EXT 0 0
OK

> MODE DISARM
OK
```

---

## Phone App Behavior

1. Connect via Bluetooth
2. Virtual joystick maps to L/R motor values
3. Send `EXT <L> <R>` every 500ms-1s while joystick active
4. On joystick release, send `EXT 0 0`
5. Watchdog (5s) handles connection loss automatically

---

## Implementation Checklist

- [x] Add MODE_EXT to mode enum
- [x] Add g_extMotorL, g_extMotorR, g_lastExtCmdMs globals
- [x] Implement EXT command parser
- [x] Add MODE_EXT case in loop()
- [x] Add watchdog timeout logic
- [x] Update TELEM STATUS to show EXT mode
- [x] Update documentation
- [ ] Add ext_watchdog_ms config parameter (currently hardcoded)
