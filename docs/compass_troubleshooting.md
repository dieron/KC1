# BNO055 Compass Troubleshooting Guide

## Symptoms You Experienced

- Compass sometimes shows correct direction, then stops updating
- Heading slowly drifts to incorrect values
- Readings are inconsistent or unreliable
- Compass "has a life of its own"

## Root Causes Fixed in v0.4.3.0

### 1. **Missing Operation Mode Configuration** ⚠️ CRITICAL

**Problem**: The sensor was never explicitly set to NDOF mode (9-axis sensor fusion with magnetometer).
**Impact**: Sensor might operate in wrong mode, causing unreliable magnetometer readings.
**Fix**: Now explicitly sets `OPERATION_MODE_NDOF` at startup.

### 2. **Uncalibrated Magnetometer** ⚠️ CRITICAL

**Problem**: No calibration checking or user guidance.
**Impact**: Magnetometer can be completely uncalibrated (cal=0), providing random/drifting headings.
**Fix**:

- Monitors magnetometer calibration status (0-3 scale)
- Requires cal≥2 for reliable heading hold
- Provides calibration guidance at startup
- Checks calibration every 5 seconds during operation

### 3. **No Stale Data Detection** ⚠️ MAJOR

**Problem**: Sensor can get "stuck" returning same value repeatedly.
**Impact**: Heading appears frozen while kayak is turning.
**Fix**: Detects when heading hasn't changed for >30 seconds and rejects the reading.

### 4. **Inadequate Data Validation** ⚠️ MAJOR

**Problem**: Only checked for NaN, not range or communication errors.
**Impact**: Invalid readings (negative, >360°, or I2C errors) were accepted.
**Fix**: Validates range (0-360°), checks for NaN, monitors consecutive failures.

### 5. **No Error Recovery** ⚠️ MODERATE

**Problem**: If sensor gets into bad state, it stays there.
**Impact**: After communication glitch, compass becomes permanently unreliable.
**Fix**: Tracks consecutive read failures, performs health check and recovery after 50 failures.

### 6. **Slow I2C Communication** ⚠️ MODERATE

**Problem**: Default 100kHz I2C clock.
**Impact**: Slower communication, more susceptible to electrical noise.
**Fix**: Now uses 400kHz I2C for faster, more reliable communication.

## Calibration Procedure

The BNO055 magnetometer **must be calibrated** for reliable heading. Calibration status ranges from 0 (uncalibrated) to 3 (fully calibrated).

### How to Calibrate

1. **Power on the system** in an open area away from metal structures
2. **Watch startup messages**:

   ```
   BNO055 OK
   BNO055 configured for NDOF mode
   Mag calib: 0
   WARNING: Magnetometer not calibrated!
   Move in figure-8 pattern to calibrate.
   ```

3. **Move the kayak in a figure-8 pattern** (horizontal plane):

   - Make slow, smooth figure-8 movements
   - Rotate the kayak through all orientations
   - Continue for 30-60 seconds
   - You can do this on land or water

4. **Check calibration status**:

   ```
   TELEM HEADING
   ```

   Look for `cal=2` or `cal=3`:

   ```
   HEADING bno=1 cal=2 fails=0 lastMs=12345 cur=45.32 hold=0
   ```

5. **Calibration is automatic** - the BNO055 stores calibration internally and may retain it across power cycles (not guaranteed).

### Calibration Status Meanings

- **cal=0**: Uncalibrated - readings are unreliable
- **cal=1**: Partially calibrated - still unreliable
- **cal=2**: Well calibrated - **minimum acceptable** for heading hold
- **cal=3**: Fully calibrated - best accuracy

## Diagnostic Commands

### Check Compass Health

```
TELEM HEADING
```

**Example output:**

```
HEADING bno=1 cal=2 fails=0 lastMs=45320 cur=87.45 hold=0
```

**What to check:**

- `bno=1`: Sensor detected (0 = not found)
- `cal=2`: Calibration status (need ≥2)
- `fails=0`: Consecutive read failures (0 = healthy)
- `lastMs=45320`: Timestamp of last successful read
- `cur=87.45`: Current corrected heading

### Monitor Continuous Operation

```
TELEM ALL
```

Shows complete system status including heading in compressed format.

## Common Issues & Solutions

### Issue: "Mag calib: 0" or "Mag calib: 1"

**Symptom**: Heading drifts or jumps randomly.

**Solution**:

1. Move kayak in figure-8 pattern for 30-60 seconds
2. Check `TELEM HEADING` until `cal=2` or `cal=3`
3. Repeat if calibration drops during operation

### Issue: Heading Stuck/Not Updating

**Symptom**: `cur=` value doesn't change when kayak turns.

**Indicators in TELEM HEADING:**

- `fails=` increasing
- `lastMs=` not updating

**Solutions**:

1. **Check wiring**: Loose I2C connections
2. **Check for interference**: Motors, ESCs, high-current wires too close to BNO055
3. **Verify sensor alive**: Look for `fails>0` and increasing
4. **Wait for auto-recovery**: System recovers after 50 consecutive failures
5. **Power cycle**: If stuck, restart controller

### Issue: Heading Slowly Drifts

**Symptom**: Heading changes by 1-2° per minute without kayak moving.

**Possible causes:**

1. **Low calibration** (`cal<2`): Recalibrate
2. **Magnetic interference**: Move sensor away from motors/ESCs/battery
3. **Temperature changes**: BNO055 compensates automatically, but large changes take time
4. **Metal nearby**: Steel hull, rebar in concrete, underground pipes

**Solutions**:

1. Ensure `cal=2` or higher
2. Mount sensor at least 15cm from motors and high-current wires
3. Use twisted-pair or shielded cable for I2C (SDA/SCL)
4. Keep sensor away from ferromagnetic materials

### Issue: Heading Jumps Suddenly

**Symptom**: Heading changes by 10-90° instantly without movement.

**Possible causes:**

1. **Magnetic disturbance**: Passing near metal object, electric field
2. **Lost calibration**: `cal` dropped to 0 or 1
3. **I2C communication error**: Electrical noise, bad connection

**What to check:**

```
TELEM HEADING
```

- If `cal` dropped: Recalibrate
- If `fails>0`: Check wiring and interference
- If `lastMs` old: Sensor communication issue

### Issue: "BNO055 NOT FOUND"

**Symptom**: Startup shows sensor not detected.

**Solutions**:

1. **Check I2C address**: BNO055 can be 0x28 or 0x29 (code tries both)
2. **Verify wiring**:
   - SDA to A4 (or dedicated SDA pin)
   - SCL to A5 (or dedicated SCL pin)
   - VCC to 3.3V or 5V (depends on module)
   - GND to GND
3. **Check power**: BNO055 requires stable power, 3.3V or 5V depending on module
4. **Pull-up resistors**: I2C needs 4.7kΩ pull-ups on SDA/SCL (often on module)
5. **Cable length**: Keep I2C wiring under 30cm if possible

## Electromagnetic Interference Mitigation

Your kayak environment has significant EMI sources:

- **Brushless motors**: Generate RF noise
- **ESCs**: High-frequency PWM switching
- **Battery cables**: High current pulses
- **Servo signals**: PWM signals can couple into I2C

### Best Practices

1. **Physical Separation**:

   - Mount BNO055 at least 15cm from motors
   - Keep sensor away from ESC and motor power wires
   - Separate I2C wires from servo/PWM signal wires

2. **Wiring**:

   - Use twisted pair or shielded cable for I2C (twist SDA+SCL together)
   - Keep I2C wires short (<30cm)
   - Add small ferrite bead on I2C cable if noise persists
   - Route I2C cable away from power wires

3. **Power**:

   - Filter BNO055 power with 10µF + 0.1µF capacitors close to sensor
   - Use separate 3.3V regulator if possible (not motor/ESC supply)

4. **Grounding**:

   - Single ground point (star ground)
   - Avoid ground loops

5. **Shielding** (advanced):
   - Mount BNO055 in small aluminum enclosure
   - Connect shield to ground at ONE point only

## Field Testing Checklist

Before heading out on water:

- [ ] Check `TELEM HEADING` shows `bno=1`
- [ ] Verify `cal=2` or `cal=3` after calibration
- [ ] Confirm heading updates in real-time when rotating kayak
- [ ] Check `fails=0` and `lastMs` incrementing
- [ ] Test heading hold mode activates (`HEAD ON`)
- [ ] Verify PID responds correctly to heading error

## Understanding the New Telemetry

**Before (v0.4.2.0):**

```
HEADING bno=1 cur=87.45 hold=0
```

**After (v0.4.3.0):**

```
HEADING bno=1 cal=2 fails=0 lastMs=45320 cur=87.45 hold=0 tgt=90.00 err=2.55
```

**New fields:**

- `cal=2`: Magnetometer calibration status (0-3)
- `fails=0`: Consecutive read failure count
- `lastMs=45320`: Milliseconds since boot of last successful read

**These help diagnose:**

- Calibration issues (low `cal`)
- Communication problems (high `fails`, stale `lastMs`)
- Sensor freezes (unchanging `cur` with old `lastMs`)

## When to Suspect Hardware Issues

If after applying all fixes you still see:

- `cal` never reaches 2, even after extensive figure-8 movement
- `fails` constantly high (>10)
- `bno=0` or frequent detection loss
- Erratic behavior only near motors (even with good separation)

**Possible hardware problems:**

1. **Faulty BNO055 module**: Manufacturing defect or damaged
2. **Bad I2C connection**: Intermittent solder joint, broken wire
3. **Incompatible I2C voltage levels**: 5V Arduino with 3.3V-only BNO055
4. **Counterfeit BNO055**: Some cheap modules are not genuine Bosch sensors

**Test**: Try the BNO055 module on a simple breadboard setup away from motors. If it works there but fails in kayak, it's an EMI/wiring issue. If it fails everywhere, hardware is suspect.

## Summary of Improvements

| Issue                | Before                | After                            |
| -------------------- | --------------------- | -------------------------------- |
| Operation mode       | Never set (undefined) | NDOF mode (9-axis fusion)        |
| Calibration check    | None                  | Every 5 seconds + startup        |
| Stale data detection | None                  | 30-second timeout                |
| Range validation     | Only NaN check        | Full 0-360° + NaN                |
| Error recovery       | None                  | Auto-recovery after 50 failures  |
| I2C speed            | 100kHz (default)      | 400kHz (fast mode)               |
| Health monitoring    | None                  | Temperature check                |
| Diagnostic data      | Minimal               | cal/fails/timestamp              |
| User guidance        | None                  | Startup calibration instructions |

These improvements should eliminate the "compass living its own life" behavior you experienced.
