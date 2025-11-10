# Compass Correction Setup Guide

## Overview

The `compass_correction_deg` parameter allows you to correct the heading reading based on how your BNO055 compass is physically mounted in your kayak. This ensures the heading display in your client matches the actual direction your kayak is pointing.

## When to Use

Use compass correction when:

- The BNO055 is not mounted with its X-axis pointing forward
- Your client displays the wrong heading compared to actual direction
- You want to define "forward" as a different angle than the sensor's natural 0°

## How It Works

The correction is added to the raw BNO055 reading:

```
Corrected Heading = (Raw BNO055 Heading + compass_correction_deg) mod 360
```

This correction is applied to:

- All heading telemetry (`TELEM HEADING`, `TELEM ALL`)
- Target heading when entering heading hold mode (via `HEAD ON` or CH5 toggle)
- Heading hold PID calculations

## Setup Procedure

### 1. Find Your Mounting Offset

1. Point your kayak exactly North (use a reference compass or GPS)
2. Connect to the serial console and run:
   ```
   TELEM HEADING
   ```
3. Note the `cur=` value (current heading)
4. Calculate offset:
   ```
   compass_correction_deg = 0 - current_heading
   ```
   Example: If current shows 90° when pointing North, correction = -90 (or 270)

### 2. Set the Correction

```
CFG SET compass_correction_deg <value>
```

Example:

```
CFG SET compass_correction_deg 270
```

The value is automatically saved to EEPROM and will persist across reboots.

### 3. Verify

Point North again and check:

```
TELEM HEADING
```

The `cur=` value should now read 0° (or very close to it).

## Valid Range

- **Min**: 0°
- **Max**: 359°
- **Step**: 1° (integer values only)

Negative values are automatically wrapped (e.g., -90 becomes 270).

## Examples

### Example 1: Sensor Rotated 90° Clockwise

If your sensor is rotated 90° clockwise (X-axis points right instead of forward):

```
CFG SET compass_correction_deg 270
```

Or equivalently:

```
CFG SET compass_correction_deg -90
```

### Example 2: Sensor Mounted Backwards

If your sensor is mounted backwards (180° rotation):

```
CFG SET compass_correction_deg 180
```

### Example 3: Sensor at 45° Angle

If your sensor is tilted 45° from forward:

```
CFG SET compass_correction_deg 315
```

Or:

```
CFG SET compass_correction_deg -45
```

## Effect on Heading Hold Mode

When you enter heading hold mode (via `HEAD ON` command or CH5 toggle), the system captures the **corrected** heading as the target. This means:

1. If you're pointing North (0°) and enter heading hold
2. The target is set to 0° (corrected)
3. The PID will maintain 0° (your desired forward direction)
4. Not the raw sensor reading

This ensures heading hold works correctly regardless of sensor mounting.

## Checking Current Setting

View current value:

```
CFG GET compass_correction_deg
```

View metadata (range, step, description):

```
CFG META compass_correction_deg
```

## Tips

1. **Use a reliable reference**: GPS heading or a good compass
2. **Test while stationary**: Magnetic interference from motors can affect readings
3. **Verify in telemetry**: Always check `TELEM HEADING` after setting
4. **Round numbers**: Usually your mount is at 0°, 90°, 180°, or 270°
5. **Fine tuning**: If slightly off, adjust in 1° increments

## Integration with Client App

When building a client app to display heading:

1. Request current heading via `TELEM HEADING` or `TELEM ALL`
2. The `cur=` or `curH=` value is already corrected
3. Display this value directly (no client-side math needed)
4. The app can allow users to adjust `compass_correction_deg` via `CFG SET`

## Troubleshooting

**Heading still wrong after setting correction:**

- Verify with `CFG GET compass_correction_deg` that it was saved
- Check if BNO055 is working: `TELEM HEADING` should show `bno=1`
- Ensure you're testing in stable magnetic environment (away from metal/electronics)
- Try `CFG SAVE` to force EEPROM write

**Correction seems backwards:**

- You may need to add/subtract 180° from your current value
- Or use the opposite sign (add 360 if negative)

**Want to reset to no correction:**

```
CFG SET compass_correction_deg 0
```
