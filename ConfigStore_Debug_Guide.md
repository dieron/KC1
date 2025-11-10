# ConfigStore::set Troubleshooting Guide

## Overview

The `ConfigStore::set` function can fail for several reasons. I've added debugging tools to help identify the issue.

## New Test Commands Added

### 1. `TEST CONFIG`

Runs a basic ConfigStore functionality test:

- Checks parameter count
- Tests get/set cycle on `dead_center` parameter
- Shows if the value actually changes
- Automatically restores original value

### 2. `TEST SETDEBUG <name> <val>`

Provides detailed debugging for a specific set operation:

- Verifies parameter exists
- Shows current value
- Displays parameter metadata (min/max/step constraints)
- Shows parsed input value
- Reports set() return value
- Shows final stored value
- Warns if value was clamped/adjusted

## Common Failure Scenarios

### 1. Parameter Name Mismatch

**Problem**: Parameter name doesn't exist
**Test**: `CFG GET badname` returns error
**Solution**: Use `CFG LIST` to see all valid parameter names

### 2. Value Outside Constraints

**Problem**: Value exceeds min/max bounds or violates step quantization
**Test**: `TEST SETDEBUG <name> <badval>` shows clamping
**Example**: Setting `dead_center` to 2000 when max is 200

### 3. Type Conversion Issues

**Problem**: Float to integer conversion or precision loss
**Test**: `TEST SETDEBUG` shows "Value was clamped/adjusted"
**Example**: Setting integer parameter to 1.7 becomes 2

### 4. EEPROM Write Failure

**Problem**: Hardware EEPROM issue (rare on Arduino Uno)
**Test**: `CFG SAVE` returns error
**Solution**: Check hardware or reset with `CFG RESET`

### 5. Metadata Constraints

**Problem**: Parameter has step quantization that modifies input
**Test**: `CFG META <name>` shows step value > 0
**Example**: Parameter with step=10 rounds 156 to 160

## Testing Procedure

1. **Basic Functionality**:

   ```
   TEST CONFIG
   ```

   If this fails, ConfigStore has fundamental issues.

2. **Specific Parameter Debug**:

   ```
   TEST SETDEBUG dead_center 75
   TEST SETDEBUG motor_expo_l 500
   TEST SETDEBUG head_kp 2.5
   ```

3. **Check Parameter Constraints**:

   ```
   CFG META dead_center
   CFG META ALL
   ```

4. **Verify Parameter List**:

   ```
   CFG LIST
   ```

5. **Test Boundary Values**:
   ```
   CFG META <param>  # Check min/max
   TEST SETDEBUG <param> <min_value>
   TEST SETDEBUG <param> <max_value>
   TEST SETDEBUG <param> <beyond_max>  # Should clamp
   ```

## Expected Behavior

- `ConfigStore::set()` returns `true` if value changed
- `ConfigStore::set()` returns `false` if value unchanged (same as current)
- Values are automatically clamped to [min,max] bounds
- Values are quantized to nearest step if step > 0
- EEPROM is automatically saved after successful set

## Debugging Tips

1. **Parameter Names**: Use exact names from `CFG LIST` (case-sensitive)
2. **Value Ranges**: Check `CFG META <name>` for valid ranges
3. **Step Quantization**: If step > 0, input will be rounded to nearest step
4. **Return Value**: `set()` returns false if no change occurred (not always an error)

## Example Debug Session

```
> TEST SETDEBUG dead_center 75
=== DEBUG SET dead_center 75 ===
Current: 50.0000
Metadata:
META dead_center min=0.00 max=200.00 step=1.00 units="" desc="RC stick deadband (+/- command units)"
Parsed value: 75.0000
Set returned: true
Final: 75.0000
=== Debug Complete ===
```

This shows a successful set operation with step quantization to nearest integer.
