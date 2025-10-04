# KC1 Enhancement TODO (Key Gaps Tracking)

Legend:

- Status: NOT (not implemented yet), PART (partially implemented), DONE (fully implemented)
- "Implemented On" can be filled with a date/commit later.

## Control & Command Input

1. Commanded Control Input (serial/app virtual sticks)  
   Status: NOT  
   Description: Add command (e.g. `CTRL SET yaw=<int> thr=<int>`) to inject control setpoints (-1000..1000) when source=APP.
2. Arming / Safety Layer Enhancements  
   Status: NOT  
   Description: Commands ARM, DISARM, two-step arming (ARM REQ/CONFIRM), EMSTOP immediate neutral.
3. Link-Loss Failsafe (App Heartbeat)  
   Status: NOT  
   Description: Heartbeat command or implicit ping; timeout triggers neutral + disarm or fallback to RC.
4. Control Source Multiplexing  
   Status: NOT  
   Description: Manage priority & switching RC vs APP (`CTRL SOURCE RC|APP`), automatic fallback to RC if pulses valid and app heartbeat lost.
5. Input Rate Limiting & Smoothing (Injected Setpoints)  
   Status: NOT  
   Description: Slew/accel limits for virtual stick changes to avoid abrupt thrust/yaw jumps.
6. Simulation/Test Mode  
   Status: NOT  
   Description: SIM mode to inject synthetic RC frames for bench/mobile app testing without radio.

## Telemetry & Data Exposure

7. Streaming Telemetry Mode  
   Status: NOT  
   Description: `TELEM STREAM START <Hz>` / `TELEM STREAM STOP` push frames (5–10 Hz) without repeated polling.
8. Timestamp Inclusion  
   Status: NOT  
   Description: Add `ms=<uint32>` (millis) to telemetry (STATUS/ALL) for client-side latency & rate calculations.
9. Calibration & IMU Status Exposure  
   Status: NOT  
   Description: Expose BNO055 calibration/system status (sys,gyr,acc,mag) via `TELEM CAL` or fields in STATUS.
10. Health & Diagnostics Telemetry  
    Status: NOT  
    Description: Loop time avg/max, RC frame drop count, failsafe reason, EEPROM dirty flag.
11. Battery / Power Telemetry  
    Status: NOT  
    Description: Add analog voltage (and current if sensor) readings: `Vbat`, `Ibat`.
12. Normalization/Scale Packet  
    Status: NOT  
    Description: `TELEM SCALE` describing ranges: command domain, PWM limits, heading units; aids dynamic clients.
13. Unified Failsafe State Reporting  
    Status: NOT  
    Description: Single FAILSAFE code (NONE|RC_STALE|STUCK|LINK_LOSS|BROWNOUT) included in STATUS / ALL.
14. Event Log / Ring Buffer  
    Status: NOT  
    Description: In-RAM log of recent events (disarm causes, mode changes) `LOG SHOW` / `LOG CLEAR`.
15. Version & Capability Discovery  
    Status: PART (2025-10-02)  
    Description: `VERSION` (build id / git hash) implemented; `FEATURES` (CSV of CFG,TELEM,HEAD,STREAM,...) pending.
16. Parameter Metadata Query  
    Status: DONE (2025-10-03)  
    Description: Implemented. `CFG META <name>` / `CFG META ALL` print type, min, max, step, default, units, desc. Added `CFG META JSON` exporting an array of objects for client UIs.
17. Autosave Toggle & Batch Config  
    Status: NOT  
    Description: `CFG AUTOSAVE ON|OFF`; `CFG BATCH BEGIN/SET/COMMIT/ABORT` for atomic multi-param updates.
18. Extended Heading Telemetry Enhancements  
    Status: NOT  
    Description: Add PID terms (P,I,D,output) optionally for tuning frames.

## Protocol & Robustness

19. Protocol Framing & CRC  
    Status: NOT  
    Description: Optional framed packets with length, type, CRC16, sequence ID; ACK/NACK for critical commands.
20. Binary Mode Option  
    Status: NOT  
    Description: Compact binary telemetry/control channel for higher rates (>10 Hz) & reduced parsing overhead.
21. Sequence Numbers / Replay Protection  
    Status: NOT  
    Description: Incrementing sequence in state-changing commands; ignore duplicates.
22. Reserved Prefix Enforcement  
    Status: NOT  
    Description: Reserve CAL, LOG, PROFILE, etc.; reject misuse with specific `ERR: reserved`.
23. Noise Immunity Command Prefix  
    Status: NOT  
    Description: Require '!' prefix for state-changing commands in optional strict mode to mitigate stray bytes.
24. Backspace & Line Editing / Echo Control  
    Status: NOT  
    Description: Support backspace editing & `ECHO ON|OFF` for cleaner machine operation (no prompts when off / streaming).
25. Authentication / Pairing (Future Wireless)  
    Status: NOT  
    Description: Simple token or challenge-response gating ARM & control commands.

## Safety & Reliability

26. Two-Level Arming Interlock  
    Status: NOT  
    Description: Requires sequential ARM REQ / ARM CONFIRM with timeout window.
27. Emergency Stop (EMSTOP)  
    Status: NOT  
    Description: Immediate neutral + disarm regardless of state; logs event.
28. Watchdog Integration & Reset Reason  
    Status: NOT  
    Description: Enable AVR WDT, store last reset cause, expose via STATUS / LOG.
29. Brownout / Voltage Failsafe  
    Status: NOT  
    Description: If Vbat below threshold, disarm or limit outputs; advertise warning flag.
30. Unified Failsafe State Machine Refactor  
    Status: NOT  
    Description: Consolidate stale RC, stuck, link-loss into one coherent state machine & transitions.

## Build / Identity / Maintenance

31. Firmware Build ID / Git Hash  
    Status: DONE (2025-10-02)  
    Description: Implemented. Git short hash injected at build via extra script; `VERSION` prints `api`, `git`, and `build` date/time.
32. Baud Rate / Throughput Strategy  
    Status: NOT  
    Description: Option to switch to 230400 (or configurable) & guidance for app selection.
33. Binary Logging Hooks (Future)  
    Status: NOT  
    Description: Internal buffer and optional dump for high-rate tuning (not a priority now).

## Misc & Enhancements

34. PID Live Tuning Aids  
    Status: NOT  
    Description: On-demand streaming of PID intermediate terms for heading tuning.
35. Structured Error Codes  
    Status: NOT  
    Description: Enumerated numeric error codes alongside textual reason (machine friendly).
36. Rate Limiting for Command Flood  
    Status: NOT  
    Description: Throttle high-frequency non-stream commands (e.g. >20 Hz) to protect loop time.
37. Config Validation Layer  
    Status: NOT  
    Description: Reject obviously unsafe combos (e.g. spin_cmd_min > spin_cmd_max) with clear errors.
38. Safe Defaults Snapshot / Profile Sets  
    Status: NOT  
    Description: Multiple stored config profiles (PROFILE SAVE/LOAD <id>).
39. Documentation Auto-Export  
    Status: NOT  
    Description: Command to emit machine-readable JSON of parameters & features for app onboarding.
40. Telemetry Compression (Optional)  
    Status: NOT  
    Description: Simple delta or key omission when unchanged in streaming mode.

41. Per-Motor Trim / Symmetric Start Threshold & Scaling  
    Status: DONE (2025-10-02)  
    Description: IMPLEMENTED. Parameters added: `motor_start_us_l`, `motor_start_us_r`, `motor_scale_l`, `motor_scale_r`, `motor_start_region` with EEPROM persistence and README calibration guidance. Applied after expo + reversal, before PWM mapping. Telemetry values (RC, setpoints) remain unmodified; only final motor commands reflect correction. Initial completion bumped version to 0.3.0; project later progressed to 0.4.0.0 under the 4-part scheme.

42. ConfigStore Parameter Schema & Constraints  
    Status: DONE (2025-10-03)  
    Description: IMPLEMENTED. Per-parameter schema (min, max, step, units, description, default) added. `CFG SET` now clamps to bounds and snaps to step. API exposes schema via `CFG META <name>`, `CFG META ALL`, and `CFG META JSON` (array of objects) for app settings UIs. (Related: #16 DONE; #37 cross-parameter validation pending.)

---

## Implementation Order Suggestion

1. (1) Control Input + (4) Source Multiplex + (3) Heartbeat
2. (7) Streaming Telemetry + (8) Timestamps
3. (15) Version/Features + (16) Parameter Metadata
4. (17) Autosave Toggle/Batch
5. (26/27) Advanced Safety (Two-level arm, EMSTOP)
6. (13/30) Unified Failsafe & Code Reporting
7. (11) Battery Telemetry
8. (9) Calibration Exposure
9. (19/20) Protocol Hardening (framing/binary) if needed

Update this file whenever an item changes state. Commit separately for clear history.
