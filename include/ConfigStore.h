// ConfigStore.h - Persistent configuration storage for KC1 (Arduino Uno)
// Provides runtime get/set/list/reset of tunable parameters using EEPROM.
// First-run initializes defaults (taken from existing compile-time #defines).

#pragma once

#include <Arduino.h>

class ConfigStore
{
public:
	// Public type info for metadata
	enum ParamType : uint8_t
	{
		PT_U8,
		PT_U16,
		PT_I16,
		PT_FLOAT
	};

	// Metadata describing constraints and docs
	struct Meta
	{
		const __FlashStringHelper *name; // matches Entry name
		ParamType type;
		float minVal; // numeric bounds (for booleans use 0..1 with step=1)
		float maxVal;
		float step;						  // UI hint; when >0 we quantize to nearest step in set()
		const __FlashStringHelper *units; // short label or empty
		const __FlashStringHelper *desc;  // brief description
	};
	// Call once in setup(). Loads from EEPROM or writes defaults if signature missing.
	static void begin();
	// Save current configuration explicitly (normally auto after set()).
	static bool save();
	// Restore defaults (in RAM) and write to EEPROM.
	static void resetDefaults();
	// Number of configurable parameters.
	static size_t count();
	// Name of parameter at index (0..count-1). Returns nullptr if out of range.
	static const __FlashStringHelper *nameAt(size_t idx);
	// Generic get by name -> value as float (integers/bools are promoted).
	static bool get(const char *name, float &outVal);
	// Set value by name (floats converted / clamped for integer fields). Auto-saves on success.
	static bool set(const char *name, float newVal);
	// Print all parameters and values to a Stream (e.g. Serial).
	static void list(Stream &s);

	// ---------- Parameter Metadata (Schema) ----------
	// Print metadata for a single parameter. Returns false if not found.
	static bool metaOne(Stream &s, const char *name);
	// Print metadata for all parameters.
	static void metaAll(Stream &s);
	// Print metadata for all parameters in JSON format.
	static void metaJson(Stream &s);

	// ---------------- Parameter Getters ----------------
	// Motor direction reversal flags (0 = normal, 1 = invert). Applied after expo, before mapping to microseconds.
	static uint8_t reverseLeft() { return _data.reverseLeft; }
	static uint8_t reverseRight() { return _data.reverseRight; }
	// Motor output exponential shaping (0..1000). 0 = linear, higher softens center & increases end authority.
	static uint16_t motorExpoL() { return _data.motorExpoL; }
	static uint16_t motorExpoR() { return _data.motorExpoR; }
	// Heading hold master enable (0 = disable feature at runtime; 1 = allow CH5 toggle to activate).
	static uint8_t headingHoldEnabled() { return _data.headingHoldEnabled; }
	// Heading error deadband in degrees. Inside this band correction is suppressed and the integrator is decayed.
	static float headingDeadbandDeg() { return _data.headingDeadbandDeg; }
	// PID gains converting heading error (deg) to yaw command (-1000..+1000 domain before mix). Tuned empirically.
	static float headKp() { return _data.headKp; }
	static float headKi() { return _data.headKi; }
	static float headKd() { return _data.headKd; }
	// Maximum absolute yaw command the PID may emit (command units). Prevents over-correction.
	static float headCmdMax() { return _data.headCmdMax; }
	// Below this average absolute motor command (0..1000) we treat craft as stationary and allow spin-in-place.
	static uint16_t speedZeroThresh() { return _data.speedZeroThresh; }
	// Fraction (0.0..1.0) of full output considered "high speed"; yaw correction is applied asymmetrically above this.
	static float speedHighFrac() { return _data.speedHighFrac; }
	// When nearly stationary, minimum absolute motor command used to initiate a spin (prevents weak ineffective spins).
	static uint16_t spinCmdMin() { return _data.spinCmdMin; }
	// Cap on spin command magnitude to avoid abrupt aggressive turns when stationary.
	static uint16_t spinCmdMax() { return _data.spinCmdMax; }
	// Failsafe heuristic master flag (0 = disable stuck-signal detection, 1 = enable). Currently compile-time gate also used.
	static uint8_t failsafeStuckThr() { return _data.failsafeStuckThr; }
	// Maximum delta (us) in pulse width considered "unchanged" for stuck detection.
	static uint16_t stuckDeltaUs() { return _data.stuckDeltaUs; }
	// Number of consecutive 20 ms cycles both yaw/throttle must remain within stuckDeltaUs to trigger neutral-hold.
	static uint16_t stuckCycles() { return _data.stuckCycles; }
	// RC stick deadband in command units (+/-1000 scale). Values inside become 0 to avoid jitter.
	static int16_t deadCenter() { return _data.deadCenter; }
	// Maximum age (ms) since last PWM edge before channel deemed stale and overall RC invalid.
	static uint16_t staleTimeoutMs() { return _data.staleTimeoutMs; }
	// Air mode incremental setpoint gain per loop at full stick deflection (command units per 20 ms).
	static int16_t airGainPerCycle() { return _data.airGainPerCycle; }
	// Heading mode incremental speed gain per loop at full stick deflection (command units per 20 ms).
	static int16_t hdgGainPerCycle() { return _data.hdgGainPerCycle; }
	// Adaptive heading boost: multiplier applied to heading deadband to start linear boost ramp (e.g. 2.0 => start at 2x deadband)
	static float headBoostTriggerMult() { return _data.headBoostTriggerMult; }
	// Adaptive heading boost maximum multiplier (1.0 = no boost, 1.35 = +35% authority at large errors)
	static float headMaxBoost() { return _data.headMaxBoost; }
	// Motor start offset microseconds (applied symmetrically for small |cmd| just above zero)
	static uint16_t motorStartUsL() { return _data.motorStartUsL; }
	static uint16_t motorStartUsR() { return _data.motorStartUsR; }
	// Motor scalar (per-motor thrust balance) 1.0 = no change
	static float motorScaleL() { return _data.motorScaleL; }
	static float motorScaleR() { return _data.motorScaleR; }
	// Command-domain magnitude threshold under which start offset is applied
	static uint16_t motorStartRegion() { return _data.motorStartRegion; }
	// Compass mounting correction in degrees (0..359). Added to raw BNO055 reading to correct for physical orientation.
	static int16_t compassCorrectionDeg() { return _data.compassCorrectionDeg; }

private:
	struct ConfigData
	{
		uint8_t reverseLeft;		  // 0/1 invert left motor direction
		uint8_t reverseRight;		  // 0/1 invert right motor direction
		uint16_t motorExpoL;		  // 0..1000 expo for left motor output shaping
		uint16_t motorExpoR;		  // 0..1000 expo for right motor output shaping
		uint8_t headingHoldEnabled;	  // 0 disable feature at runtime, 1 allow activation via CH5
		float headingDeadbandDeg;	  // deg threshold where corrections halt & integrator decays
		float headKp;				  // proportional gain (deg -> command units)
		float headKi;				  // integral gain (deg*s -> command units)
		float headKd;				  // derivative gain (deg/s -> command units)
		float headCmdMax;			  // max absolute yaw command emitted by PID
		uint16_t speedZeroThresh;	  // avg |cmd| below this treated as stationary for spin logic
		float speedHighFrac;		  // fraction of full output considered high speed (0..1)
		uint16_t spinCmdMin;		  // minimum spin command magnitude when stationary
		uint16_t spinCmdMax;		  // maximum spin command magnitude when stationary
		uint8_t failsafeStuckThr;	  // master enable for stuck-signal heuristic
		uint16_t stuckDeltaUs;		  // max microsecond delta counted as unchanged
		uint16_t stuckCycles;		  // consecutive cycles required to declare stuck
		int16_t deadCenter;			  // stick deadband (+/- command units)
		uint16_t staleTimeoutMs;	  // ms since last edge before channel invalid
		int16_t airGainPerCycle;	  // Air mode setpoint increment at full stick per loop
		int16_t hdgGainPerCycle;	  // Heading mode forward speed increment at full stick per loop
		float headBoostTriggerMult;	  // Multiplier of deadband where boost begins (e.g. 2.0)
		float headMaxBoost;			  // Maximum adaptive boost multiplier (>=1.0)
		uint16_t motorStartUsL;		  // Start offset left (µs)
		uint16_t motorStartUsR;		  // Start offset right (µs)
		float motorScaleL;			  // Scale factor left
		float motorScaleR;			  // Scale factor right
		uint16_t motorStartRegion;	  // Command magnitude threshold for start offset (e.g. 150)
		int16_t compassCorrectionDeg; // 0..359 compass mounting correction degrees (ADDED AT END for EEPROM compatibility)
	};
	struct Entry
	{
		const __FlashStringHelper *name; // PROGMEM string
		ParamType type;
		void *valuePtr;
		const void *defaultPtr; // points into default struct
	};

	static ConfigData _data;		   // live values
	static const ConfigData _defaults; // compile-time defaults
	static bool _dirty;				   // pending changes
	static void loadDefaultsIntoData();
	static Entry *table();
	static size_t tableSize();
	static Entry *find(const char *name);
	static const Meta *metaTable();
	static size_t metaSize();
	static const Meta *findMeta(const char *name);
};
