// ConfigStore.cpp - implementation

#include <EEPROM.h>
#include "ConfigStore.h"

// -----------------------------------------------------------------------------
// Compile-time default fallbacks
// Each configurable field has a macro fallback here so that if a project-wide
// define is supplied (e.g. via build flags), first-run EEPROM initialization
// inherits those values. Afterwards user changes persist in EEPROM until a
// resetDefaults() call or signature/version change.
// These mirrors reflect historical hard-coded constants from main.cpp.
// -----------------------------------------------------------------------------
#ifndef REVERSE_LEFT
#define REVERSE_LEFT 0
#endif
#ifndef REVERSE_RIGHT
#define REVERSE_RIGHT 0
#endif
#ifndef MOTOR_EXPO
#define MOTOR_EXPO 400
#endif
#ifndef MOTOR_EXPO_L
#define MOTOR_EXPO_L MOTOR_EXPO
#endif
#ifndef MOTOR_EXPO_R
#define MOTOR_EXPO_R MOTOR_EXPO
#endif
#ifndef HEADHOLD_ENABLED
#define HEADHOLD_ENABLED 1
#endif
#ifndef HEADING_DEADBAND_DEG
#define HEADING_DEADBAND_DEG 5.0f
#endif
#ifndef HEAD_KP
#define HEAD_KP 3.5f
#endif
#ifndef HEAD_KI
#define HEAD_KI 0.05f
#endif
#ifndef HEAD_KD
#define HEAD_KD 0.8f
#endif
#ifndef HEAD_CMD_MAX
#define HEAD_CMD_MAX 400.0f
#endif
#ifndef SPEED_ZERO_THRESH
#define SPEED_ZERO_THRESH 50
#endif
#ifndef SPEED_HIGH_FRAC
#define SPEED_HIGH_FRAC 0.80f
#endif
#ifndef SPIN_CMD_MIN
#define SPIN_CMD_MIN 180
#endif
#ifndef SPIN_CMD_MAX
#define SPIN_CMD_MAX 700
#endif
#ifndef FAILSAFE_STUCK_THR
#define FAILSAFE_STUCK_THR 1
#endif
#ifndef STUCK_DELTA_US
#define STUCK_DELTA_US 1
#endif
#ifndef STUCK_CYCLES
#define STUCK_CYCLES 75
#endif
#ifndef DEAD_CENTER
#define DEAD_CENTER 50
#endif
#ifndef STALE_TIMEOUT_MS
#define STALE_TIMEOUT_MS 100
#endif
#ifndef AIR_GAIN_PER_CYCLE
#define AIR_GAIN_PER_CYCLE 40
#endif
#ifndef HDG_GAIN_PER_CYCLE
#define HDG_GAIN_PER_CYCLE 15
#endif

// Static storage
ConfigStore::ConfigData ConfigStore::_data = {};
const ConfigStore::ConfigData ConfigStore::_defaults = {
	(uint8_t)REVERSE_LEFT,
	(uint8_t)REVERSE_RIGHT,
	(uint16_t)MOTOR_EXPO_L,
	(uint16_t)MOTOR_EXPO_R,
	(uint8_t)HEADHOLD_ENABLED,
	(float)HEADING_DEADBAND_DEG,
	(float)HEAD_KP,
	(float)HEAD_KI,
	(float)HEAD_KD,
	(float)HEAD_CMD_MAX,
	(uint16_t)SPEED_ZERO_THRESH,
	(float)SPEED_HIGH_FRAC,
	(uint16_t)SPIN_CMD_MIN,
	(uint16_t)SPIN_CMD_MAX,
	(uint8_t)FAILSAFE_STUCK_THR,
	(uint16_t)STUCK_DELTA_US,
	(uint16_t)STUCK_CYCLES,
	(int16_t)DEAD_CENTER,
	(uint16_t)STALE_TIMEOUT_MS,
	(int16_t)AIR_GAIN_PER_CYCLE,
	(int16_t)HDG_GAIN_PER_CYCLE};
bool ConfigStore::_dirty = false;

// PROGMEM names
static const char n_reverseLeft[] PROGMEM = "reverse_left";
static const char n_reverseRight[] PROGMEM = "reverse_right";
static const char n_motorExpoL[] PROGMEM = "motor_expo_l";
static const char n_motorExpoR[] PROGMEM = "motor_expo_r";
static const char n_headingHoldEn[] PROGMEM = "heading_hold_en";
static const char n_headingDeadband[] PROGMEM = "heading_deadband_deg";
static const char n_headKp[] PROGMEM = "head_kp";
static const char n_headKi[] PROGMEM = "head_ki";
static const char n_headKd[] PROGMEM = "head_kd";
static const char n_headCmdMax[] PROGMEM = "head_cmd_max";
static const char n_speedZeroThresh[] PROGMEM = "speed_zero_thresh";
static const char n_speedHighFrac[] PROGMEM = "speed_high_frac";
static const char n_spinCmdMin[] PROGMEM = "spin_cmd_min";
static const char n_spinCmdMax[] PROGMEM = "spin_cmd_max";
static const char n_failsafeStuckThr[] PROGMEM = "failsafe_stuck_thr";
static const char n_stuckDeltaUs[] PROGMEM = "stuck_delta_us";
static const char n_stuckCycles[] PROGMEM = "stuck_cycles";
static const char n_deadCenter[] PROGMEM = "dead_center";
static const char n_staleTimeoutMs[] PROGMEM = "stale_timeout_ms";
static const char n_airGainPerCycle[] PROGMEM = "air_gain_per_cycle";
static const char n_hdgGainPerCycle[] PROGMEM = "hdg_gain_per_cycle";

ConfigStore::ConfigStore::Entry *ConfigStore::table()
{
	static Entry entries[] = {
		{(const __FlashStringHelper *)n_reverseLeft, PT_U8, &_data.reverseLeft, &_defaults.reverseLeft},
		{(const __FlashStringHelper *)n_reverseRight, PT_U8, &_data.reverseRight, &_defaults.reverseRight},
		{(const __FlashStringHelper *)n_motorExpoL, PT_U16, &_data.motorExpoL, &_defaults.motorExpoL},
		{(const __FlashStringHelper *)n_motorExpoR, PT_U16, &_data.motorExpoR, &_defaults.motorExpoR},
		{(const __FlashStringHelper *)n_headingHoldEn, PT_U8, &_data.headingHoldEnabled, &_defaults.headingHoldEnabled},
		{(const __FlashStringHelper *)n_headingDeadband, PT_FLOAT, &_data.headingDeadbandDeg, &_defaults.headingDeadbandDeg},
		{(const __FlashStringHelper *)n_headKp, PT_FLOAT, &_data.headKp, &_defaults.headKp},
		{(const __FlashStringHelper *)n_headKi, PT_FLOAT, &_data.headKi, &_defaults.headKi},
		{(const __FlashStringHelper *)n_headKd, PT_FLOAT, &_data.headKd, &_defaults.headKd},
		{(const __FlashStringHelper *)n_headCmdMax, PT_FLOAT, &_data.headCmdMax, &_defaults.headCmdMax},
		{(const __FlashStringHelper *)n_speedZeroThresh, PT_U16, &_data.speedZeroThresh, &_defaults.speedZeroThresh},
		{(const __FlashStringHelper *)n_speedHighFrac, PT_FLOAT, &_data.speedHighFrac, &_defaults.speedHighFrac},
		{(const __FlashStringHelper *)n_spinCmdMin, PT_U16, &_data.spinCmdMin, &_defaults.spinCmdMin},
		{(const __FlashStringHelper *)n_spinCmdMax, PT_U16, &_data.spinCmdMax, &_defaults.spinCmdMax},
		{(const __FlashStringHelper *)n_failsafeStuckThr, PT_U8, &_data.failsafeStuckThr, &_defaults.failsafeStuckThr},
		{(const __FlashStringHelper *)n_stuckDeltaUs, PT_U16, &_data.stuckDeltaUs, &_defaults.stuckDeltaUs},
		{(const __FlashStringHelper *)n_stuckCycles, PT_U16, &_data.stuckCycles, &_defaults.stuckCycles},
		{(const __FlashStringHelper *)n_deadCenter, PT_I16, &_data.deadCenter, &_defaults.deadCenter},
		{(const __FlashStringHelper *)n_staleTimeoutMs, PT_U16, &_data.staleTimeoutMs, &_defaults.staleTimeoutMs},
		{(const __FlashStringHelper *)n_airGainPerCycle, PT_I16, &_data.airGainPerCycle, &_defaults.airGainPerCycle},
		{(const __FlashStringHelper *)n_hdgGainPerCycle, PT_I16, &_data.hdgGainPerCycle, &_defaults.hdgGainPerCycle},
	};
	return entries;
}

size_t ConfigStore::tableSize()
{
	return 21; // keep in sync with entries above
}

void ConfigStore::loadDefaultsIntoData()
{
	_data = _defaults;
	_dirty = true;
}

ConfigStore::Entry *ConfigStore::find(const char *name)
{
	if (!name)
		return nullptr;
	Entry *ents = table();
	size_t n = tableSize();
	for (size_t i = 0; i < n; ++i)
	{
		// Compare PROGMEM string with RAM name
		char buf[26]; // names are short
		strncpy_P(buf, (PGM_P)ents[i].name, sizeof(buf) - 1);
		buf[sizeof(buf) - 1] = '\0';
		if (strcmp(buf, name) == 0)
			return &ents[i];
	}
	return nullptr;
}

// EEPROM layout: [0..3] signature 'K','C','1','C'  [4] version  [5..] ConfigData binary
static const uint8_t CFG_SIGNATURE[4] = {'K', 'C', '1', 'C'};
static const uint8_t CFG_VERSION = 1;

void ConfigStore::begin()
{
	bool sigOk = true;
	for (uint8_t i = 0; i < 4; i++)
	{
		if (EEPROM.read(i) != CFG_SIGNATURE[i])
		{
			sigOk = false;
			break;
		}
	}
	if (sigOk && EEPROM.read(4) == CFG_VERSION)
	{
		// Load
		for (size_t b = 0; b < sizeof(ConfigData); ++b)
		{
			*((uint8_t *)&_data + b) = EEPROM.read(5 + b);
		}
		_dirty = false;
	}
	else
	{
		// Initialize
		loadDefaultsIntoData();
		save();
	}
}

bool ConfigStore::save()
{
	if (!_dirty)
		return true;
	// Write signature/version then data
	for (uint8_t i = 0; i < 4; i++)
		EEPROM.update(i, CFG_SIGNATURE[i]);
	EEPROM.update(4, CFG_VERSION);
	for (size_t b = 0; b < sizeof(ConfigData); ++b)
	{
		EEPROM.update(5 + b, *((uint8_t *)&_data + b));
	}
	_dirty = false;
	return true;
}

void ConfigStore::resetDefaults()
{
	loadDefaultsIntoData();
	save();
}

size_t ConfigStore::count() { return tableSize(); }

const __FlashStringHelper *ConfigStore::nameAt(size_t idx)
{
	Entry *ents = table();
	if (idx >= tableSize())
		return nullptr;
	return ents[idx].name;
}

bool ConfigStore::get(const char *name, float &outVal)
{
	Entry *e = find(name);
	if (!e)
		return false;
	switch (e->type)
	{
	case PT_U8:
		outVal = *(uint8_t *)e->valuePtr;
		break;
	case PT_U16:
		outVal = *(uint16_t *)e->valuePtr;
		break;
	case PT_I16:
		outVal = *(int16_t *)e->valuePtr;
		break;
	case PT_FLOAT:
		outVal = *(float *)e->valuePtr;
		break;
	default:
		return false;
	}
	return true;
}

static long lroundf_safe(float v) { return (long)(v >= 0 ? v + 0.5f : v - 0.5f); }

bool ConfigStore::set(const char *name, float newVal)
{
	Entry *e = find(name);
	if (!e)
		return false;
	bool changed = false;
	switch (e->type)
	{
	case PT_U8:
	{
		uint8_t nv = (uint8_t)constrain(lroundf_safe(newVal), 0L, 255L);
		if (*(uint8_t *)e->valuePtr != nv)
		{
			*(uint8_t *)e->valuePtr = nv;
			changed = true;
		}
		break;
	}
	case PT_U16:
	{
		uint16_t nv = (uint16_t)constrain(lroundf_safe(newVal), 0L, 65535L);
		if (*(uint16_t *)e->valuePtr != nv)
		{
			*(uint16_t *)e->valuePtr = nv;
			changed = true;
		}
		break;
	}
	case PT_I16:
	{
		long nvL = constrain(lroundf_safe(newVal), -32768L, 32767L);
		int16_t nv = (int16_t)nvL;
		if (*(int16_t *)e->valuePtr != nv)
		{
			*(int16_t *)e->valuePtr = nv;
			changed = true;
		}
		break;
	}
	case PT_FLOAT:
	{
		float nv = newVal;
		if (*(float *)e->valuePtr != nv)
		{
			*(float *)e->valuePtr = nv;
			changed = true;
		}
		break;
	}
	default:
		return false;
	}
	if (changed)
	{
		_dirty = true;
		save();
	}
	return changed;
}

void ConfigStore::list(Stream &s)
{
	Entry *ents = table();
	size_t n = tableSize();
	for (size_t i = 0; i < n; i++)
	{
		char nameBuf[26];
		strncpy_P(nameBuf, (PGM_P)ents[i].name, sizeof(nameBuf) - 1);
		nameBuf[sizeof(nameBuf) - 1] = '\0';
		s.print(nameBuf);
		s.print('=');
		switch (ents[i].type)
		{
		case PT_U8:
			s.println(*(uint8_t *)ents[i].valuePtr);
			break;
		case PT_U16:
			s.println(*(uint16_t *)ents[i].valuePtr);
			break;
		case PT_I16:
			s.println(*(int16_t *)ents[i].valuePtr);
			break;
		case PT_FLOAT:
			s.println(*(float *)ents[i].valuePtr, 4);
			break;
		default:
			s.println('?');
			break;
		}
	}
}
