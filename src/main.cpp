// Kayak Controller v1 for Arduino Uno (ATmega328P, 16 MHz)
// - Reads 4 RC channels via pin change interrupts (CH1 yaw, CH2 throttle, CH3 Normal button, CH4 Air button)
// - Maps 1000–2000 us to -1000..+1000
// - Modes: Normal (direct mix) and Air (latched setpoints with incremental adjust)
// - Outputs two ESC signals at 50 Hz using Servo (Timer1)
// - Starts Disarmed; requires sticks centered and mode button press to arm
// - 20 ms loop cadence with millis()

#include <Arduino.h>
#include <Servo.h>
// Compass / IMU
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "ConfigStore.h"

// (SoftwareSerial removed due to ISR vector conflicts with PCINT-based RC capture)

// ========= Debug configuration =========
// Set DEBUG_ENABLED to 0 to disable all Serial prints at compile time
#ifndef DEBUG_ENABLED
#define DEBUG_ENABLED 0
#endif
// DEBUG_STYLE: 0 = verbose (current), 1 = pseudo-graphics line
#ifndef DEBUG_STYLE
#define DEBUG_STYLE 0
#endif

// ==== Firmware identity (4-part version: MAJOR.MINOR.PATCH.HOTFIX) ====
// Versioning policy:
//  - MAJOR (breaking architectural changes) updated manually.
//  - MINOR ("big" feature sets / mode additions) increments for significant additions.
//  - PATCH (small feature / enhancement / non-breaking additions) increments frequently.
//  - HOTFIX (urgent post-release fix) increments only for targeted fixes; otherwise 0.
// These can be overridden via build flags (e.g. PlatformIO build_flags).
#ifndef KC1_VER_MAJOR
#define KC1_VER_MAJOR 0
#endif
#ifndef KC1_VER_MINOR
#define KC1_VER_MINOR 4
#endif
#ifndef KC1_VER_PATCH
#define KC1_VER_PATCH 1
#endif
#ifndef KC1_VER_HOTFIX
#define KC1_VER_HOTFIX 0
#endif

// Stringify helpers
#define KC1_STR_HELPER(x) #x
#define KC1_STR(x) KC1_STR_HELPER(x)

#ifndef KC1_API_VERSION
#define KC1_API_VERSION KC1_STR(KC1_VER_MAJOR) "." KC1_STR(KC1_VER_MINOR) "." KC1_STR(KC1_VER_PATCH) "." KC1_STR(KC1_VER_HOTFIX)
#endif

// Optionally define at build: -DKC1_GIT_HASH=\"abcdef1\"
#ifndef KC1_GIT_HASH
#define KC1_GIT_HASH "unknown"
#endif
#ifndef KC1_BUILD_DATE
#define KC1_BUILD_DATE __DATE__
#endif
#ifndef KC1_BUILD_TIME
#define KC1_BUILD_TIME __TIME__
#endif

// ========= Output configuration (dynamic via ConfigStore) =========
static inline bool reverseLeft() { return ConfigStore::reverseLeft() != 0; }
static inline bool reverseRight() { return ConfigStore::reverseRight() != 0; }
static inline uint16_t motorExpoL() { return (uint16_t)ConfigStore::motorExpoL(); }
static inline uint16_t motorExpoR() { return (uint16_t)ConfigStore::motorExpoR(); }
static inline uint16_t cfgMotorStartUsL() { return ConfigStore::motorStartUsL(); }
static inline uint16_t cfgMotorStartUsR() { return ConfigStore::motorStartUsR(); }
static inline float cfgMotorScaleL() { return ConfigStore::motorScaleL(); }
static inline float cfgMotorScaleR() { return ConfigStore::motorScaleR(); }
static inline uint16_t cfgMotorStartRegion() { return ConfigStore::motorStartRegion(); }

// ========= Heading hold (BNO055) ========= (always compiled; runtime enable via ConfigStore)

// ========= Failsafe tuning ========= (runtime via ConfigStore)

// ========= Pin assignments =========
// RC input pins (PORTD PCINT2 group: D4..D7)
static const uint8_t CH1_YAW_PIN = 4; // PD4 / PCINT20
static const uint8_t CH2_THR_PIN = 5; // PD5 / PCINT21
static const uint8_t CH3_NRM_PIN = 6; // PD6 / PCINT22
static const uint8_t CH4_AIR_PIN = 7; // PD7 / PCINT23
// Additional RC input on PORTB (PCINT0 group): D8 for Heading Hold
static const uint8_t CH5_HOLD_PIN = 8; // PB0 / PCINT0

// ESC outputs (Timer1 capable pins for Servo lib)
static const uint8_t ESC_L_PIN = 9;	 // PB1 / OC1A
static const uint8_t ESC_R_PIN = 10; // PB2 / OC1B

// ========= RC capture (ISR) =========
// Volatile data shared with ISR
volatile uint16_t g_pulseWidthUs[5] = {1500, 1500, 1000, 1000, 1000};
volatile uint32_t g_riseTimeUs[5] = {0, 0, 0, 0, 0};
volatile uint32_t g_prevRiseUs[5] = {0, 0, 0, 0, 0};
volatile uint32_t g_lastFrameUs[5] = {0, 0, 0, 0, 0}; // last time we observed a plausible PWM frame period
volatile uint8_t g_lastPortD = 0;
volatile uint8_t g_lastPortB = 0;
volatile uint32_t g_lastUpdateUs[5] = {0, 0, 0, 0, 0};

// Bit masks for D4..D7
static const uint8_t MASK_CH1 = _BV(PD4);
static const uint8_t MASK_CH2 = _BV(PD5);
static const uint8_t MASK_CH3 = _BV(PD6);
static const uint8_t MASK_CH4 = _BV(PD7);
static const uint8_t MASK_CH5 = _BV(PB0);

static inline void pcint2Enable()
{
	// Enable Pin Change Interrupts for PORTD (PCIE2)
	PCICR |= _BV(PCIE2);
	// Enable PCINT for PD4..PD7 (PCINT20..23)
	PCMSK2 |= _BV(PCINT20) | _BV(PCINT21) | _BV(PCINT22) | _BV(PCINT23);
}

static inline void pcint0Enable()
{
	// Enable Pin Change Interrupts for PORTB (PCIE0)
	PCICR |= _BV(PCIE0);
	// Enable PCINT for PB0 (PCINT0)
	PCMSK0 |= _BV(PCINT0);
}

ISR(PCINT2_vect)
{
	uint8_t nowD = PIND; // read PORTD snapshot
	uint8_t changed = nowD ^ g_lastPortD;
	g_lastPortD = nowD;

	uint32_t nowUs = micros();

	// Channel helper lambda
	auto handleEdge = [&](uint8_t mask, uint8_t idx)
	{
		if (!(changed & mask))
			return;
		if (nowD & mask)
		{
			// Rising edge
			g_riseTimeUs[idx] = nowUs;
			// Detect plausible frame period (~50 Hz => ~20 ms). Use generous 8-40 ms window
			uint32_t prev = g_prevRiseUs[idx];
			if (prev != 0)
			{
				uint32_t period = nowUs - prev;
				if (period >= 8000UL && period <= 40000UL)
				{
					g_lastFrameUs[idx] = nowUs;
				}
			}
			g_prevRiseUs[idx] = nowUs;
		}
		else
		{
			// Falling edge
			uint32_t start = g_riseTimeUs[idx];
			if (start != 0)
			{
				uint32_t w = nowUs - start;
				if (w >= 800 && w <= 2500)
				{
					g_pulseWidthUs[idx] = (uint16_t)w;
					g_lastUpdateUs[idx] = nowUs;
				}
			}
		}
	};

	handleEdge(MASK_CH1, 0);
	handleEdge(MASK_CH2, 1);
	handleEdge(MASK_CH3, 2);
	handleEdge(MASK_CH4, 3);
}

ISR(PCINT0_vect)
{
	uint8_t nowB = PINB; // read PORTB snapshot
	uint8_t changed = nowB ^ g_lastPortB;
	g_lastPortB = nowB;

	uint32_t nowUs = micros();

	if (changed & MASK_CH5)
	{
		uint8_t idx = 4; // CH5 index
		if (nowB & MASK_CH5)
		{
			g_riseTimeUs[idx] = nowUs;
			uint32_t prev = g_prevRiseUs[idx];
			if (prev != 0)
			{
				uint32_t period = nowUs - prev;
				if (period >= 8000UL && period <= 40000UL)
				{
					g_lastFrameUs[idx] = nowUs;
				}
			}
			g_prevRiseUs[idx] = nowUs;
		}
		else
		{
			uint32_t start = g_riseTimeUs[idx];
			if (start != 0)
			{
				uint32_t w = nowUs - start;
				if (w >= 800 && w <= 2500)
				{
					g_pulseWidthUs[idx] = (uint16_t)w;
					g_lastUpdateUs[idx] = nowUs;
				}
			}
		}
	}
}

// Safe copy of RC pulse width in microseconds
static inline uint16_t getPulseWidthUs(uint8_t chIdx)
{
	uint16_t v;
	noInterrupts();
	v = g_pulseWidthUs[chIdx];
	interrupts();
	return v;
}

// Time since last RC update for a channel (microseconds)
static inline uint32_t getSinceUpdateUs(uint8_t chIdx)
{
	noInterrupts();
	uint32_t last = g_lastUpdateUs[chIdx];
	interrupts();
	if (last == 0)
		return UINT32_MAX;
	uint32_t now = micros();
	return now - last;
}

// ========= Mapping & utilities =========
static inline int16_t mapUsToSigned1000(uint16_t us)
{
	// Clamp first to 1000..2000
	if (us < 1000)
		us = 1000;
	else if (us > 2000)
		us = 2000;
	// Center at 1500 -> 0; 1000 -> -1000; 2000 -> +1000
	int16_t signedVal = (int16_t)us - 1500; // -500..+500
	return (int16_t)(signedVal * 2);		// -1000..+1000
}

static inline uint16_t mapSigned1000ToUs(int16_t v)
{
	if (v < -1000)
		v = -1000;
	else if (v > 1000)
		v = 1000;
	return (uint16_t)(1500 + (v / 2)); // -1000 -> 1000us, +1000 -> 2000us
}

static inline int16_t applyDeadband(int16_t v, int16_t db)
{
	if (v > -db && v < db)
		return 0;
	return v;
}

// Apply exponential curve while preserving endpoints and sign.
// expo: 0..1000, where 0 = linear, 1000 blends fully to cubic (center softer, ends stronger)
static inline int16_t applyExpoSigned1000(int16_t v, uint16_t expo)
{
	if (expo == 0)
		return v;
	if (v == 0)
		return 0;
	int32_t x = v;						 // -1000..1000
	int32_t x3 = (x * x * x) / 1000000L; // divide by 1000^2 to keep scale
	// y = x*(1 - e) + x^3 * e, with e in [0..1] scaled by 1000
	int32_t y = (x * (int32_t)(1000 - expo) + x3 * (int32_t)expo) / 1000;
	if (y > 1000)
		y = 1000;
	else if (y < -1000)
		y = -1000;
	return (int16_t)y;
}

// ========= Mode & mixing =========
enum Mode : uint8_t
{
	MODE_DISARMED = 0,
	MODE_NORMAL = 1,
	MODE_AIR = 2,
	MODE_HEADING = 3
};

struct RcInputs
{
	uint16_t usYaw;		// CH1
	uint16_t usThr;		// CH2
	uint16_t usBtnNorm; // CH3
	uint16_t usBtnAir;	// CH4
	uint16_t usBtnHold; // CH5 (heading hold)
	int16_t yaw;		// -1000..1000
	int16_t thr;		// -1000..1000
	bool btnNormal;		// >1500us
	bool btnAir;		// >1500us
	bool btnHold;		// >1500us
	bool valid;			// all channels fresh
};

// Global state
static Mode g_mode = MODE_DISARMED;
static bool g_armed = false;
static int16_t g_airYawSet = 0;			 // -1000..1000 (latched)
static int16_t g_airThrSet = 0;			 // -1000..1000 (latched)
static bool g_skipAirUpdateOnce = false; // when true, skip one Air update to enforce neutral immediately
static bool g_holdFailsafe = false;		 // constant-signal heuristic: hold motors neutral without disarming
// Heading mode specific state
static int16_t g_headingModeSpeed = 0; // 0..1000 forward-only speed command while in MODE_HEADING

// Heading-hold state (always compiled)
// BNO055 and heading-hold state
static Adafruit_BNO055 g_bno = Adafruit_BNO055(55, 0x28);
static bool g_bnoOk = false;
static bool g_headingHoldActive = false;
static float g_headingTargetDeg = 0.0f; // 0..360
static float g_headErrInt = 0.0f;
static float g_headPrevErr = 0.0f;
static uint32_t g_headLastMs = 0;
// Telemetry: last commanded motor values (signed command domain and microseconds)
static int16_t g_lastCmdL = 0;
static int16_t g_lastCmdR = 0;
static uint16_t g_lastUsL = 1500;
static uint16_t g_lastUsR = 1500;

// disarm moved early to avoid any forward-decl issues

static float normalizeAngleDeg(float a)
{
	while (a >= 360.0f)
		a -= 360.0f;
	while (a < 0.0f)
		a += 360.0f;
	return a;
}

static float shortestDiffDeg(float target, float current)
{
	float diff = normalizeAngleDeg(target) - normalizeAngleDeg(current);
	if (diff > 180.0f)
		diff -= 360.0f;
	if (diff < -180.0f)
		diff += 360.0f;
	return diff;
}

static bool bnoBeginAuto()
{
	if (g_bno.begin())
		return true;
	Adafruit_BNO055 alt(55, 0x29);
	if (alt.begin())
	{
		g_bno = alt;
		return true;
	}
	return false;
}

static bool readHeadingDeg(float &headingOut)
{
	if (!g_bnoOk)
		return false;
	imu::Vector<3> euler = g_bno.getVector(Adafruit_BNO055::VECTOR_EULER);
	float h = euler.x();
	if (isnan(h))
		return false;
	headingOut = normalizeAngleDeg(h);
	return true;
}

static inline float cfgHeadKp() { return ConfigStore::headKp(); }
static inline float cfgHeadKi() { return ConfigStore::headKi(); }
static inline float cfgHeadKd() { return ConfigStore::headKd(); }
static inline float cfgHeadCmdMax() { return ConfigStore::headCmdMax(); }
static inline float cfgHeadingDeadband() { return ConfigStore::headingDeadbandDeg(); }
static inline uint16_t cfgSpeedZeroThresh() { return (uint16_t)ConfigStore::speedZeroThresh(); }
static inline float cfgSpeedHighFrac() { return ConfigStore::speedHighFrac() / 1000.0f; }
static inline int16_t cfgSpinCmdMin() { return ConfigStore::spinCmdMin(); }
static inline int16_t cfgSpinCmdMax() { return ConfigStore::spinCmdMax(); }

static float headingPidStep(float errDeg, float dtSec)
{
	// Integrator with light decay to reduce residual bias
	g_headErrInt += errDeg * dtSec;

	// Simple anti-windup when output is (likely) saturated: if last output magnitude was near limit and
	// new error is still large and of same sign, damp further integral growth.
	float cmdMax = cfgHeadCmdMax();
	float absPrev = fabsf(g_headPrevErr * cfgHeadKp()); // rough proxy before full term
	if (absPrev > (cmdMax * 0.9f) && (errDeg * g_headPrevErr) > 0.0f)
	{
		g_headErrInt *= 0.98f; // small bleed against windup
	}
	// When error is within half the deadband, bleed off integral quickly
	if (fabsf(errDeg) <= (cfgHeadingDeadband() * 0.5f))
	{
		g_headErrInt *= 0.7f; // decay towards 0
		if (fabsf(errDeg) < 0.2f)
		{
			g_headErrInt = 0.0f; // essentially aligned
		}
	}
	float Kp = cfgHeadKp();
	float Ki = cfgHeadKi();
	float Kd = cfgHeadKd();
	cmdMax = cfgHeadCmdMax();
	float iMax = (Ki > 0.0f) ? (cmdMax / Ki) : 0.0f;
	if (Ki > 0.0f)
	{
		if (g_headErrInt > iMax)
			g_headErrInt = iMax;
		else if (g_headErrInt < -iMax)
			g_headErrInt = -iMax;
	}
	float d = (dtSec > 0.0f) ? ((errDeg - g_headPrevErr) / dtSec) : 0.0f;
	g_headPrevErr = errDeg;
	float out = Kp * errDeg + Ki * g_headErrInt + Kd * d;
	if (out > cmdMax)
		out = cmdMax;
	else if (out < -cmdMax)
		out = -cmdMax;
	return out;
}

static void applyHeadingHoldIfNeeded(int16_t &cmdL, int16_t &cmdR)
{
	if (!g_headingHoldActive || !g_bnoOk)
		return;
	// Preserve base (pre-correction) commands so we can restore them when error is small
	int16_t baseL = cmdL;
	int16_t baseR = cmdR;
	float heading;
	if (!readHeadingDeg(heading))
		return;
	float err = shortestDiffDeg(g_headingTargetDeg, heading);
	float aerr = fabsf(err);

	uint32_t nowMs = millis();
	float dt = (g_headLastMs == 0) ? 0.02f : (nowMs - g_headLastMs) / 1000.0f;
	g_headLastMs = nowMs;

	// Evaluate current output magnitude before deciding actions
	int16_t avgMag = (abs(cmdL) + abs(cmdR)) / 2;
	float frac = avgMag / 1000.0f;

	// Near-zero error handling: bleed integrator and RESTORE base outputs
	if (aerr <= cfgHeadingDeadband())
	{
		// Run integrator decay via PID step with tiny dt to keep state consistent
		(void)headingPidStep(err, dt);
		// Restore original pre-correction outputs so Air setpoints are respected
		cmdL = baseL;
		cmdR = baseR;
		return;
	}

	float yawCmdF = headingPidStep(err, dt);
	// Previously inverted (yawCmdF = -yawCmdF;) but behavior was opposite in field.
	// Use direct PID output now. Add configurable inversion later if needed.
	//	yawCmdF = -yawCmdF;
	int16_t yawCmd = (int16_t)yawCmdF;
	// Avoid tiny bias from noise
	if (abs((int)yawCmd) < 20)
		yawCmd = 0;

	// Adaptive linear boost: scale up gently with larger heading error while respecting kayak's limited turn rate.
	// Rationale: abrupt fixed boosts can cause overshoot; here we linearly scale from 1.0x at 2*deadband
	// to a modest maximum (e.g. 1.35x) at a large but realistic error (e.g. 90 deg).
	{
		float db = cfgHeadingDeadband();
		float triggerErr = db * ConfigStore::headBoostTriggerMult(); // configurable start multiplier
		float e = fabsf(err);
		if (e > triggerErr && yawCmd != 0)
		{
			// Limit boost consideration if we're already near max authority
			int16_t absYaw = abs(yawCmd);
			int16_t cmdMax = (int16_t)cfgHeadCmdMax();
			if (absYaw < (int16_t)(cmdMax * 0.85f))
			{
				const float maxErrForBoost = 90.0f;					// beyond this treat as "fully boosted" (kayak practically turning as hard as allowed)
				const float maxBoost = ConfigStore::headMaxBoost(); // configurable gentle upper multiplier
				float t = (e - triggerErr) / (maxErrForBoost - triggerErr);
				if (t > 1.0f)
					t = 1.0f;
				else if (t < 0.0f)
					t = 0.0f;
				float mult = 1.0f + t * (maxBoost - 1.0f);
				int32_t boosted = (int32_t)((float)yawCmd * mult);
				if (boosted > cmdMax)
					boosted = cmdMax;
				else if (boosted < -cmdMax)
					boosted = -cmdMax;
				yawCmd = (int16_t)boosted;
			}
		}
	}

	if (avgMag <= (int16_t)cfgSpeedZeroThresh())
	{
		int16_t spinMag = abs(yawCmd);
		if (spinMag < cfgSpinCmdMin())
			spinMag = cfgSpinCmdMin();
		if (spinMag > cfgSpinCmdMax())
			spinMag = cfgSpinCmdMax();
		if (yawCmd > 0)
		{
			cmdL = spinMag;
			cmdR = -spinMag;
		}
		else
		{
			cmdL = -spinMag;
			cmdR = spinMag;
		}
	}
	else if (frac >= cfgSpeedHighFrac())
	{
		int16_t reduce = abs(yawCmd);
		if (reduce > 800)
			reduce = 800;
		if (yawCmd > 0)
		{
			cmdR -= reduce;
		}
		else
		{
			cmdL -= reduce;
		}
	}
	else
	{
		cmdL += yawCmd;
		cmdR -= yawCmd;
	}

	if (cmdL > 1000)
		cmdL = 1000;
	else if (cmdL < -1000)
		cmdL = -1000;
	if (cmdR > 1000)
		cmdR = 1000;
	else if (cmdR < -1000)
		cmdR = -1000;
}
// End heading-hold section

// Tuning
// Replaced fixed constants with ConfigStore-backed inline accessors
static inline int16_t cfgDeadCenter() { return ConfigStore::deadCenter(); }
static inline uint16_t cfgStaleTimeoutMs() { return ConfigStore::staleTimeoutMs(); }
static inline int16_t cfgAirGainPerCycle() { return ConfigStore::airGainPerCycle(); }

// ESC control
Servo escL, escR;

static RcInputs readRc()
{
	RcInputs r{};
	r.usYaw = getPulseWidthUs(0);
	r.usThr = getPulseWidthUs(1);
	r.usBtnNorm = getPulseWidthUs(2);
	r.usBtnAir = getPulseWidthUs(3);
	r.usBtnHold = getPulseWidthUs(4);

	r.yaw = applyDeadband(mapUsToSigned1000(r.usYaw), cfgDeadCenter());
	r.thr = applyDeadband(mapUsToSigned1000(r.usThr), cfgDeadCenter());
	r.btnNormal = r.usBtnNorm > 1500;
	r.btnAir = r.usBtnAir > 1500;
	r.btnHold = r.usBtnHold > 1500;

	// Validity: require yaw/throttle pulses to be fresh; ignore buttons for validity
	bool fresh = true;
	if (getSinceUpdateUs(0) > (uint32_t)cfgStaleTimeoutMs() * 1000UL)
		fresh = false; // yaw
	if (getSinceUpdateUs(1) > (uint32_t)cfgStaleTimeoutMs() * 1000UL)
		fresh = false; // thr

	// Stuck-signal heuristic (runtime enabled via failsafeStuckThr config flag)
	{
		static uint16_t lastThrUs = 0, lastYawUs = 0;
		static uint16_t thrStable = 0, yawStable = 0;
		if (lastThrUs == 0)
			lastThrUs = r.usThr;
		if (lastYawUs == 0)
			lastYawUs = r.usYaw;
		uint16_t stuckDelta = (uint16_t)ConfigStore::stuckDeltaUs();
		uint16_t stuckCycles = (uint16_t)ConfigStore::stuckCycles();
		if ((uint16_t)abs((int)r.usThr - (int)lastThrUs) <= stuckDelta)
		{
			if (thrStable < 0xFFFF)
				thrStable++;
		}
		else
		{
			thrStable = 0;
		}
		if ((uint16_t)abs((int)r.usYaw - (int)lastYawUs) <= stuckDelta)
		{
			if (yawStable < 0xFFFF)
				yawStable++;
		}
		else
		{
			yawStable = 0;
		}
		lastThrUs = r.usThr;
		lastYawUs = r.usYaw;
		if (ConfigStore::failsafeStuckThr() && thrStable >= stuckCycles && yawStable >= stuckCycles)
			g_holdFailsafe = true;
		else
			g_holdFailsafe = false;
	}
	r.valid = fresh;
	return r;
}

static void mixDifferential(int16_t thr, int16_t yaw, int16_t &outL, int16_t &outR)
{
	// Simple differential thrust
	long left = (long)thr + (long)yaw;
	long right = (long)thr - (long)yaw;
	if (left > 1000)
		left = 1000;
	else if (left < -1000)
		left = -1000;
	if (right > 1000)
		right = 1000;
	else if (right < -1000)
		right = -1000;
	outL = (int16_t)left;
	outR = (int16_t)right;
}

static void updateAirSetpoints(int16_t inputThr, int16_t inputYaw)
{
	// Incremental adjustment proportional to stick deflection
	int16_t gain = cfgAirGainPerCycle();
	int32_t stepThr = ((int32_t)inputThr * gain) / 1000;
	int32_t stepYaw = ((int32_t)inputYaw * gain) / 1000;

	int32_t newThr = (int32_t)g_airThrSet + stepThr;
	int32_t newYaw = (int32_t)g_airYawSet + stepYaw;

	if (newThr > 1000)
		newThr = 1000;
	else if (newThr < -1000)
		newThr = -1000;
	if (newYaw > 1000)
		newYaw = 1000;
	else if (newYaw < -1000)
		newYaw = -1000;

	g_airThrSet = (int16_t)newThr;
	g_airYawSet = (int16_t)newYaw;
}

static void writeEscOutputsUs(uint16_t usL, uint16_t usR)
{
	escL.writeMicroseconds(usL);
	escR.writeMicroseconds(usR);
}

// ========= Debug helpers =========
#if DEBUG_ENABLED && (DEBUG_STYLE == 0)
static void printDebugVerbose(const RcInputs &rc, Mode mode, bool armed, int16_t cmdL, int16_t cmdR)
{
	Serial.print(F("mode="));
	switch (mode)
	{
	case MODE_DISARMED:
		Serial.print(F("DISARMED"));
		break;
	case MODE_NORMAL:
		Serial.print(F("NORMAL"));
		break;
	case MODE_AIR:
		Serial.print(F("AIR"));
		break;
	default:
		break;
	}
	Serial.print(F(" armed="));
	Serial.print(armed ? F("1") : F("0"));
	Serial.print(F(" | in(us): yaw="));
	Serial.print((int)rc.usYaw);
	Serial.print(F(" thr="));
	Serial.print((int)rc.usThr);
	Serial.print(F(" nrm="));
	Serial.print((int)rc.usBtnNorm);
	Serial.print(F(" air="));
	Serial.print((int)rc.usBtnAir);
	Serial.print(F(" | in(\u00B1): yaw="));
	Serial.print((int)rc.yaw);
	Serial.print(F(" thr="));
	Serial.print((int)rc.thr);
	Serial.print(F(" | set: yaw="));
	Serial.print((int)g_airYawSet);
	Serial.print(F(" thr="));
	Serial.print((int)g_airThrSet);

	uint16_t usL = mapSigned1000ToUs(cmdL);
	Serial.print(F(" | out(us): L="));
	Serial.print((int)usL);
	Serial.print(F(" R="));
	Serial.print((int)usR);

	Serial.print(F(" | hdg:"));
	Serial.print(g_headingHoldActive ? F("ON") : F("OFF"));
	if (g_bnoOk)
	{
		float h;
		if (readHeadingDeg(h))
		{
			// Always show current heading; add target+error when hold is active
			Serial.print(F(" cur="));
			Serial.print(h);
			if (g_headingHoldActive)
			{
				float err = g_headingTargetDeg - h;
				if (err > 180.0f)
					err -= 360.0f;
				else if (err < -180.0f)
					err += 360.0f;
				Serial.print(F(" tgt="));
				Serial.print(g_headingTargetDeg);
				Serial.print(F(" err="));
				Serial.print(err);
			}
		}
		else
		{
			Serial.print(F(" cur=?"));
		}
	}
	Serial.println();
}
#endif // DEBUG_ENABLED && DEBUG_STYLE == 0

#if DEBUG_ENABLED
static void printDebugPseudo(const RcInputs &rc, Mode mode, bool armed, int16_t cmdL, int16_t cmdR)
{
	// Pseudo-graphics: single compact line with mode, buttons, motor direction/speed
	// Direction: '<' for reverse, '>' for forward; magnitude as -1000..+1000 and bar size
	auto printMotor = [](const __FlashStringHelper *label, int16_t cmd)
	{
		int16_t v = cmd;
		if (v > 1000)
			v = 1000;
		if (v < -1000)
			v = -1000;
		int16_t mag = v < 0 ? -v : v;
		uint8_t bars = (uint8_t)((mag + 49) / 100); // 0..10 roughly
		Serial.print(label);
		Serial.print(v >= 0 ? F("+") : F("-"));
		Serial.print((int)mag);
		Serial.print(F(" "));
		if (v < 0)
		{
			for (uint8_t i = 0; i < bars; ++i)
				Serial.print('<');
		}
		if (mode == MODE_DISARMED)
			Serial.print(F("DIS"));
		else if (mode == MODE_NORMAL)
			Serial.print(F("NRM"));
		else if (mode == MODE_AIR)
			Serial.print(F("AIR"));
		else if (mode == MODE_HEADING)
			Serial.print(F("HDG"));
		else
			Serial.print(F("?"));
		{
			for (uint8_t i = 0; i < bars; ++i)
				Serial.print('>');
		}
		else
		{
			Serial.print('-');
		}
	};

	// Header
	// Example: [AIR][ARM][N:1 A:0] L:+350 >>> R:-120 <<
	// Start with carriage return to overwrite the same line in terminal
	Serial.print('\r');
	uint8_t count = 0;
	Serial.print(F("["));
	count += 1;
	Serial.print(mode == MODE_DISARMED ? F("DIS") : (mode == MODE_NORMAL ? F("NRM") : F("AIR")));
	count += 3;
	Serial.print(F("]"));
	count += 1;
	Serial.print(armed ? F("[ARM]") : F("[DIS]"));
	count += 5;
	Serial.print(F("[N:"));
	count += 3;
	Serial.print(rc.btnNormal ? F("1") : F("0"));
	count += 1;
	Serial.print(F(" A:"));
	count += 3;
	Serial.print(rc.btnAir ? F("1") : F("0"));
	count += 1;
	Serial.print(F(" H:"));
	count += 3;
	Serial.print(rc.btnHold ? F("1") : F("0"));
	count += 1;
	Serial.print(F("] "));
	count += 2;

	// We cannot easily count chars inside printMotor; just print, then append raw values and pad.
	printMotor(F("L:"), cmdL);
	Serial.print(F(" "));
	printMotor(F("R:"), cmdR);

	// Append raw channel values for inspection (short labels to avoid wrapping)
	Serial.print(F(" | Y:"));
	Serial.print((int)rc.usYaw);
	Serial.print(F(" T:"));
	Serial.print((int)rc.usThr);
	Serial.print(F(" N:"));
	Serial.print((int)rc.usBtnNorm);
	Serial.print(F(" A:"));
	Serial.print((int)rc.usBtnAir);
	Serial.print(F(" H:"));
	Serial.print((int)rc.usBtnHold);
	// Pad with spaces if current line is shorter than previous
	// Emit several spaces and a carriage return next time will overwrite them
	// A simple fixed pad ensures clearing leftovers

	// Append heading information briefly to avoid wrapping
	Serial.print(F(" | H:"));
	Serial.print(g_headingHoldActive ? F("ON ") : F("OFF "));
	if (g_bnoOk)
	{
		float h;
		if (readHeadingDeg(h))
		{
			Serial.print(h, 1);
			if (g_headingHoldActive)
			{
				float err = g_headingTargetDeg - h;
				if (err > 180.0f)
					err -= 360.0f;
				else if (err < -180.0f)
					err += 360.0f;
				Serial.print(F("/"));
				Serial.print(g_headingTargetDeg, 1);
				Serial.print(F(" e="));
				Serial.print(err, 1);
			}
		}
		else
		{
			Serial.print(F("?"));
		}
	}
	// Keep the line short to minimize wrapping; no large fixed padding
}
#endif // DEBUG_ENABLED

// ========= Setup & loop =========
// ---- Serial command parser (minimal skeleton) ----
// Line buffer (small to save RAM). Commands terminated by \n or \r.
static char g_lineBuf[64];
static uint8_t g_lineLen = 0;

// Case-insensitive compare (ASCII); returns 0 if equal.
static int icmp(const char *a, const char *b)
{
	while (*a && *b)
	{
		char ca = *a;
		char cb = *b;
		if (ca >= 'A' && ca <= 'Z')
			ca += 32;
		if (cb >= 'A' && cb <= 'Z')
			cb += 32;
		if (ca != cb)
			return (int)(uint8_t)ca - (int)(uint8_t)cb;
		++a;
		++b;
	}
	return (int)(uint8_t)*a - (int)(uint8_t)*b;
}

// Skip leading spaces; return pointer to first token; also null-terminate after token.
static char *nextToken(char *&cursor)
{
	if (!cursor)
		return nullptr;
	while (*cursor == ' ' || *cursor == '\t')
		++cursor;
	if (*cursor == 0)
		return nullptr;
	char *start = cursor;
	while (*cursor && *cursor != ' ' && *cursor != '\t')
		++cursor;
	if (*cursor)
	{
		*cursor = 0;
		++cursor;
	}
	return start;
}

static void replyOk() { Serial.println(F("OK")); }
static void replyErr(const __FlashStringHelper *msg)
{
	Serial.print(F("ERR: "));
	Serial.println(msg);
}

static void cmdHelp()
{
	Serial.println(F("Commands:"));
	Serial.println(F("  HELP - list commands"));
	Serial.println(F("  VERSION - firmware/api version & git hash"));
	Serial.println(F("  CFG LIST - list all config params"));
	Serial.println(F("  CFG GET <name> - read one param"));
	Serial.println(F("  CFG META <name|ALL|JSON> - parameter schema/constraints"));
	Serial.println(F("  CFG SET <name> <val> - set param (auto-save)"));
	Serial.println(F("  CFG RESET - restore factory defaults"));
	Serial.println(F("  CFG SAVE - force EEPROM save"));
	Serial.println(F("  TELEM STATUS - system status"));
	Serial.println(F("  TELEM RC - raw + mapped RC inputs"));
	Serial.println(F("  TELEM MOTORS - last motor commands"));
	Serial.println(F("  TELEM HEADING - current/target/error"));
	Serial.println(F("  TELEM ALL - combined snapshot"));
	Serial.println(F("  HEAD ON - enable heading hold (capture current)"));
	Serial.println(F("  HEAD OFF - disable heading hold"));
	Serial.println(F("  HEAD SET <deg> - set target to value"));
	Serial.println(F("  HEAD TARGET - report current target"));
	Serial.println(F("  RESET - disarm and neutral outputs"));
}

static void cmdCfgList()
{
	ConfigStore::list(Serial); // prints in format name=value (one per line)
}

static void cmdCfgGet(char *name)
{
	if (!name)
	{
		replyErr(F("missing name"));
		return;
	}
	float v;
	if (ConfigStore::get(name, v))
	{
		Serial.print(F("CFG "));
		Serial.print(name);
		Serial.print(F("="));
		Serial.println(v, 4);
		return;
	}
	replyErr(F("no such param"));
}

static void cmdCfgSet(char *name, char *valTok)
{
	if (!name || !valTok)
	{
		replyErr(F("usage SET <name> <val>"));
		return;
	}
	// Confirm parameter exists first
	float cur;
	if (!ConfigStore::get(name, cur))
	{
		replyErr(F("no such param"));
		return;
	}
	char *endp = nullptr;
	double nv = strtod(valTok, &endp);
	if (endp == valTok || (*endp != 0))
	{
		replyErr(F("bad number"));
		return;
	}
	if (ConfigStore::set(name, (float)nv))
	{
		Serial.print(F("OK "));
		Serial.print(name);
		Serial.print(F("="));
		Serial.println((float)nv, 4);
	}
	else
	{
		replyErr(F("set failed"));
	}
}

static void cmdCfgReset()
{
	ConfigStore::resetDefaults();
	replyOk();
}

static void cmdCfgSave()
{
	if (ConfigStore::save())
		replyOk();
	else
		replyErr(F("save failed"));
}

static void disarm()
{
	g_armed = false;
	g_mode = MODE_DISARMED;
	g_airYawSet = 0;
	g_airThrSet = 0;
	writeEscOutputsUs(1500, 1500);
	g_headingHoldActive = false;
	g_headingModeSpeed = 0;
	g_lastCmdL = 0;
	g_lastCmdR = 0;
	g_lastUsL = 1500;
	g_lastUsR = 1500;
}

static void processLine(char *line)
{
	char *cursor = line;
	char *tok = nextToken(cursor);
	if (!tok)
		return;
	if (icmp(tok, "HELP") == 0)
	{
		cmdHelp();
		return;
	}
	if (icmp(tok, "VERSION") == 0)
	{
		Serial.print(F("VERSION api="));
		Serial.print(F(KC1_API_VERSION));
		Serial.print(F(" git="));
		Serial.print(F(KC1_GIT_HASH));
		Serial.print(F(" build="));
		Serial.print(F(KC1_BUILD_DATE));
		Serial.print(F("T"));
		Serial.print(F(KC1_BUILD_TIME));
		Serial.println();
		return;
	}
	if (icmp(tok, "CFG") == 0)
	{
		char *sub = nextToken(cursor);
		if (!sub)
		{
			replyErr(F("CFG missing sub"));
			return;
		}
		if (icmp(sub, "META") == 0)
		{
			char *pname = nextToken(cursor);
			if (!pname)
			{
				replyErr(F("usage META <name|ALL>"));
				return;
			}
			if (icmp(pname, "JSON") == 0)
			{
				ConfigStore::metaJson(Serial);
				return;
			}
			if (icmp(pname, "ALL") == 0)
			{
				ConfigStore::metaAll(Serial);
				return;
			}
			if (!ConfigStore::metaOne(Serial, pname))
				replyErr(F("no such param"));
			return;
		}
		if (icmp(sub, "LIST") == 0)
		{
			cmdCfgList();
			return;
		}
		if (icmp(sub, "GET") == 0)
		{
			if (icmp(sub, "MOTORS") == 0)
			{
				Serial.print(F("MOTORS cmdL="));
				Serial.print(g_lastCmdL);
				Serial.print(F(" cmdR="));
				Serial.print(g_lastCmdR);
				Serial.print(F(" usL="));
				Serial.print(g_lastUsL);
				Serial.print(F(" usR="));
				Serial.print(g_lastUsR);
				Serial.println();
				return;
			}
			if (icmp(sub, "HEADING") == 0)
			{
				Serial.print(F("HEADING bno="));
				Serial.print(g_bnoOk ? 1 : 0);
				float curH;
				if (g_bnoOk && readHeadingDeg(curH))
				{
					Serial.print(F(" cur="));
					Serial.print(curH, 2);
					Serial.print(F(" hold="));
					Serial.print(g_headingHoldActive ? 1 : 0);
					if (g_headingHoldActive)
					{
						float err = shortestDiffDeg(g_headingTargetDeg, curH);
						Serial.print(F(" tgt="));
						Serial.print(g_headingTargetDeg, 2);
						Serial.print(F(" err="));
						Serial.print(err, 2);
					}
				}
				Serial.println();
				return;
			}
			if (icmp(sub, "ALL") == 0)
			{
				// Combine STATUS + RC + MOTORS + HEADING in a single line for easy logging
				RcInputs rc = readRc();
				Serial.print(F("ALL mode="));
				switch (g_mode)
				{
				case MODE_DISARMED:
					Serial.print(F("DIS"));
					break;
				case MODE_NORMAL:
					Serial.print(F("NRM"));
					break;
				case MODE_AIR:
					Serial.print(F("AIR"));
					break;
				case MODE_HEADING:
					Serial.print(F("HDG"));
					break;
				default:
					Serial.print(F("?"));
					break;
				}
				Serial.print(F(" armed="));
				Serial.print(g_armed ? 1 : 0);
				Serial.print(F(" bno="));
				Serial.print(g_bnoOk ? 1 : 0);
				Serial.print(F(" fsHold="));
				Serial.print(g_holdFailsafe ? 1 : 0);
				Serial.print(F(" yawUs="));
				Serial.print(rc.usYaw);
				Serial.print(F(" thrUs="));
				Serial.print(rc.usThr);
				Serial.print(F(" nUs="));
				Serial.print(rc.usBtnNorm);
				Serial.print(F(" aUs="));
				Serial.print(rc.usBtnAir);
				Serial.print(F(" hUs="));
				Serial.print(rc.usBtnHold);
				Serial.print(F(" yaw="));
				Serial.print(rc.yaw);
				Serial.print(F(" thr="));
				Serial.print(rc.thr);
				Serial.print(F(" cmdL="));
				Serial.print(g_lastCmdL);
				Serial.print(F(" cmdR="));
				Serial.print(g_lastCmdR);
				Serial.print(F(" usL="));
				Serial.print(g_lastUsL);
				Serial.print(F(" usR="));
				Serial.print(g_lastUsR);
				float curH;
				if (g_bnoOk && readHeadingDeg(curH))
				{
					Serial.print(F(" curH="));
					Serial.print(curH, 1);
					if (g_headingHoldActive)
					{
						float err = shortestDiffDeg(g_headingTargetDeg, curH);
						Serial.print(F(" tgtH="));
						Serial.print(g_headingTargetDeg, 1);
						Serial.print(F(" errH="));
						Serial.print(err, 1);
					}
				}
				Serial.println();
				return;
			}
			char *pname = nextToken(cursor);
			cmdCfgGet(pname);
			return;
		}
		if (icmp(sub, "SET") == 0)
		{
			char *pname = nextToken(cursor);
			char *pval = nextToken(cursor);
			cmdCfgSet(pname, pval);
			return;
		}
		if (icmp(sub, "RESET") == 0)
		{
			cmdCfgReset();
			return;
		}
		if (icmp(sub, "SAVE") == 0)
		{
			cmdCfgSave();
			return;
		}
		replyErr(F("bad CFG sub"));
		return;
	}
	if (icmp(tok, "TELEM") == 0)
	{
		char *sub = nextToken(cursor);
		if (!sub)
		{
			replyErr(F("TELEM missing sub"));
			return;
		}
		if (icmp(sub, "STATUS") == 0)
		{
			// Emit one line with key=value pairs
			Serial.print(F("STATUS mode="));
			switch (g_mode)
			{
			case MODE_DISARMED:
				Serial.print(F("DIS"));
				break;
			case MODE_NORMAL:
				Serial.print(F("NRM"));
				break;
			case MODE_AIR:
				Serial.print(F("AIR"));
				break;
			case MODE_HEADING:
				Serial.print(F("HDG"));
				break;
			default:
				Serial.print(F("?"));
				break;
			}
			Serial.print(F(" armed="));
			Serial.print(g_armed ? 1 : 0);
			Serial.print(F(" bno="));
			Serial.print(g_bnoOk ? 1 : 0);
			Serial.print(F(" fsHold="));
			Serial.print(g_holdFailsafe ? 1 : 0);
			Serial.print(F(" cmdL="));
			Serial.print(g_lastCmdL);
			Serial.print(F(" cmdR="));
			Serial.print(g_lastCmdR);
			Serial.print(F(" usL="));
			Serial.print(g_lastUsL);
			Serial.print(F(" usR="));
			Serial.print(g_lastUsR);
			Serial.println();
			return;
		}
		if (icmp(sub, "RC") == 0)
		{
			// Snapshot RC values (raw pulses + mapped)
			RcInputs rc = readRc();
			Serial.print(F("RC yawUs="));
			Serial.print(rc.usYaw);
			Serial.print(F(" thrUs="));
			Serial.print(rc.usThr);
			Serial.print(F(" nUs="));
			Serial.print(rc.usBtnNorm);
			Serial.print(F(" aUs="));
			Serial.print(rc.usBtnAir);
			Serial.print(F(" hUs="));
			Serial.print(rc.usBtnHold);
			Serial.print(F(" yaw="));
			Serial.print(rc.yaw);
			Serial.print(F(" thr="));
			Serial.print(rc.thr);
			Serial.print(F(" valid="));
			Serial.print(rc.valid ? 1 : 0);
			Serial.println();
			return;
		}
		if (icmp(sub, "MOTORS") == 0)
		{
			Serial.print(F("MOTORS cmdL="));
			Serial.print(g_lastCmdL);
			Serial.print(F(" cmdR="));
			Serial.print(g_lastCmdR);
			Serial.print(F(" usL="));
			Serial.print(g_lastUsL);
			Serial.print(F(" usR="));
			Serial.print(g_lastUsR);
			Serial.println();
			return;
		}
		if (icmp(sub, "HEADING") == 0)
		{
			Serial.print(F("HEADING bno="));
			Serial.print(g_bnoOk ? 1 : 0);
			float curH;
			if (g_bnoOk && readHeadingDeg(curH))
			{
				Serial.print(F(" cur="));
				Serial.print(curH, 2);
				Serial.print(F(" hold="));
				Serial.print(g_headingHoldActive ? 1 : 0);
				if (g_headingHoldActive)
				{
					float err = shortestDiffDeg(g_headingTargetDeg, curH);
					Serial.print(F(" tgt="));
					Serial.print(g_headingTargetDeg, 2);
					Serial.print(F(" err="));
					Serial.print(err, 2);
				}
			}
			Serial.println();
			return;
		}
		if (icmp(sub, "ALL") == 0)
		{
			RcInputs rc = readRc();
			Serial.print(F("ALL mode="));
			switch (g_mode)
			{
			case MODE_DISARMED:
				Serial.print(F("DIS"));
				break;
			case MODE_NORMAL:
				Serial.print(F("NRM"));
				break;
			case MODE_AIR:
				Serial.print(F("AIR"));
				break;
			case MODE_HEADING:
				Serial.print(F("HDG"));
				break;
			default:
				Serial.print(F("?"));
				break;
			}
			Serial.print(F(" armed="));
			Serial.print(g_armed ? 1 : 0);
			Serial.print(F(" hold="));
			Serial.print(g_headingHoldActive ? 1 : 0);
			Serial.print(F(" bno="));
			Serial.print(g_bnoOk ? 1 : 0);
			Serial.print(F(" fsHold="));
			Serial.print(g_holdFailsafe ? 1 : 0);
			Serial.print(F(" yawUs="));
			Serial.print(rc.usYaw);
			Serial.print(F(" thrUs="));
			Serial.print(rc.usThr);
			Serial.print(F(" nUs="));
			Serial.print(rc.usBtnNorm);
			Serial.print(F(" aUs="));
			Serial.print(rc.usBtnAir);
			Serial.print(F(" hUs="));
			Serial.print(rc.usBtnHold);
			Serial.print(F(" yaw="));
			Serial.print(rc.yaw);
			Serial.print(F(" thr="));
			Serial.print(rc.thr);
			Serial.print(F(" cmdL="));
			Serial.print(g_lastCmdL);
			Serial.print(F(" cmdR="));
			Serial.print(g_lastCmdR);
			Serial.print(F(" usL="));
			Serial.print(g_lastUsL);
			Serial.print(F(" usR="));
			Serial.print(g_lastUsR);
			float curH;
			if (g_bnoOk && readHeadingDeg(curH))
			{
				Serial.print(F(" curH="));
				Serial.print(curH, 1);
				if (g_headingHoldActive)
				{
					float err = shortestDiffDeg(g_headingTargetDeg, curH);
					Serial.print(F(" tgtH="));
					Serial.print(g_headingTargetDeg, 1);
					Serial.print(F(" errH="));
					Serial.print(err, 1);
				}
			}
			Serial.println();
			return;
		}
		replyErr(F("bad TELEM sub"));
		return;
	}
	if (icmp(tok, "HEAD") == 0)
	{
		char *sub = nextToken(cursor);
		if (!sub)
		{
			replyErr(F("HEAD missing sub"));
			return;
		}
		if (icmp(sub, "ON") == 0)
		{
			if (!ConfigStore::headingHoldEnabled())
			{
				replyErr(F("feature disabled"));
				return;
			}
			if (!g_armed)
			{
				replyErr(F("not armed"));
				return;
			}
			// Capture existing forward speed when entering heading mode.
			// If coming from AIR, use its latched forward setpoint (positive portion only).
			// If from NORMAL, approximate from last motor commands.
			if (g_mode != MODE_HEADING)
			{
				int16_t baseSpeed = 0;
				if (g_mode == MODE_AIR)
				{
					baseSpeed = g_airThrSet; // already -1000..1000
				}
				else if (g_mode == MODE_NORMAL)
				{
					baseSpeed = (int16_t)((g_lastCmdL + g_lastCmdR) / 2);
				}
				if (baseSpeed < 0)
					baseSpeed = 0;
				if (baseSpeed > 1000)
					baseSpeed = 1000;
				g_headingModeSpeed = baseSpeed;
			}
			float h;
			if (g_bnoOk && readHeadingDeg(h))
			{
				g_headingTargetDeg = h;
				g_headingHoldActive = true;
				g_headErrInt = 0.0f;
				g_headPrevErr = 0.0f;
				g_headLastMs = millis();
				g_mode = MODE_HEADING;
				Serial.print(F("HEAD MODE ON tgt="));
				Serial.println(g_headingTargetDeg, 2);
			}
			else
			{
				replyErr(F("no heading"));
			}
			return;
		}
		if (icmp(sub, "OFF") == 0)
		{
			if (g_mode == MODE_HEADING)
			{
				g_headingHoldActive = false;
				g_mode = MODE_NORMAL;	// default return
				g_headingModeSpeed = 0; // zero speed when leaving via command
				replyOk();
			}
			else
			{
				replyErr(F("not in heading"));
			}
			return;
		}
		if (icmp(sub, "SET") == 0)
		{
			char *val = nextToken(cursor);
			if (!val)
			{
				replyErr(F("need deg"));
				return;
			}
			char *endp = nullptr;
			double deg = strtod(val, &endp);
			if (endp == val || (*endp != 0))
			{
				replyErr(F("bad number"));
				return;
			}
			while (deg >= 360.0)
				deg -= 360.0;
			while (deg < 0.0)
				deg += 360.0;
			g_headingTargetDeg = (float)deg;
			g_headErrInt = 0.0f; // reset integrator when externally changed
			Serial.print(F("HEAD TARGET="));
			Serial.println(g_headingTargetDeg, 2);
			return;
		}
		if (icmp(sub, "TARGET") == 0)
		{
			Serial.print(F("HEAD TARGET="));
			Serial.println(g_headingTargetDeg, 2);
			return;
		}
		replyErr(F("bad HEAD sub"));
		return;
	}
	if (icmp(tok, "RESET") == 0)
	{
		// Forward declaration for disarm added above if necessary
		disarm();
		replyOk();
		return;
	}
	replyErr(F("unknown"));
}

// Handle command input from USB Serial only
static void handleSerial()
{
	while (Serial.available())
	{
		int c = Serial.read();
		if (c == '\r' || c == '\n')
		{
			g_lineBuf[g_lineLen] = 0;
			if (g_lineLen > 0)
			{
				processLine(g_lineBuf);
			}
			Serial.print(F("> "));
			g_lineLen = 0;
			continue;
		}
		if (c < 32 || c > 126)
			continue; // skip non-printable
		if (g_lineLen < (sizeof(g_lineBuf) - 1))
		{
			g_lineBuf[g_lineLen++] = (char)c;
		}
	}
}

void setup()
{
	// Serial.begin(115200);
	// Serial.begin(57600);
	Serial.begin(9600);
	delay(100);
	Serial.print(F("KC1 - Kayak Controller (Uno) API v"));
	Serial.println(F(KC1_API_VERSION));
	Serial.print(F("> "));
	// Initialize persistent configuration (loads or creates defaults)
	ConfigStore::begin();
#if DEBUG_ENABLED && (DEBUG_STYLE == 0)
	Serial.println(F("Config params:"));
	ConfigStore::list(Serial);
#endif

	pinMode(CH1_YAW_PIN, INPUT);
	pinMode(CH2_THR_PIN, INPUT);
	pinMode(CH3_NRM_PIN, INPUT);
	pinMode(CH4_AIR_PIN, INPUT);
	pinMode(CH5_HOLD_PIN, INPUT);

	// Initialize ISR state
	g_lastPortD = PIND;
	g_lastPortB = PINB;
	pcint2Enable();
	pcint0Enable();

	// Attach ESCs at 50Hz (Servo uses Timer1)
	escL.attach(ESC_L_PIN, 1000, 2000);
	escR.attach(ESC_R_PIN, 1000, 2000);
	writeEscOutputsUs(1500, 1500); // disarmed neutral

	// Initialize BNO055 (always compiled; runtime usage conditioned by headingHoldEnabled())
	Wire.begin();
	g_bnoOk = bnoBeginAuto();
	if (g_bnoOk)
	{
#if DEBUG_ENABLED && (DEBUG_STYLE == 0)
		Serial.println(F("BNO055 OK"));
#endif
		g_bno.setExtCrystalUse(true);
	}
	else
	{
#if DEBUG_ENABLED && (DEBUG_STYLE == 0)
		Serial.println(F("BNO055 NOT FOUND"));
#endif
	}
}

static void maybeArmOrSwitchMode(const RcInputs &rc)
{
	// Toggle-based event detection on CH3/CH4/CH5 (>500us change counts as a click)
	static bool init = false;
	static uint16_t lastNormUs = 0, lastAirUs = 0, lastHoldUs = 0;
	const uint16_t TOGGLE_DELTA_US = 500; // strictly >500 requested

	if (!init)
	{
		lastNormUs = rc.usBtnNorm;
		lastAirUs = rc.usBtnAir;
		lastHoldUs = rc.usBtnHold;
		init = true;
		return;
	}

	int normDiff = (int)rc.usBtnNorm - (int)lastNormUs;
	int airDiff = (int)rc.usBtnAir - (int)lastAirUs;
	int holdDiff = (int)rc.usBtnHold - (int)lastHoldUs;
	bool normToggled = (normDiff > (int)TOGGLE_DELTA_US) || (normDiff < -(int)TOGGLE_DELTA_US);
	bool airToggled = (airDiff > (int)TOGGLE_DELTA_US) || (airDiff < -(int)TOGGLE_DELTA_US);
	bool holdToggled = (holdDiff > (int)TOGGLE_DELTA_US) || (holdDiff < -(int)TOGGLE_DELTA_US);

	// Update last seen values for next call
	lastNormUs = rc.usBtnNorm;
	lastAirUs = rc.usBtnAir;
	lastHoldUs = rc.usBtnHold;

	// If multiple toggles in the same cycle, ignore to avoid conflicting actions
	if ((normToggled && airToggled) || (normToggled && holdToggled) || (airToggled && holdToggled))
		return;

	bool sticksCentered = (abs(rc.yaw) == 0 && abs(rc.thr) == 0);

	if (!g_armed)
	{
		if (sticksCentered && (normToggled || airToggled))
		{
			g_mode = airToggled ? MODE_AIR : MODE_NORMAL;
			g_armed = true;
			// Latch current (near zero) setpoints for Air mode
			g_airYawSet = rc.yaw;
			g_airThrSet = rc.thr;
#if DEBUG_ENABLED && (DEBUG_STYLE == 0)
			Serial.println(F("ARMED"));
#endif
		}
		return;
	}

	if (airToggled)
	{
		if (g_mode == MODE_AIR)
		{
			// Reset both motors to neutral in Air mode
			g_airYawSet = 0;
			g_airThrSet = 0;
			g_skipAirUpdateOnce = true; // ensure neutral holds for this cycle
#if DEBUG_ENABLED && (DEBUG_STYLE == 0)
			Serial.println(F("AIR -> NEUTRAL"));
#endif
		}
		else
		{
			if (g_mode == MODE_HEADING)
			{
				g_headingHoldActive = false;
				g_headingModeSpeed = 0;
			}
			g_mode = MODE_AIR;
			// Initialize setpoints from current commands to avoid jump
			g_airYawSet = rc.yaw;
			g_airThrSet = rc.thr;
#if DEBUG_ENABLED && (DEBUG_STYLE == 0)
			Serial.println(F("MODE -> AIR"));
#endif
		}
		return;
	}

	if (normToggled)
	{
		if (g_mode != MODE_NORMAL)
		{
			if (g_mode == MODE_HEADING)
			{
				g_headingHoldActive = false;
				g_headingModeSpeed = 0;
			}
			g_mode = MODE_NORMAL;
#if DEBUG_ENABLED && (DEBUG_STYLE == 0)
			Serial.println(F("MODE -> NORMAL"));
#endif
		}
		return;
	}

	if (holdToggled)
	{
		if (!ConfigStore::headingHoldEnabled())
			return; // feature disabled
		if (g_mode != MODE_HEADING && g_armed)
		{
			float h;
			if (readHeadingDeg(h))
			{
				g_headingTargetDeg = h;
				g_headErrInt = 0.0f;
				g_headPrevErr = 0.0f;
				g_headLastMs = millis();
				// Capture existing forward speed when entering heading mode from RC toggle
				int16_t baseSpeed = 0;
				if (g_mode == MODE_AIR)
					baseSpeed = g_airThrSet;
				else if (g_mode == MODE_NORMAL)
					baseSpeed = (int16_t)((g_lastCmdL + g_lastCmdR) / 2);
				if (baseSpeed < 0)
					baseSpeed = 0;
				if (baseSpeed > 1000)
					baseSpeed = 1000;
				g_headingModeSpeed = baseSpeed;
				g_headingHoldActive = true; // active within heading mode
				g_mode = MODE_HEADING;
#if DEBUG_ENABLED && (DEBUG_STYLE == 0)
				Serial.print(F("MODE -> HEADING tgt="));
				Serial.println(g_headingTargetDeg);
#endif
			}
			else
			{
#if DEBUG_ENABLED && (DEBUG_STYLE == 0)
				Serial.println(F("HEADING MODE FAILED (no heading)"));
#endif
			}
		}
		return;
	}
}

void loop()
{
	static uint32_t nextTickMs = 0;
	static uint32_t lastDebugMs = 0;
	static uint8_t invalidCount = 0; // debounce invalid RC

	// Poll serial for incoming command characters
	handleSerial();

	uint32_t nowMs = millis();
	if (nowMs < nextTickMs)
	{
		// Maintain ~20ms cadence
		delay((unsigned long)(nextTickMs - nowMs));
	}
	nextTickMs = millis() + 20; // schedule next

	RcInputs rc = readRc();

	// Failsafe with debounce on invalid RC
	if (!rc.valid)
	{
		if (invalidCount < 3)
			invalidCount++;
		if (invalidCount >= 3)
		{
			if (g_armed)
			{
#if DEBUG_ENABLED && (DEBUG_STYLE == 0)
				Serial.println(F("RC STALE -> DISARM"));
#endif
			}
			disarm();
		}
	}
	else
	{
		invalidCount = 0;
		maybeArmOrSwitchMode(rc);
	}

	int16_t cmdL = 0, cmdR = 0; // -1000..1000

	if (!g_armed)
	{
		writeEscOutputsUs(1500, 1500);
		cmdL = 0; // reflect neutral in debug
		cmdR = 0;
		g_lastCmdL = cmdL;
		g_lastCmdR = cmdR;
		g_lastUsL = 1500;
		g_lastUsR = 1500;
	}
	else
	{
		if (g_holdFailsafe)
		{
			// Hold neutral outputs without changing setpoints
			writeEscOutputsUs(1500, 1500);
			cmdL = 0;
			cmdR = 0;
			g_lastCmdL = cmdL;
			g_lastCmdR = cmdR;
			g_lastUsL = 1500;
			g_lastUsR = 1500;
		}
		else
		{
			switch (g_mode)
			{
			case MODE_NORMAL:
			{
				mixDifferential(rc.thr, rc.yaw, cmdL, cmdR);
				break;
			}
			case MODE_AIR:
			{
				// Update setpoints incrementally from stick deflection
				if (g_skipAirUpdateOnce)
				{
					// Skip one update so neutral takes effect immediately
					g_skipAirUpdateOnce = false;
				}
				else
				{
					updateAirSetpoints(rc.thr, rc.yaw);
				}
				mixDifferential(g_airThrSet, g_airYawSet, cmdL, cmdR);
				break;
			}
			case MODE_HEADING:
			{
				// Incremental forward speed adjustment similar to Air mode but one-dimensional and slower
				int16_t gain = ConfigStore::hdgGainPerCycle();
				if (gain < 0)
					gain = 0;
				// rc.thr is -1000..1000 after deadband; treat positive as accelerate, negative as decelerate
				int32_t step = ((int32_t)rc.thr * gain) / 1000; // could be negative
				int32_t newSpeed = (int32_t)g_headingModeSpeed + step;
				if (newSpeed < 0)
					newSpeed = 0;
				if (newSpeed > 1000)
					newSpeed = 1000;
				g_headingModeSpeed = (int16_t)newSpeed;
				cmdL = g_headingModeSpeed;
				cmdR = g_headingModeSpeed;
				break;
			}
			default:
				cmdL = 0;
				cmdR = 0;
				break;
			}

			// Apply heading-hold correction before expo and reversal
			// Runtime heading hold application
			if (ConfigStore::headingHoldEnabled())
			{
				// In MODE_HEADING we force g_headingHoldActive true; other modes depend on user toggle
				applyHeadingHoldIfNeeded(cmdL, cmdR);
			}

			// Apply output shaping: exponential and optional reversal
			cmdL = applyExpoSigned1000(cmdL, motorExpoL());
			cmdR = applyExpoSigned1000(cmdR, motorExpoR());
			if (reverseLeft())
				cmdL = -cmdL;
			if (reverseRight())
				cmdR = -cmdR;

			// Per-motor start offset & scaling (affects final thrust symmetry). Applied symmetrically.
			// Only apply start offset for small magnitude commands just above zero so motors spin up together.
			int16_t origCmdL = cmdL;
			int16_t origCmdR = cmdR;
			uint16_t startL = cfgMotorStartUsL();
			uint16_t startR = cfgMotorStartUsR();
			float scaleL = cfgMotorScaleL();
			float scaleR = cfgMotorScaleR();
			if (scaleL < 0.5f)
				scaleL = 0.5f;
			else if (scaleL > 1.5f)
				scaleL = 1.5f;
			if (scaleR < 0.5f)
				scaleR = 0.5f;
			else if (scaleR > 1.5f)
				scaleR = 1.5f;
			// Region threshold for applying start offset (command domain)
			int16_t startRegion = (int16_t)cfgMotorStartRegion();
			if (startRegion < 0)
				startRegion = 0;
			else if (startRegion > 400)
				startRegion = 400; // clamp reasonable range
			if (cmdL != 0)
			{
				int16_t mag = abs(cmdL);
				long scaled = (long)mag;
				if (mag < startRegion && startL > 0)
					scaled += startL; // add microsecond-equivalent offset in command space approximation
				// Apply scale after offset approximation
				scaled = (long)(scaled * scaleL);
				if (scaled > 1000)
					scaled = 1000;
				cmdL = (cmdL > 0) ? (int16_t)scaled : (int16_t)(-scaled);
			}
			if (cmdR != 0)
			{
				int16_t mag = abs(cmdR);
				long scaled = (long)mag;
				if (mag < startRegion && startR > 0)
					scaled += startR;
				scaled = (long)(scaled * scaleR);
				if (scaled > 1000)
					scaled = 1000;
				cmdR = (cmdR > 0) ? (int16_t)scaled : (int16_t)(-scaled);
			}

			uint16_t usL = mapSigned1000ToUs(cmdL);
			uint16_t usR = mapSigned1000ToUs(cmdR);
			writeEscOutputsUs(usL, usR);
			g_lastCmdL = cmdL;
			g_lastCmdR = cmdR;
			g_lastUsL = usL;
			g_lastUsR = usR;
		}
	}

#if DEBUG_ENABLED
	// Debug at ~5 Hz
	uint32_t nowDbg = millis();
	if (nowDbg - lastDebugMs >= 200)
	{
		lastDebugMs = nowDbg;
#if DEBUG_STYLE == 0
		printDebugVerbose(rc, g_mode, g_armed, cmdL, cmdR);
#else
		printDebugPseudo(rc, g_mode, g_armed, cmdL, cmdR);
#endif
	}
#endif
}
