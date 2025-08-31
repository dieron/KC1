// Kayak Controller v1 for Arduino Uno (ATmega328P, 16 MHz)
// - Reads 4 RC channels via pin change interrupts (CH1 yaw, CH2 throttle, CH3 Normal button, CH4 Air button)
// - Maps 1000–2000 us to -1000..+1000
// - Modes: Normal (direct mix) and Air (latched setpoints with incremental adjust)
// - Outputs two ESC signals at 50 Hz using Servo (Timer1)
// - Starts Disarmed; requires sticks centered and mode button press to arm
// - 20 ms loop cadence with millis()

#include <Arduino.h>
#include <Servo.h>

// ========= Debug configuration =========
// Set DEBUG_ENABLED to 0 to disable all Serial prints at compile time
#ifndef DEBUG_ENABLED
#define DEBUG_ENABLED 0
#endif
// DEBUG_STYLE: 0 = verbose (current), 1 = pseudo-graphics line
#ifndef DEBUG_STYLE
#define DEBUG_STYLE 0
#endif

// ========= Output configuration =========
// Set motor direction reversal (1 = reverse, 0 = normal)
#ifndef REVERSE_LEFT
#define REVERSE_LEFT 0
#endif
#ifndef REVERSE_RIGHT
#define REVERSE_RIGHT 0
#endif

// Exponential curve for motor output, 0..1000 (0 = linear, 1000 = strong expo)
#ifndef MOTOR_EXPO
#define MOTOR_EXPO 400
#endif
#ifndef MOTOR_EXPO_L
#define MOTOR_EXPO_L MOTOR_EXPO
#endif
#ifndef MOTOR_EXPO_R
#define MOTOR_EXPO_R MOTOR_EXPO
#endif

// ========= Failsafe tuning =========
// Enable heuristic: if yaw and throttle stay exactly stable (within 1us) for a long window, treat as TX loss
#ifndef FAILSAFE_STUCK_THR
#define FAILSAFE_STUCK_THR 1
#endif
#ifndef STUCK_DELTA_US
#define STUCK_DELTA_US 1
#endif
#ifndef STUCK_CYCLES
#define STUCK_CYCLES 75 // ~1.5s at 20ms loop
#endif

// ========= Pin assignments =========
// RC input pins (PORTD PCINT2 group: D4..D7)
static const uint8_t CH1_YAW_PIN = 4; // PD4 / PCINT20
static const uint8_t CH2_THR_PIN = 5; // PD5 / PCINT21
static const uint8_t CH3_NRM_PIN = 6; // PD6 / PCINT22
static const uint8_t CH4_AIR_PIN = 7; // PD7 / PCINT23

// ESC outputs (Timer1 capable pins for Servo lib)
static const uint8_t ESC_L_PIN = 9;	 // PB1 / OC1A
static const uint8_t ESC_R_PIN = 10; // PB2 / OC1B

// ========= RC capture (ISR) =========
// Volatile data shared with ISR
volatile uint16_t g_pulseWidthUs[4] = {1500, 1500, 1000, 1000};
volatile uint32_t g_riseTimeUs[4] = {0, 0, 0, 0};
volatile uint32_t g_prevRiseUs[4] = {0, 0, 0, 0};
volatile uint32_t g_lastFrameUs[4] = {0, 0, 0, 0}; // last time we observed a plausible PWM frame period
volatile uint8_t g_lastPortD = 0;
volatile uint32_t g_lastUpdateUs[4] = {0, 0, 0, 0};

// Bit masks for D4..D7
static const uint8_t MASK_CH1 = _BV(PD4);
static const uint8_t MASK_CH2 = _BV(PD5);
static const uint8_t MASK_CH3 = _BV(PD6);
static const uint8_t MASK_CH4 = _BV(PD7);

static inline void pcint2Enable()
{
	// Enable Pin Change Interrupts for PORTD (PCIE2)
	PCICR |= _BV(PCIE2);
	// Enable PCINT for PD4..PD7 (PCINT20..23)
	PCMSK2 |= _BV(PCINT20) | _BV(PCINT21) | _BV(PCINT22) | _BV(PCINT23);
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
		if (changed & mask)
		{
			if (nowD & mask)
			{
				// Rising edge
				g_riseTimeUs[idx] = nowUs;
				// Detect plausible frame period (~50 Hz => ~20 ms). Use a generous range 8-40 ms
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
		}
	};

	handleEdge(MASK_CH1, 0);
	handleEdge(MASK_CH2, 1);
	handleEdge(MASK_CH3, 2);
	handleEdge(MASK_CH4, 3);
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
	MODE_AIR = 2
};

struct RcInputs
{
	uint16_t usYaw;		// CH1
	uint16_t usThr;		// CH2
	uint16_t usBtnNorm; // CH3
	uint16_t usBtnAir;	// CH4
	int16_t yaw;		// -1000..1000
	int16_t thr;		// -1000..1000
	bool btnNormal;		// >1500us
	bool btnAir;		// >1500us
	bool valid;			// all channels fresh
};

// Global state
static Mode g_mode = MODE_DISARMED;
static bool g_armed = false;
static int16_t g_airYawSet = 0;			 // -1000..1000 (latched)
static int16_t g_airThrSet = 0;			 // -1000..1000 (latched)
static bool g_skipAirUpdateOnce = false; // when true, skip one Air update to enforce neutral immediately
static bool g_holdFailsafe = false;		 // constant-signal heuristic: hold motors neutral without disarming

// Tuning
static const int16_t DEAD_CENTER = 50;		  // deadband for center
static const uint16_t STALE_TIMEOUT_MS = 100; // RC failsafe timeout
static const int16_t AIR_GAIN_PER_CYCLE = 40; // max +/- step per 20ms at full deflection

// ESC control
Servo escL, escR;

static RcInputs readRc()
{
	RcInputs r{};
	r.usYaw = getPulseWidthUs(0);
	r.usThr = getPulseWidthUs(1);
	r.usBtnNorm = getPulseWidthUs(2);
	r.usBtnAir = getPulseWidthUs(3);

	r.yaw = applyDeadband(mapUsToSigned1000(r.usYaw), DEAD_CENTER);
	r.thr = applyDeadband(mapUsToSigned1000(r.usThr), DEAD_CENTER);
	r.btnNormal = r.usBtnNorm > 1500;
	r.btnAir = r.usBtnAir > 1500;

	// Validity: require yaw/throttle pulses to be fresh; ignore buttons for validity
	bool fresh = true;
	if (getSinceUpdateUs(0) > (uint32_t)STALE_TIMEOUT_MS * 1000UL)
		fresh = false; // yaw
	if (getSinceUpdateUs(1) > (uint32_t)STALE_TIMEOUT_MS * 1000UL)
		fresh = false; // thr

#if FAILSAFE_STUCK_THR
	// Heuristic: if both yaw and throttle are effectively unchanged for a long time, activate neutral-hold failsafe
	static uint16_t lastThrUs = 0, lastYawUs = 0;
	static uint16_t thrStable = 0, yawStable = 0;
	if (lastThrUs == 0)
	{
		lastThrUs = r.usThr;
	}
	if (lastYawUs == 0)
	{
		lastYawUs = r.usYaw;
	}

	if ((uint16_t)abs((int)r.usThr - (int)lastThrUs) <= STUCK_DELTA_US)
	{
		if (thrStable < 0xFFFF)
			thrStable++;
	}
	else
	{
		thrStable = 0;
	}
	if ((uint16_t)abs((int)r.usYaw - (int)lastYawUs) <= STUCK_DELTA_US)
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

	if (thrStable >= STUCK_CYCLES && yawStable >= STUCK_CYCLES)
	{
		g_holdFailsafe = true;
	}
	else
	{
		g_holdFailsafe = false;
	}
#endif
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
	int32_t stepThr = ((int32_t)inputThr * AIR_GAIN_PER_CYCLE) / 1000;
	int32_t stepYaw = ((int32_t)inputYaw * AIR_GAIN_PER_CYCLE) / 1000;

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
		Serial.print(F("?"));
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
	uint16_t usR = mapSigned1000ToUs(cmdR);
	Serial.print(F(" | out(us): L="));
	Serial.print((int)usL);
	Serial.print(F(" R="));
	Serial.print((int)usR);
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
		else if (v > 0)
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
	Serial.print(F("] "));
	count += 2;

	// We cannot easily count chars inside printMotor; just print, then append raw values and pad.
	printMotor(F("L:"), cmdL);
	Serial.print(F(" "));
	printMotor(F("R:"), cmdR);

	// Append raw channel values for inspection
	Serial.print(F(" | raw(us) Y:"));
	Serial.print((int)rc.usYaw);
	Serial.print(F(" T:"));
	Serial.print((int)rc.usThr);
	Serial.print(F(" N:"));
	Serial.print((int)rc.usBtnNorm);
	Serial.print(F(" A:"));
	Serial.print((int)rc.usBtnAir);
	// Pad with spaces if current line is shorter than previous
	// Emit several spaces and a carriage return next time will overwrite them
	// A simple fixed pad ensures clearing leftovers
	Serial.print(F("                    ")); // 20 spaces padding
}
#endif // DEBUG_ENABLED

// ========= Setup & loop =========
void setup()
{
	Serial.begin(115200);
	delay(100);
	Serial.println(F("KC1 - Kayak Controller (Uno)"));

	pinMode(CH1_YAW_PIN, INPUT);
	pinMode(CH2_THR_PIN, INPUT);
	pinMode(CH3_NRM_PIN, INPUT);
	pinMode(CH4_AIR_PIN, INPUT);

	// Initialize ISR state
	g_lastPortD = PIND;
	pcint2Enable();

	// Attach ESCs at 50Hz (Servo uses Timer1)
	escL.attach(ESC_L_PIN, 1000, 2000);
	escR.attach(ESC_R_PIN, 1000, 2000);
	writeEscOutputsUs(1500, 1500); // disarmed neutral
}

static void disarm()
{
	g_armed = false;
	g_mode = MODE_DISARMED;
	g_airYawSet = 0;
	g_airThrSet = 0;
	writeEscOutputsUs(1500, 1500);
}

static void maybeArmOrSwitchMode(const RcInputs &rc)
{
	// Toggle-based event detection on CH3/CH4 (>500us change counts as a click)
	static bool init = false;
	static uint16_t lastNormUs = 0, lastAirUs = 0;
	const uint16_t TOGGLE_DELTA_US = 500; // strictly >500 requested

	if (!init)
	{
		lastNormUs = rc.usBtnNorm;
		lastAirUs = rc.usBtnAir;
		init = true;
		return;
	}

	int normDiff = (int)rc.usBtnNorm - (int)lastNormUs;
	int airDiff = (int)rc.usBtnAir - (int)lastAirUs;
	bool normToggled = (normDiff > (int)TOGGLE_DELTA_US) || (normDiff < -(int)TOGGLE_DELTA_US);
	bool airToggled = (airDiff > (int)TOGGLE_DELTA_US) || (airDiff < -(int)TOGGLE_DELTA_US);

	// Update last seen values for next call
	lastNormUs = rc.usBtnNorm;
	lastAirUs = rc.usBtnAir;

	// If both toggled in the same cycle, ignore to avoid conflicting actions
	if (normToggled && airToggled)
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
	}
	else
	{
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
				g_mode = MODE_AIR;
				// Initialize setpoints from current commands to avoid jump
				g_airYawSet = rc.yaw;
				g_airThrSet = rc.thr;
#if DEBUG_ENABLED && (DEBUG_STYLE == 0)
				Serial.println(F("MODE -> AIR"));
#endif
			}
		}
		else if (normToggled)
		{
			if (g_mode != MODE_NORMAL)
			{
				g_mode = MODE_NORMAL;
#if DEBUG_ENABLED && (DEBUG_STYLE == 0)
				Serial.println(F("MODE -> NORMAL"));
#endif
			}
		}
	}
}

void loop()
{
	static uint32_t nextTickMs = 0;
	static uint32_t lastDebugMs = 0;
	static uint8_t invalidCount = 0; // debounce invalid RC

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
	}
	else
	{
		if (g_holdFailsafe)
		{
			// Hold neutral outputs without changing setpoints
			writeEscOutputsUs(1500, 1500);
			cmdL = 0;
			cmdR = 0;
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
			default:
				cmdL = 0;
				cmdR = 0;
				break;
			}

			// Apply output shaping: exponential and optional reversal
			cmdL = applyExpoSigned1000(cmdL, MOTOR_EXPO_L);
			cmdR = applyExpoSigned1000(cmdR, MOTOR_EXPO_R);
			if (REVERSE_LEFT)
				cmdL = -cmdL;
			if (REVERSE_RIGHT)
				cmdR = -cmdR;

			uint16_t usL = mapSigned1000ToUs(cmdL);
			uint16_t usR = mapSigned1000ToUs(cmdR);
			writeEscOutputsUs(usL, usR);
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
