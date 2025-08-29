// Kayak Controller v1 for Arduino Uno (ATmega328P, 16 MHz)
// - Reads 4 RC channels via pin change interrupts (CH1 yaw, CH2 throttle, CH3 Normal button, CH4 Air button)
// - Maps 1000–2000 us to -1000..+1000
// - Modes: Normal (direct mix) and Air (latched setpoints with incremental adjust)
// - Outputs two ESC signals at 50 Hz using Servo (Timer1)
// - Starts Disarmed; requires sticks centered and mode button press to arm
// - 20 ms loop cadence with millis()

#include <Arduino.h>
#include <Servo.h>

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
static int16_t g_airYawSet = 0; // -1000..1000 (latched)
static int16_t g_airThrSet = 0; // -1000..1000 (latched)

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

	bool fresh = true;
	for (uint8_t i = 0; i < 4; ++i)
	{
		if (getSinceUpdateUs(i) > (uint32_t)STALE_TIMEOUT_MS * 1000UL)
		{
			fresh = false;
			break;
		}
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
	// Arm only when sticks centered and a mode button is pressed
	bool sticksCentered = (abs(rc.yaw) == 0 && abs(rc.thr) == 0);
	if (!g_armed)
	{
		if (sticksCentered && (rc.btnNormal || rc.btnAir))
		{
			g_mode = rc.btnAir ? MODE_AIR : MODE_NORMAL;
			g_armed = true;
			// Latch current (near zero) setpoints for Air mode
			g_airYawSet = rc.yaw;
			g_airThrSet = rc.thr;
			Serial.println(F("ARMED"));
		}
	}
	else
	{
		// Allow switching modes on button press
		if (rc.btnAir && g_mode != MODE_AIR)
		{
			g_mode = MODE_AIR;
			// Initialize setpoints from current commands to avoid jump
			g_airYawSet = rc.yaw;
			g_airThrSet = rc.thr;
			Serial.println(F("MODE -> AIR"));
		}
		else if (rc.btnNormal && g_mode != MODE_NORMAL)
		{
			g_mode = MODE_NORMAL;
			Serial.println(F("MODE -> NORMAL"));
		}
	}
}

void loop()
{
	static uint32_t nextTickMs = 0;
	static uint32_t lastDebugMs = 0;

	uint32_t nowMs = millis();
	if (nowMs < nextTickMs)
	{
		// Maintain ~20ms cadence
		delay((unsigned long)(nextTickMs - nowMs));
	}
	nextTickMs = millis() + 20; // schedule next

	RcInputs rc = readRc();

	// Failsafe if RC stale
	if (!rc.valid)
	{
		if (g_armed)
		{
			Serial.println(F("RC STALE -> DISARM"));
		}
		disarm();
	}
	else
	{
		maybeArmOrSwitchMode(rc);
	}

	int16_t cmdL = 0, cmdR = 0; // -1000..1000

	if (!g_armed)
	{
		writeEscOutputsUs(1500, 1500);
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
			updateAirSetpoints(rc.thr, rc.yaw);
			mixDifferential(g_airThrSet, g_airYawSet, cmdL, cmdR);
			break;
		}
		default:
			cmdL = 0;
			cmdR = 0;
			break;
		}

		uint16_t usL = mapSigned1000ToUs(cmdL);
		uint16_t usR = mapSigned1000ToUs(cmdR);
		writeEscOutputsUs(usL, usR);
	}

	// Debug at ~5 Hz
	uint32_t nowDbg = millis();
	if (nowDbg - lastDebugMs >= 200)
	{
		lastDebugMs = nowDbg;
		Serial.print(F("mode="));
		switch (g_mode)
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
		}
		Serial.print(F(" armed="));
		Serial.print(g_armed ? F("1") : F("0"));
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
		// Report latest outputs (read back from last write)
		// For simplicity re-compute when armed else 1000
		if (g_armed)
		{
			int16_t dbgL = 0, dbgR = 0;
			if (g_mode == MODE_NORMAL)
				mixDifferential(rc.thr, rc.yaw, dbgL, dbgR);
			else if (g_mode == MODE_AIR)
				mixDifferential(g_airThrSet, g_airYawSet, dbgL, dbgR);
			Serial.print(F(" | out(us): L="));
			Serial.print((int)mapSigned1000ToUs(dbgL));
			Serial.print(F(" R="));
			Serial.print((int)mapSigned1000ToUs(dbgR));
		}
		else
		{
			Serial.print(F(" | out(us): L=1500 R=1500"));
		}
		Serial.println();
	}
}
