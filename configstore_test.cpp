// Test ConfigStore::set functionality
// Add this to your main.cpp or create a separate test file

// Add these functions to help debug ConfigStore::set issues:

static void testConfigStore()
{
	Serial.println(F("=== ConfigStore Debug Test ==="));

	// Test 1: Check if ConfigStore is properly initialized
	Serial.print(F("ConfigStore count: "));
	Serial.println(ConfigStore::count());

	// Test 2: List all current parameters to see what's available
	Serial.println(F("Current parameters:"));
	ConfigStore::list(Serial);

	// Test 3: Test a simple parameter that should exist
	float testVal;
	const char *testParam = "dead_center";

	Serial.print(F("Testing parameter: "));
	Serial.println(testParam);

	// Get current value
	if (ConfigStore::get(testParam, testVal))
	{
		Serial.print(F("Current value: "));
		Serial.println(testVal, 4);
	}
	else
	{
		Serial.println(F("ERROR: Failed to get parameter!"));
		return;
	}

	// Test 4: Try to set a new value
	float newVal = testVal + 10.0f; // Change by 10
	Serial.print(F("Attempting to set new value: "));
	Serial.println(newVal, 4);

	bool setResult = ConfigStore::set(testParam, newVal);
	Serial.print(F("Set result: "));
	Serial.println(setResult ? F("SUCCESS") : F("FAILED"));

	// Test 5: Verify the change
	float verifyVal;
	if (ConfigStore::get(testParam, verifyVal))
	{
		Serial.print(F("Verified value: "));
		Serial.println(verifyVal, 4);
		if (abs(verifyVal - newVal) < 0.001f)
		{
			Serial.println(F("Value change CONFIRMED"));
		}
		else
		{
			Serial.println(F("ERROR: Value did not change as expected!"));
		}
	}

	// Test 6: Test parameter constraints (if metadata exists)
	Serial.println(F("Testing parameter metadata:"));
	if (ConfigStore::metaOne(Serial, testParam))
	{
		Serial.println(F("Metadata found"));
	}
	else
	{
		Serial.println(F("No metadata for this parameter"));
	}

	Serial.println(F("=== Test Complete ==="));
}

// Add this more comprehensive test for specific failure scenarios
static void debugConfigSetFailure(const char *paramName, float value)
{
	Serial.print(F("DEBUG: Testing CFG SET "));
	Serial.print(paramName);
	Serial.print(F(" "));
	Serial.println(value, 4);

	// Check if parameter exists
	float currentVal;
	if (!ConfigStore::get(paramName, currentVal))
	{
		Serial.println(F("ERROR: Parameter does not exist"));
		return;
	}

	Serial.print(F("Current value: "));
	Serial.println(currentVal, 4);

	// Check metadata constraints
	Serial.println(F("Parameter constraints:"));
	if (!ConfigStore::metaOne(Serial, paramName))
	{
		Serial.println(F("No metadata found"));
	}

	// Attempt the set operation
	bool result = ConfigStore::set(paramName, value);
	Serial.print(F("Set operation result: "));
	Serial.println(result ? F("true") : F("false"));

	// Check what value was actually set
	float finalVal;
	if (ConfigStore::get(paramName, finalVal))
	{
		Serial.print(F("Final value: "));
		Serial.println(finalVal, 4);

		if (abs(finalVal - currentVal) < 0.001f)
		{
			Serial.println(F("WARNING: Value unchanged - may have been clamped or rejected"));
		}
		else
		{
			Serial.println(F("Value was changed"));
		}
	}
}

// Add this to your existing command handler (around line 1050 in main.cpp)
// Add this case to your processLine function after the existing CFG SET handler:

/*
if (icmp(tok, "TEST") == 0) {
  char *sub = nextToken(cursor);
  if (!sub) {
	replyErr(F("TEST missing sub"));
	return;
  }
  if (icmp(sub, "CONFIG") == 0) {
	testConfigStore();
	return;
  }
  if (icmp(sub, "SET") == 0) {
	char *pname = nextToken(cursor);
	char *pval = nextToken(cursor);
	if (!pname || !pval) {
	  replyErr(F("usage TEST SET <name> <val>"));
	  return;
	}
	char *endp = nullptr;
	double nv = strtod(pval, &endp);
	if (endp == pval || (*endp != 0)) {
	  replyErr(F("bad number"));
	  return;
	}
	debugConfigSetFailure(pname, (float)nv);
	return;
  }
  replyErr(F("bad TEST sub"));
  return;
}
*/