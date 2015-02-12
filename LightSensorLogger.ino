#include <Arduino.h>
#include <EEPROM.h>

#define SENSOR_PIN A0

// EEPROM size
#define EEPROM_SIZE 1024

// sample rate (in seconds) for the ldr (light sensor)
#define BRIGHTNESS_SAMPLE_RATE 5

// # samples for smoothening the raw input value from sensor
#define BRIGHTNESS_SAMPLES_NUM 5

// seconds between two datapoints (saved in EEPROM)
#define DATAPOINT_PERIOD 120

static int brightness = -1; // current brightness value (0..255)
static int eeprom_addr = 0;
static bool dataloggerActive = true;

/**
 * Process the data from the LDR sensor and save the smoothed value in
 * the global variable brightness.
 */

void static inline processLightSensor() {
	const int sampleRate = BRIGHTNESS_SAMPLE_RATE * 1000; // sample freq := 1/sampleRate
	const float smoothSamplesNum = BRIGHTNESS_SAMPLES_NUM;

	static uint32_t triggerTimestamp = 0;
	static double smoothedVal = 500; // start with some middle default value

	if (millis() > triggerTimestamp) {
		int sensorVal = analogRead(SENSOR_PIN);
		smoothedVal += (double)(sensorVal-smoothedVal) / smoothSamplesNum;

		// calculate the brightness value from 0..100
		uint8_t brightnessValue = map(smoothedVal,0,1023,0,254); // 255 means "no value"
		brightness = brightnessValue;

		triggerTimestamp = millis() + sampleRate;
	}
}

// search the tail of the datalog buffer (don't overwrite existing
// data)
void static inline setupDatalog() {
	int a;
	for (a = 0; a < EEPROM_SIZE; a++) {
		byte val = EEPROM.read(a);
		if (val == 0xff) break; // 255 => "no value"
	}
	eeprom_addr = a;
}

void static inline processDatalog() {
	// begin with the data logging 10 seconds after power-on
	static uint32_t triggerTimestamp = 10000;
	const uint32_t period = DATAPOINT_PERIOD;

	if (!dataloggerActive) return;

	uint32_t ts = millis();
	if (ts > triggerTimestamp) {
		EEPROM.write(eeprom_addr, brightness);
		if (++eeprom_addr > 1023) dataloggerActive = false; // no space left on EEPROM
		triggerTimestamp = ts + period * 1000;
	}
}

static const short LF = 10; // ASCII linefeed

void static inline processCommand(String cmd) {

	if (cmd == String(F("help"))) {
		Serial.println(F("\
help:   this help screen\n\
stop:   stop the datalogger\n\
start:  (re)start the datalogger\n\
status: show status infos and uptime in milliseconds\n\
purge : purge all data in EEPROM\n\
get:    get datalog"));
	} else if (cmd == String(F("status")) || cmd == String("")) {
		goto status;
	} else if (cmd == String(F("stop"))) {
		dataloggerActive = false; // disable the datalogger
		goto status;
	} else if (cmd == String(F("start"))) {
		dataloggerActive = true; // enable the datalogger
		goto status;
	} else if (cmd == String(F("purge"))) {
		for (int a = 0; a < EEPROM_SIZE; a++) {
			byte val = EEPROM.read(a);
			if (val == 255) break;
			EEPROM.write(a,255);
		}
		eeprom_addr = 0;
		goto status;
	} else if (cmd == String(F("get"))) {
		for (int a = 0; a < EEPROM_SIZE; a++) {
			byte val = EEPROM.read(a);
			if (val == 255) break;
			String data = String(a) + String(';') + String(val);
			Serial.println(data);
		}
	} else {
		Serial.println(F("Unknown command."));
	}

	goto end;

 status:
	Serial.println(String(F("Datalogger is ")) + (dataloggerActive ? String(F("active")) : String(F("stopped"))));
	Serial.println(String(F("Datapoints count: ")) + String(eeprom_addr));
	Serial.print(F("brightness value: "));Serial.println(brightness);
	Serial.println(String(F("uptime: ")) + String(millis()));

 end:
	Serial.println();
}

/**
 * Console input state machine
 *
 * Display a prompt, read the command from the serial line and
 * dispatch to processCommand()
 */

void static inline processSerial() {

	enum serialState {
		INIT,
		WAIT_FOR_CMD
	};

	static bool isInitialized = false;
	static uint8_t state = INIT;

	if (!Serial) return;

	if (!isInitialized) {
		// show help screen after initial connect
		processCommand(String(F("help")));

		// disable the datalogger
		dataloggerActive = false;
		isInitialized = true;
	}

	switch (state) {
	case INIT:
		Serial.print(F("> Enter command: "));
		state = WAIT_FOR_CMD;
		break;
	case WAIT_FOR_CMD:
		if (Serial.available() > 0) {
			String cmd = Serial.readStringUntil(LF);
			Serial.println(cmd+String('\n'));
			processCommand(cmd);
			state = INIT;
		}
		break;
	}
}

void setup() {
	Serial.begin(115200);
	setupDatalog();
}

void loop() {
	processLightSensor();
	processDatalog();
	processSerial();
}
