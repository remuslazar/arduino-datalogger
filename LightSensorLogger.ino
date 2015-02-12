#include <EEPROM.h>
#include "version.h"
#include <LiquidCrystal.h>

// wire-up the LCD library accordingly
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// LDR sensor input pin
#define SENSOR_PIN A0

// EEPROM size
#define EEPROM_SIZE 1024

// sample rate (in seconds) for the ldr (light sensor)
#define BRIGHTNESS_SAMPLE_RATE 1

// # samples for smoothening the raw input value from sensor
#define BRIGHTNESS_SAMPLES_NUM 8

// seconds between two datapoints (saved in EEPROM)
#define DATAPOINT_PERIOD 120

static int brightness = -1; // current brightness value (0..255)
static int eeprom_addr = 0;
static bool dataloggerActive = true;
static int sensorVal = -1;

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
		sensorVal = analogRead(SENSOR_PIN);
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
	// call "make version" to update VERSION

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

	typedef enum {
		INIT,
		WAIT_FOR_CMD
	} serial_state_t;

	const short LF = 10; // ASCII linefeed
	static bool isInitialized = false;
	static serial_state_t state = INIT;

	if (!Serial) return;

	if (!isInitialized) {
		// show help screen after initial connect
		Serial.println(F("\n\
Arduino Datalogger Serial Console\n\
=================================\n"));
		Serial.println(String(F("Firmware Version: ")) + String(F(VERSION)) + String('\n') );

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

/**
 * LCD Display state machine
 *
 * Display current status and update it
 * periodically
 */

void static inline processLcd() {

	typedef enum {
		INIT,
		STARTUP,
		UPDATE_LOOP
	} lcd_state_t;

	static lcd_state_t state = INIT;
	const uint32_t startupTimeout = 5 * 1000;
	static uint32_t triggerTimestamp = 0;
	const uint32_t refreshPeriod = 1 * 1000; // in ms

	switch(state) {
	case INIT:
		lcd.setCursor(0,0);
		lcd.print(F("Datalogger"));
		lcd.setCursor(0,1);
		lcd.print(F("Ver: "));
		lcd.print(F(VERSION));
		state = STARTUP;
		break;

	case STARTUP:
		if (millis() > startupTimeout) {
			// init status screen, so we don't always repaint static
			// data in the UPDATE_LOOP
			lcd.setCursor(0,0);

			//          |0123456789012345|
			lcd.print(F("Mem: -          "));
			lcd.setCursor(0,1);
			//          |0123456789012345|
			lcd.print(F("Vd=?    b=?     "));
			state = UPDATE_LOOP;
		}
		break;

	case UPDATE_LOOP:
		if (millis() > triggerTimestamp) {

			// put the live data on the right place

			// LCD line 0
			lcd.setCursor(5,0);
			lcd.print(String(eeprom_addr) + String(F(" (")) +
			          String((float)eeprom_addr*100/EEPROM_SIZE,1) + String(F("%)    ")));

			// LCD line 1
			lcd.setCursor(3,1);
			lcd.print(sensorVal);
			lcd.setCursor(10,1);
			lcd.print(brightness);
			lcd.print(F("    "));

			triggerTimestamp = millis() + refreshPeriod;
		}
		break;
	}

}

void setup() {
	lcd.begin(16, 2);
	Serial.begin(115200);
	setupDatalog();
}

void loop() {
	processLightSensor();
	processDatalog();
	processSerial();
	processLcd();
}
