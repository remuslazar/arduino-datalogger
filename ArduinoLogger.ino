#include <EEPROM.h>
#include "version.h"
#include <LiquidCrystal.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include "RTClib.h"

// LDR sensor input pin
#define SENSOR_PIN A0

// EEPROM size
#define EEPROM_SIZE 1024

// sample rate (in seconds) for the ldr (light sensor)
#define BRIGHTNESS_SAMPLE_RATE 1

// # samples for smoothening the raw input value from sensor
#define BRIGHTNESS_SAMPLES_NUM 5

// seconds between two datapoints (saved in EEPROM)
#define DATAPOINT_PERIOD 120

static int brightness = -1; // current brightness value (0..255)

// the address of the next free slot in the data buffer. If the memory
// is full, this will be one = EEPROM_SIZE, which is okay; we do take
// care of that edge case later in our methods.
static int eeprom_addr = 0;
static bool dataloggerActive = true;
static int sensorVal = -1;

RTC_DS1307 RTC;
RTC_Millis clock;

#define USE_LCD

#ifdef USE_LCD
// wire-up the LCD library accordingly
LiquidCrystal LCD(12, 11, 7, 6, 5, 4);
void LCDprintf(const __FlashStringHelper *format, ...) {
	char buf[17]; // 16 chars (one line) buffer for vsnprintf, incl. the trailing zero
	va_list ap;
	va_start(ap, format);
	vsnprintf_P(buf, sizeof(buf), (const char *)format, ap); // progmem for AVR
	for(char *p = &buf[0]; *p; p++) {
		LCD.write(*p);
	}
	va_end(ap);
}
#endif

/**
 * Process the data from the LDR sensor and save the smoothed value in
 * the global variable brightness.
 */

void static inline processLightSensor() {
	const int sampleRate = BRIGHTNESS_SAMPLE_RATE * 1000; // sample freq := 1/sampleRate
	const float smoothSamplesNum = BRIGHTNESS_SAMPLES_NUM;

	static uint32_t triggerTimestamp = 0;
	static float smoothedVal = -1;

	if (millis() > triggerTimestamp) {
		sensorVal = analogRead(SENSOR_PIN);
		if (smoothedVal == -1) smoothedVal = sensorVal;
		else smoothedVal += ((float)sensorVal-smoothedVal) / smoothSamplesNum;

		// calculate the brightness value from 0..253
		uint8_t brightnessValue = (uint8_t)(smoothedVal * 253 / 1023 + .5);
		brightness = brightnessValue;

		triggerTimestamp += sampleRate;
	}
}

// search the tail of the datalog buffer (don't overwrite existing
// data)

void static inline setupDatalog() {
	for (eeprom_addr = 0; eeprom_addr < EEPROM_SIZE; eeprom_addr++) {
		byte val = EEPROM.read(eeprom_addr);
		if (val == 0xff) return; // 255 => "no value"
	}
	// memory full, eeprom_addr = EEPROM_SIZE
}

void static inline processDatalog() {
	// begin with the data logging 10 seconds after power-on
	static uint32_t triggerTimestamp = 10000;
	const uint32_t period = (uint32_t)DATAPOINT_PERIOD * 1000;

	uint32_t ts = millis();
	if (!dataloggerActive) {
		triggerTimestamp = ts;
		// start data logging on next tick when dataloggerActive > true
		return;
	}

	if (ts > triggerTimestamp) {
		if (eeprom_addr < EEPROM_SIZE) {
			// make sure we don't go beyond the bounds
			EEPROM.write(eeprom_addr++, brightness);
			triggerTimestamp += period;
		} else {
			// memory full, stop the datalogger
			dataloggerActive = false;
		}
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
	Serial.println(String(F("brightness value: ")) + String(brightness));
	Serial.println(String(F("uptime: ")) + getTime());
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

#ifdef USE_LCD

#define LCD_GLYPH_PLAY 5
#define LCD_GLYPH_PAUSE 6

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
	const uint32_t startupTimeout = 5 * 1000L;
	static uint32_t triggerTimestamp = 0;
	const uint32_t refreshPeriod = 1 * 1000L; // in ms

	switch(state) {
	case INIT:
		LCD.setCursor(0,0);
		LCD.print(F("Datalogger"));
		LCD.setCursor(0,1);
		LCD.print(F( "Ver: " VERSION ));
		state = STARTUP;
		break;

	case STARTUP:
		if (millis() > startupTimeout) {
			LCD.clear();
			// init status screen, so we don't always repaint static
			// data in the UPDATE_LOOP
			//LCD.setCursor(0,1);
			//          |0123456789012345|
			//printf_P(PSTR("Up:"));
			//LCD.print(F("Up:"));
			state = UPDATE_LOOP;
		}
		break;

	case UPDATE_LOOP:
		if (millis() > triggerTimestamp) {

			// put the live data on the right place for performance
			// reasons we don't want to clear the screen prior to the
			// updates. We just write the data on the right spot on
			// the LCD and make sure to overwrite the longest posible
			// width.

			// 1. row: display Vd raw value, brightness and memory usage

			LCD.setCursor(0,0);LCDprintf(F("%-4d %-3d"), sensorVal, brightness);

			// only update memory usage when needed (minor performance enhancement)
			static int last_eeprom_addr = -1;
			if (last_eeprom_addr != eeprom_addr) {
				LCDprintf(F(" %3d%%"),(int)((float)eeprom_addr*100.0/EEPROM_SIZE+.5));
				last_eeprom_addr = eeprom_addr;
			}

			LCD.setCursor(15,0); // rightmost char
			// write a "play" glyph if the datalogger is active, else a "pause" glyph
			LCD.write(dataloggerActive ? LCD_GLYPH_PLAY : LCD_GLYPH_PAUSE);

			// 2. row: display the uptime in seconds

			// setup the offsets and lengths for the various values as constants
			LCD.setCursor(0,1);
			//LCD.rightToLeft();
			LCDprintf(F("%-16s"), getTime().c_str());
			//LCD.leftToRight();

			triggerTimestamp = millis() + refreshPeriod;
		}
		break;
	}
}

String getTime() {
	DateTime now = clock.now();
	char buf[16];
	// 12.12 12:12:33
	// 0123456789012345
	snprintf_P(buf,
	          sizeof(buf),
	          PSTR("%d.%d %02d:%02d:%02d"),
	          now.day(),
	          now.month(),
	          now.hour(),
	          now.minute(),
	          now.second());
	return String(buf);
}

void static setupLcd() {
	LCD.begin(16, 2);

#ifdef LCD_BARS
	// setup some glyph (5x7) for the vu-meter display
	byte buf[8];
	byte c = 0x00;
	for (byte i=0; i<5; i++) {
		c |= (1 << i);
		memset(buf,c,sizeof(buf));
		LCD.createChar(i, buf);
	}
#endif
	byte glyph[2][8] = {
		{ // play
			B01000,
			B01100,
			B01110,
			B01111,
			B01110,
			B01100,
			B01000,
		}, { // pause
			B11011,
			B11011,
			B11011,
			B11011,
			B11011,
			B11011,
			B11011,
			B11011,
		}
	};
	LCD.createChar(LCD_GLYPH_PLAY, glyph[0]);
	LCD.createChar(LCD_GLYPH_PAUSE, glyph[1]);
	// one glyph left

}
#endif

void setup() {
#ifdef USE_LCD
	setupLcd();
#endif
	Serial.begin(115200);
	//RTC.begin();
	/* if (! RTC.isrunning()) { */
	/* 	Serial.println("RTC is NOT running!"); */
	/* 	// following line sets the RTC to the date & time this sketch was compiled */
	/* 	//RTC.adjust(DateTime(__DATE__, __TIME__)); */
	/* } */
	clock.adjust(DateTime(__DATE__, __TIME__) + 20);
	setupDatalog();
}

void loop() {
	processLightSensor();
	processDatalog();
	processSerial();
#ifdef USE_LCD
	processLcd();
#endif
}
