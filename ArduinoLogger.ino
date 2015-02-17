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
DateTime bootTimestamp;

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

String formatTimespan(TimeSpan timespan) {
	char buf[16];
	snprintf_P(buf,
	          sizeof(buf),
	          PSTR("%dd %dh %dm %ds"),
	          timespan.days(),
	          timespan.hours(),
	          timespan.minutes(),
	          timespan.seconds());
	return String(buf);
}

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

	typedef enum {
		STOPPED,
		RUNNING
	} datalog_state_t;

	datalog_state_t static state = STOPPED;

	// begin with the data logging 10 seconds after power-on
	static uint32_t triggerTimestamp = 10000;
	const uint32_t period = (uint32_t)DATAPOINT_PERIOD * 1000;

	uint32_t ts = millis();
	if (!dataloggerActive) {
		state = STOPPED;
		triggerTimestamp = ts;
		// start data logging on next tick when dataloggerActive > true
		return;
	}

	if (ts > triggerTimestamp) {

		uint32_t now = clock.now().unixtime();
		switch (state) {
		case STOPPED:
			if (EEPROM_SIZE - eeprom_addr > 4) {
				byte *p = (byte *)&now;
				EEPROM.write(eeprom_addr++, 254); // 254: marker for timestamp
				EEPROM.write(eeprom_addr++, *p++);
				EEPROM.write(eeprom_addr++, *p++);
				EEPROM.write(eeprom_addr++, *p++);
				EEPROM.write(eeprom_addr++, *p++);
			} else dataloggerActive = false; // memory full
			state = RUNNING;
			// fallthrough

		case RUNNING:
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
}

String getTime(DateTime ts, boolean showYear = false) {
	char buf[21];
	// 12.12 12:12:33
	// 0123456789012345
	if (showYear)
		snprintf_P(buf,
		           sizeof(buf),
		           PSTR("%d.%d.%d %02d:%02d:%02d"),
		           ts.day(),
		           ts.month(),
		           ts.year(),
		           ts.hour(),
		           ts.minute(),
		           ts.second());
	else
		snprintf_P(buf,
		           sizeof(buf),
		           PSTR("%d.%d %02d:%02d:%02d"),
		           ts.day(),
		           ts.month(),
		           ts.hour(),
		           ts.minute(),
		           ts.second());
	return String(buf);
}

void static inline processCommand(String cmd) {

	if (cmd == String(F("help"))) {
		Serial.println(F("\
help:    this help screen\n\
settime: set the RTC (Format: 2015-02-16 12:34:56)\n\
stop:    stop the datalogger\n\
start:   (re)start the datalogger\n\
status:  show status infos and uptime in milliseconds\n\
purge:   purge all data in EEPROM\n\
get:     get datalog"));
	} else if (cmd.startsWith(String(F("settime ")))) {
		String param = cmd.substring(cmd.indexOf(' ')+1);
		if (param.length() > 0) {
			int y,m,d,H,M,S;
			if (sscanf_P(param.c_str(),
			             PSTR("%4d-%2d-%4d%2d:%2d:%2d"),
			             &y, &m, &d, &H, &M, &S)) {
				RTC.adjust(DateTime(y,m,d,H,M,S));
				adjustSystemClock();
			}
			else
				Serial.println(F("parse error"));
		} else goto end;
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
		DateTime dateTime;
		for (int a = 0; a < EEPROM_SIZE; a++) {
			byte val = EEPROM.read(a);
			if (val == 255) break;
			if (val == 254) {
				uint32_t ts = 0;
				byte *p = (byte *)&ts;
				*p++ = EEPROM.read(++a);
				*p++ = EEPROM.read(++a);
				*p++ = EEPROM.read(++a);
				*p++ = EEPROM.read(++a);
				dateTime = DateTime(ts);
				continue;
			}
			String data = getTime(dateTime, true) + String(';') + String(val);
			Serial.println(data);
			dateTime = dateTime + TimeSpan(DATAPOINT_PERIOD);
		}
	} else {
		Serial.println(F("Unknown command."));
	}

	goto end;

 status:
	Serial.println(String(F("Current DateTime: ")) + getTime(clock.now(),true));
	Serial.println(String(F("Datalogger status: ")) + (dataloggerActive ? String(F("active")) : String(F("stopped"))));
	Serial.println(String(F("EEPROM usage: ")) + String(eeprom_addr) + String('/') + String(EEPROM_SIZE));
	Serial.println(String(F("Current Brightness value: ")) + String(brightness));
	Serial.println(String(F("Uptime: ")) + formatTimespan(clock.now() - bootTimestamp));
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
			LCDprintf(F("%-16s"), getTime(clock.now()).c_str());
			//LCD.leftToRight();

			triggerTimestamp = millis() + refreshPeriod;
		}
		break;
	}
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

// synchronize the system clock (RTC_Millis) to the RTC chip
void adjustSystemClock() {
	clock.adjust(RTC.now());
}

// every 10 minutes do synchronize the system clock from the RTC clock
// source
void static inline processSysclockSync() {
	const uint32_t period = (uint32_t)10 * 60 * 1000; // every 5 minutes

	static uint32_t triggerTimestamp = period;
	if (millis() > triggerTimestamp) {
		adjustSystemClock();
		triggerTimestamp += period;
	}
}

void setup() {
#ifdef USE_LCD
	setupLcd();
#endif
	Serial.begin(115200);
	Wire.begin();
	RTC.begin();
	if (! RTC.isrunning()) {
		// following line sets the RTC to the date & time this sketch was compiled
		//RTC.adjust(DateTime(__DATE__, __TIME__));
		RTC.adjust(DateTime(__DATE__, __TIME__)+2);
	}
	adjustSystemClock();
	bootTimestamp = clock.now();
	setupDatalog();
}

void loop() {
	processLightSensor();
	processDatalog();
	processSerial();
#ifdef USE_LCD
	processLcd();
#endif
	processSysclockSync();
}
