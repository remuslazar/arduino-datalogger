#include "Arduino.h"
#include "version.h"
#include <LiquidCrystal.h>
#include <Wire.h>
#include "RTClib.h"
#include "Datalogger.h"

// LDR sensor input pin
#define SENSOR_PIN A0

// sample rate (in seconds) for the ldr (light sensor)
#define BRIGHTNESS_SAMPLE_RATE 1

// # samples for smoothening the raw input value from sensor
#define BRIGHTNESS_SAMPLES_NUM 5

// seconds between two datapoints (saved in EEPROM)
#define DATAPOINT_PERIOD 120

static uint8_t brightness = 0; // current brightness value (0..255)

static int sensorVal = -1;

RTC_DS1307 RTC;
RTC_Millis clock;
DateTime bootTimestamp;
Datalogger datalog((uint16_t)DATAPOINT_PERIOD);

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

void inline processLightSensor() {
	const int led = LED_BUILTIN;
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

void inline processCommand(String cmd) {

	struct Datapoint first;

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
			if (param.length() > 8 && sscanf_P(param.c_str(),
			             PSTR("%4d-%2d-%4d%2d:%2d:%2d"),
			             &y, &m, &d, &H, &M, &S)) {
				RTC.adjust(DateTime(y,m,d,H,M,S));
				adjustSystemClock();
			} else if (sscanf_P(param.c_str(),
			                    PSTR("2d:%2d:%2d"),
			                    &H, &M, &S)) {
				DateTime now = RTC.now();
				RTC.adjust(DateTime(now.year(),
				                    now.month(),
				                    now.day(),H,M,S));
				adjustSystemClock();
			} else Serial.println(F("parse error"));
		} else goto end;
	} else if (cmd == String(F("status")) || cmd == String("")) {
		goto status;
	} else if (cmd == String(F("stop"))) {
		datalog.stop(); // disable the datalogger
		goto status;
	} else if (cmd == String(F("start"))) {
		datalog.start(); // enable the datalogger
		goto status;
	} else if (cmd == String(F("purge"))) {
		datalog.purge();
		goto status;
	} else if (cmd == String(F("get"))) {
		datalog.getDataInit();
		while (datalog.getDataHasNext()) {
			struct Datapoint data = datalog.getDataNext();
			Serial.println(getTime(DateTime(data.ts), true) + String(';') + String(data.val));
		}
	} else {
		Serial.println(F("Unknown command."));
	}

	goto end;

 status:
	datalog.getDataInit(); first = datalog.getDataNext();
	Serial.println(String(F("Current DateTime: ")) + getTime(clock.now(),true));
	Serial.println(String(F("Datalogger status: ")) + (datalog.isRunning() ? String(F("active")) : String(F("stopped"))));
	Serial.println(String(F("Oldest Datapoint: ")) + getTime(DateTime(first.ts), true));
	Serial.println(String(F("EEPROM usage: ")) + String(datalog.getUsedMemory()) + String('/') + String(datalog.getMaxMemory()));
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

void inline processSerial() {

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
		datalog.stop();
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

void inline processLcd() {
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
			if (last_eeprom_addr != datalog.getUsedMemory()) {
				LCDprintf(F(" %3d%%"),(int)(100.0 * datalog.getUsedMemory()/datalog.getMaxMemory()+.5));
				last_eeprom_addr = datalog.getUsedMemory();
			}

			LCD.setCursor(15,0); // rightmost char
			// write a "play" glyph if the datalogger is active, else a "pause" glyph
			LCD.write(datalog.isRunning() ? LCD_GLYPH_PLAY : LCD_GLYPH_PAUSE);

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

void inline setupLcd() {
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

// then a millis() overflow occurred or else every 10 minutes do
// synchronize the system clock from the RTC clock source
void inline processSysclockSync() {
	const uint32_t period = (uint32_t)10 * 60 * 1000; // every 5 minutes

	static uint32_t lastMillis = 0;
	if (millis() < lastMillis || // millis overflow
	    millis() > lastMillis + period) {
		adjustSystemClock();
		lastMillis = millis();
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
	datalog.begin(&brightness);
}

void loop() {
	// make sure that our clock is fine
	processSysclockSync();

	uint32_t now = clock.now().unixtime();

	processLightSensor();
	datalog.loop(now);
	processSerial();
#ifdef USE_LCD
	processLcd();
#endif
}
