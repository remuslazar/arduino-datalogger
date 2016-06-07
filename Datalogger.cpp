#include "Arduino.h"
#include "DataLogger.h"

Datalogger::Datalogger(uint16_t period) {
	this->period = period;
	// search the tail of the datalog buffer (don't overwrite existing
	// data)
	for (eeprom_addr = 0; eeprom_addr < EEPROM_SIZE; eeprom_addr++) {
		uint8_t val = EEPROM.read(eeprom_addr);
		if (val == 0xff) return; // 255 => "no value"
	}
}

void Datalogger::purge() {
	for (int a = 0; a < EEPROM_SIZE; a++) {
		uint8_t val = EEPROM.read(a);
		if (val == 255) break;
		EEPROM.write(a,255);
	}
	eeprom_addr = 0;
}

void Datalogger::getDataInit() {
	getData_addr = 0;
}

uint8_t Datalogger::getDataHasNext() {
	return (getData_addr < eeprom_addr);
}

struct Datapoint Datalogger::getDataNext() {
	struct Datapoint data;
	// increment the last timestamp
	uint8_t val = EEPROM.read(getData_addr++);
	static uint32_t lastTimestamp = 0;

	if (val == 254) {
		uint8_t *p = (uint8_t *)&lastTimestamp;
		for (uint8_t i=0; i<4; i++)
			*p++ = EEPROM.read(getData_addr++);
		lastTimestamp -= period; // we will add the period in the next call
		return getDataNext();
	} else {
		lastTimestamp += period;
		data.val = val;
		data.ts = lastTimestamp;
		return data;
	}
}

void Datalogger::begin(uint8_t *value) {
	this->value = value;
	state = INIT;
}

void Datalogger::loop(uint32_t now) {
	// begin with the data logging 10 seconds after power-on
	const uint32_t initialDelay = (uint32_t)10;
	// interval between 2 recorded datapoints

	static uint32_t triggerTimestamp;

	// datalogger state machine
	switch (state) {
	case INIT:
		triggerTimestamp = now + initialDelay;
		state = INITIAL_DELAY;
		break;

	case INITIAL_DELAY: // initial state
		if (now > triggerTimestamp) {
			state = START;
		}
		break;

	case STOPPED: // datalogger is currently stopped
		break;

	case START: // start the datalogger now (write timestamp)
		if (EEPROM_SIZE - eeprom_addr > 4) {
			uint8_t *p = (uint8_t *)&now;
			EEPROM.write(eeprom_addr++, 254); // 254: marker for timestamp
			for (uint8_t i=0; i<4; i++)
				EEPROM.write(eeprom_addr++, *p++);
			state = RUNNING;
			triggerTimestamp = now; // start now
		} else {
			state = STOPPED; // memory full
			break;
		}
		// no break, fallthrough

	case RUNNING: // datalogger is running
		if (now >= triggerTimestamp) {
			if (eeprom_addr < EEPROM_SIZE) {
				// make sure we don't go beyond the bounds
				EEPROM.write(eeprom_addr++, *value);
				triggerTimestamp += period;
			} else {
				// memory full, stop the datalogger
				state = STOPPED;
			}
		}
		break;
	}
}
