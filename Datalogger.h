#ifndef Datalogger_h
#define Datalogger_h
#include <inttypes.h>
#include <EEPROM.h>

// EEPROM size
#define EEPROM_SIZE 1024

struct Datapoint {
	uint32_t ts;
	uint8_t val;
};

class Datalogger {

	typedef enum {
		INIT,
		INITIAL_DELAY,
		STOPPED,
		START,
		RUNNING
	} datalog_state_t;

 public:
	Datalogger(uint16_t period);
	void begin(uint8_t *value); // pointer to the variable with the current data to record
	void loop(uint32_t ts); // timestamp in seconds (unixtime)

	uint8_t isRunning() { return state == RUNNING || state == START; }
	int getUsedMemory() { return eeprom_addr; }
	int getMaxMemory() const { return EEPROM_SIZE; }
	void start() { state = START; }
	void stop() { state = STOPPED; }
	void purge();

	void getDataInit();
	uint8_t getDataHasNext(); // more items available?
	struct Datapoint getDataNext(); // get next item from datastore

 protected:
	// current state of the datalogger
	datalog_state_t state;

	// the address of the next free slot in the data buffer. If the memory
	// is full, this will be one = EEPROM_SIZE, which is okay; we do take
	// care of that edge case later in our methods.
	uint16_t eeprom_addr;

	uint8_t *value;

 private:
	uint16_t getData_addr;
	uint16_t period; // time between 2 datapoints

};

#endif
