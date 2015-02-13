Arduino Datalogger Sketch
=========================

This is a simple Arduino-based datalogger. The data is saved in the
internal AVR EEPROM memory (1K on an e.g. Arduino Leonardo).

A simple CLI interface over the serial line is available to retrieve
the saved data and set/get status infos.

A LCD-Display (Hitachi 16x2 Interface using the LiquidCrystal library)
can optionally be used to display live values (memory usage,
sensor-data).

How it works
------------

After Power-On, the sketch will start monitoring the data from the
attached sensor and save data samples periodically.

When the internal EEPROM memory is exhausted, the datalogger will be
automatically stopped and no data will be overwritten. Use the
`purge` CLI command to reset the memory.

The maximum duration of a data logging session is limited by the
`DATAPOINT_PERIOD` setting and the size of the internal EEPROM. Using
a 1K EEPROM (Arduino Leonardo) and a period of 2 minutes between the
samples this will lead to about 34h of continuous data logging.

Configuration
-------------

There are a couple of `#define`'s in the main sketch file where
various parameters should be configured. The most important setting is
`DATAPOINT_PERIOD`, which defines the time between the samples (in
seconds) and indirectly the maximum duration of a data logging session.

See the inline documentation for further details.

Serial Console CLI
------------------

Using the serial

Open the serial console (set the speed to 115200bps) and wait for the
CLI prompt. Enter `help` to print out a list of available commands.
