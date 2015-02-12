LCD Display Wiring
==================

We're using a LCD Display (16x2) from DisplayTech
([Datasheet](http://cdn.displaytech-us.com/sites/default/files/display-data-sheet/162A%20series-v21.pdf)).

This kind of display are using a standard so called
4bit-"Hitachi"-Interface:

Connector
---------

Pin | Name | Arduino Pin
----|------|------------
 1  | VSS  | GND
 2  | VDD  | +5V
 3  | V0   |
 4  | RS   | 12
 5  | R/W  |
 6  | E    | 11
 7  | DB0  |
 8  | DB1  |
 9  | DB2  |
10  | DB3  |
11  | DB4  | 5
12  | DB5  | 4
13  | DB6  | 3
14  | DB7  | 2
15  | LED+ |
16  | LED- |

Background Lit
--------------

The Forward-Voltage for the blue colored low-power version is 3.2V and
the forward current rated at max 30mA.

### The resistor value

R = (5V-3.2V) / 30mA = 60 Ohm. We're choosing
