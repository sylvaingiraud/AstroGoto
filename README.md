# AstroGoto

Goto with remote BT for Astronomical Equatorial stand.

v1.01 - 08/2016 By Sylvain GIRAUD

## For EQ5 Motor Drive stepper motors Tracking + Goto with remote command via Bluetooth.

//    Asc Dte = AD = RA

//    Dec = Declination

// Do by default Tracking of Asc Dte (0,25° per minute)

// Get target move degres or steps for Asc Dte + Dec from Serial BT HC-06 and do Goto. Can do both simultaneously.

// Optimized speed for tracking and goto (Interleave and Double)

// Do power release forced to prevent L293D heating with slows speed (tracking).

// Catchup tracking while goto: missed steps during duration of gotoSpeed

// Compensate direction change and mechanical lag for A and D (always ends goto by positive steps)

//    Including compensate mechanical lag when change direction to redo tracking asap: 760steps clockwise if previous goto was anticlockwise
// Correction of conversion degres to steps using actual measures of 360 degres

// Other commands possible, see switch (incomingCmd)

//    Change default values

//    Reset to orginal values and state as on boot

//    Inhibit/Restore Asc Dte tracking.

// Fastest speed tested OK: 1 round (6000 steps, DOUBLE) in 12 seconds

![Stand](https://github.com/sylvaingiraud/AstroGoto/blob/master/EQ5-Stand-GotoBT.jpg "Stand")

## Requires:

// - Arduino Uno (tested R3 Rev3 ATmega328P CH340G)

// - Adafruit Motor shield v1 with L293D

// - HC-06 Bluetooth for Arduino (must cross cables Tx and Rx)

// - A proper power supply (even if measures shows only 200mA while Tracking + Goto 500pps (9,5V) :-)

// - Check stepper connectivity: positive values run clockwise (actually, this is after gear box)

// - AFMotor library (https://github.com/adafruit/Adafruit-Motor-Shield-library)

// - AccelStepper library v1.2 (https://github.com/adafruit/AccelStepper) (v1.2+ doesn't work, tested NOK with v1.51)

## Tests:

// Needs now a mobile App to generate BT commands using a star catalogue (help welcome)

// Can be tested easily with great app [ArduinoRC](https://play.google.com/store/apps/details?id=eu.jahnestacado.arduinorc) (Android) 
