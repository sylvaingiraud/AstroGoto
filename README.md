# AstroGoto
// Very simple Goto with remote BT for Astronomical Equatorial stand.
// Tested with EQ5 but should works on any stand with "4 cables" stepper motors.
//
// v1.02  - 07/2016 By Sylvain GIRAUD
// v1.03  - 05/2017 By Sylvain GIRAUD - Added calibration with 2 stars.
// For EQ5 Motor Drive stepper motors Tracking + Goto with remote command via Bluetooth.
//    RA  = Asc Dte = AD
//    DEC = Declination
// Do by default Tracking of RA (0,25° per minute, 1 roundtrip in 24h)
// Get commands degres or steps for Asc Dte + Dec from Serial BT HC-06 and do Goto. Can do both simultaneously.
// Optimized speed for tracking and goto (Interleave and Double)
// Backlash: Compensate direction change and mechanical lag for A and D (always ends goto by positive steps)
//    Including compensate mechanical lag when change direction to redo tracking asap: 760steps clockwise if previous goto was anticlockwise
// Correction of conversion degres to steps using actual measures of 360 degres. 2 stars calibration possible.
// Other commands possible, see printHelp();
//    Get help: send "h" to Arduino.
//    Show state and parameters
//    Run a calibration using 2 known stars 
//    Change default values
//    Reset to orginal values and state as on boot
//    Inhibit/Restore Asc Dte tracking
//    Manual moves for control pad
// Fastest speed tested OK: 1 round (6000 steps, DOUBLE) in 12 seconds
// Catchup tracking while goto: missed steps during duration of gotoSpeed
// Do power release forced to prevent L293D heating with slow speed (slowTrackingReleaseTime).
// Not done yet:
//    Spiral search
//    3 stars calibration (currently 2)
//    Backlash calibration (currently manually set)
//    Send An;Dn commands from an application with stars catalogue (currently using a spreadsheet + copy/paste)
//    LX200 protocol !

![Stand](https://github.com/sylvaingiraud/AstroGoto/blob/master/EQ5-Stand-GotoBT.jpg "Stand")
         
## Requires only:
// - Arduino Uno (tested R3 Rev3 ATmega328P CH340G)
// - Adafruit Motor shield v1 with L293D
// - Cable with RJ11 plug to EQ5 MD motors (4 pins)
// - HC-06 Bluetooth for Arduino (must cross cables Tx and Rx) or use USB cable and Terminal application
// - A proper power supply (even if measures shows only 200mA while Tracking + Goto 500pps (9,5V) :-)
// - Check stepper connectivity: positive values run clockwise (actually, this is after gear box)
// - AFMotor library (https://github.com/adafruit/Adafruit-Motor-Shield-library)
// - AccelStepper library v1.2 (https://github.com/adafruit/AccelStepper) (v1.2+ doesn't work, tested NOK with v1.51)
// THIS ONE WORKS WITH AccelStepper 1.2. Issue "2nd moveTo goes 1 PPS only" is fixed (reset all: speed, accel,...etc).
// FAILS WITH AccelStepper 1.51
// Public domain!

## Tests:

// Needs now a mobile App to generate BT commands using a star catalogue (help welcome)
// Can be tested easily with great app [ArduinoRC](https://play.google.com/store/apps/details?id=eu.jahnestacado.arduinorc) (Android) 
