// -*- mode: C++ -*-
// Goto with remote BT for Astronomical Equatorial stand.
//
// v1.0 - 07/2016 By Sylvain GIRAUD
// For EQ5 Motor Drive stepper motors Tracking + Goto with remote command via Bluetooth.
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

// Requires:
// - Arduino Uno (tested R3 Rev3 ATmega328P CH340G)
// - Adafruit Motor shield v1 with L293D
// - HC-06 Bluetooth for Arduino (must cross cables Tx and Rx)
// - A proper power supply (even if measures shows only 200mA while Tracking + Goto 500pps (9,5V) :-)
// - Check stepper connectivity: positive values run clockwise (actually, this is after gear box)
// - AFMotor library (https://github.com/adafruit/Adafruit-Motor-Shield-library)
// - AccelStepper library v1.2 (https://github.com/adafruit/AccelStepper) (v1.2+ doesn't work, tested NOK with v1.51)
// THIS ONE WORKS WITH AccelStepper 1.2. Issue "2nd moveTo goes 1 PPS only" is fixed (reset all: speed, accel,...etc).
// FAILS WITH AccelStepper 1.51

// Public domain!

// Minor issue 01: when getting serial data, motors freeze a short instant (~0.5 sec)
// Minor issue 02: atfer goto, decelerate slowly (~0.25 sec) before stopping
// Minor issue 03: changing tracking to goto start doing some positive steps (~80) even if goto is negative

#include <AccelStepper12.h>
#include <AFMotor.h>

// ****************** PARAMETERS TO SET ACCORDING TO YOUR STEPPER MOTORS ************************************** 
// MAXI SPEED 525.0 measured with EQ5 MD
// (pps)
#define MAXSPEED 900
// (ppss)
#define ACCELERATION 525.0
// (pps)
#define GOTOSPEED 500.0
// Speeds in steps per sec (sps)
// Higher than 1000 fails with 10.5V/INTERLEAVE
// Tested with SINGLE 400 sps. Get speed x33 compare to tracking. Higher speed fails to run motor.
// (pps)
float gotoSpeed = GOTOSPEED;

// AccelStepper.h: in steps per second
// 20 pps with AccelStepper.h v1.2 and INTERLEAVE looks good for follow-up
// Positive for clockwise
// (pps)
#define TRACKINGSPEED 20.0
// 20 with INTERLEAVE. Steps per sec.
float trackingSpeed = TRACKINGSPEED;

// During goto of Asc Dte need to compute duration to catchup tracking delay
// Considering goto duration = steps / gotoSpeed
// Catchup done in DOUBLE, ie. double of Tracking speed in INTERLEAVE

// Compute time when to release pins after last step. Used for very slow speed to reduce heating of L293D.
// For 20 sps, full interval is 50ms
// Get 56°C max (approx) on L293D with no ReleaseTime afer a short 1 min
// Get 49°C max with 15ms ReleaseTime after 4 min
// Get 37°C max with 05ms ReleaseTime after more than 10 minutes (10% of 1 step duration at 20 sps)
// Formula computes time as percentage of trackingSpeed (/10 gives 10% of 1 step duration)
// (ms)
unsigned long slowTrackingReleaseTime = 1000 / trackingSpeed / 10;
unsigned long timeStep = millis();
bool isTrackingA = true;
bool inhibitTracking = false;

// Compensate mechanical lag:
//     On next negative move, compensation is added to the movement
//     After any negative move (anti-clockwise), do positive steps (so that for any move, we are sure the previous move was always positive)
//     (Assumption yet is that the same number of steps is used for both compensations)
// flag for A anticlockwise goto, must be compensated with 760 steps before tracking works again;
// + Include 80 steps to fix errouneous positive move before A goes anti clockwise (workaround for issue 03)
// Set below number of steps to compensate mechanical lag
// (steps)
#define STEPSCOMPENSATEA 760
unsigned long StepsCompensateA = STEPSCOMPENSATEA;
bool mustCompensateA = false;
// same for D, compensate mechanical lag when direction change
#define STEPSCOMPENSATED 970
unsigned long StepsCompensateD = STEPSCOMPENSATED;
bool mustCompensateD = false;

// approximately 6000 steps is 1 round after gear box (SINGLE/DOUBLE mode)
// (steps)
#define ONEROUND 6000

// Parameters below are used to convert degres to steps
// For eg. theoretical value for 360 deg is 6000 steps * (360/2.5) deg = 864000 steps
// But measures show some gaps. Enter below corrected number of steps for doing 360 deg.
// Steps measured to make 360 degres depending on direction A+ A- D+ D- :
// (steps)
#define STEPS360APOS 848000
#define STEPS360ANEG 851820
#define STEPS360DPOS 855880
#define STEPS360DNEG 855880
unsigned long steps360APos = STEPS360APOS;
unsigned long steps360ANeg = STEPS360ANEG;
unsigned long steps360DPos = STEPS360DPOS;
unsigned long steps360DNeg = STEPS360DNEG;

// Nb of steps when using pad
// (steps)
#define PADSTEPS 100
unsigned long padSteps = PADSTEPS;

// ****************** End of PARAMETERS TO SET ACCODING TO YOUR STEPPER MOTORS ************************************** 


// initial = 200 steps for 1 revolution
// EQ5 stepper = 5958 steps for 1 revolution of EQ5 Molette (not stepper itself!)
// Value seems to have no effect on AccelStepper methods
// Probably used only by AFMotor that sets speed in RPM
// Measured 46 poles for EQ5 MD (20pps, Interleave mode, 1 min, 1200 1/2 steps)
// Motor on port 1 = Asc Dte
AF_Stepper motorA(46, 1);
// Motor on port 2 = Dec
AF_Stepper motorD(46, 2);

// you can change these to SINGLE, DOUBLE or INTERLEAVE or MICROSTEP!
// SINGLE and DOUBLE are more noisy.
void trackingstepA() {  motorA.onestep(FORWARD,  INTERLEAVE);}
void forwardstepA()  {  motorA.onestep(FORWARD,  DOUBLE);}
void backwardstepA() {  motorA.onestep(BACKWARD, DOUBLE);}
// Create 2 steppers A objects, 1 for tracking, 1 for goto to optimize speed
// Goto A
AccelStepper stepperA(forwardstepA, backwardstepA); // use functions to step
// Tracking INTERLEAVE and backward DOUBLE
// Goto 1 round backward :  6000, 12 sec
// Goto 1 round forward  : 12000, 24 sec
AccelStepper trackerA(trackingstepA, backwardstepA); // use functions to step

// you can change these to SINGLE, DOUBLE or INTERLEAVE or MICROSTEP!
// Goto D
void forwardstepD() {  motorD.onestep(FORWARD, DOUBLE);}
void backwardstepD() {  motorD.onestep(BACKWARD, DOUBLE);}
AccelStepper stepperD(forwardstepD, backwardstepD); // use functions to step

long incomingConsA;
long incomingConsD=600;

void resetParam()
{
        gotoSpeed        = GOTOSPEED;
        trackingSpeed    = TRACKINGSPEED;
        slowTrackingReleaseTime = 1000 / trackingSpeed / 10;
        StepsCompensateA = STEPSCOMPENSATEA;
        StepsCompensateD = STEPSCOMPENSATED;        
        steps360APos     = STEPS360APOS;
        steps360ANeg     = STEPS360ANEG;
        steps360DPos     = STEPS360DPOS;
        steps360DNeg     = STEPS360DNEG;
        padSteps         = PADSTEPS;
}

void stopD()
{
      // Must release curent once motor stops otherwise L293 becomes hot. Tested OK.
      // AccelStepper release doesn't prevent heating !!
      //       stepperD.disableOutputs();
      // Works ok with AFMotor release
      motorD.release();
      // Bug fix: reset all otherwise D never restart
      stepperD.setMaxSpeed(MAXSPEED);	
      stepperD.setAcceleration(ACCELERATION);
      stepperD.setSpeed(gotoSpeed);
      stepperD.setCurrentPosition(0);

      stepperD.move(0);
	  
      // Go for another one: NO
      //stepperD.setCurrentPosition(1);
      //stepperD.move(ONEROUND);
      //stepperD.runToNewPosition(-6000);
      //stepperD.run();
}

void stopA()
{
      // Same for Asc Dte before doing Goto
      motorA.release();
      stepperA.setMaxSpeed(MAXSPEED);	
      stepperA.setAcceleration(ACCELERATION);
      stepperA.setSpeed(gotoSpeed);
      stepperA.setCurrentPosition(0);

      stepperA.move(0);

      // Obviously no more tracking in progress
      isTrackingA = false;
}

void trackingA()
{
  // A back to tracking after Goto
  trackerA.setMaxSpeed(MAXSPEED);
  trackerA.setSpeed(trackingSpeed);
  isTrackingA = true;
}


void setup()
{
  // BT
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("EQ5 Goto!");

  resetParam();
  trackingA();
  stopD();
  // Optional: just to check motor works on swich on
  // stepperD.move(incomingConsD);
}

void loop()
{

  // see if there's incoming serial data:
  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    // for some commands assume the entries are available immediately. No wait.
    char incomingCmd = Serial.read();
    //
    switch (incomingCmd) {
      // Commands:
      //    An; : move Asc Dte, degres      +/-, float
      //    Bn; : move Asc Dte, nb of steps +/-, long
      //    Dn; : move     Dec, degres      +/-, float
      //    En; : move     Dec, nb of steps +/-, long
      //    Fn; : set trackingSpeed (steps per seconds, float)
      //    Gn; : set gotoSpeed (steps per seconds, float)
      //    H I J K : Go fixed nb of steps (=padSteps) in all 4 directions A+ A- D+ D- (for pad control). Note: keep tracking state
      //    Q; : Query, are you done with all move?
      //    Rn; : set slowTrackingReleaseTime that release current to prevent L293D heating
      //    Sxn; Set a new value for parameter
      //         x defines which parameter (see switch):
      //            set StepsCompensateA and D (steps to compensate mechnical lags when changing direction)
      //            set steps360APos, steps360ANeg, steps360DPos, steps360ANeg
      //            set padSpeed
      //    T; : Activate tracking (0.25 deg/min) on Asc Dte
      //    U; : De-Activate tracking on Asc Dte
      //    Z; : reset and flush serial
      case 'A':
        // Command for Asc Dte in degres
	// Hope we get a float positive or negative
        {
        float incomingConsDeg = Serial.parseFloat();
        // Convert deg to steps using proper multiplier
        if (incomingConsDeg > 0 ) {
          incomingConsA = steps360APos / 360 * incomingConsDeg;
          } else {
          incomingConsA = steps360ANeg / 360 * incomingConsDeg;
          }
        // Compensate anticlockwise after goto
        if (incomingConsA < 0 ) {
           mustCompensateA = true;
           incomingConsA -= StepsCompensateA;
           }
        // Add catchup of tracking
        if (isTrackingA) {
          // Get goto duration and add nb of tracking steps missed. 
          // Divide by 2 as gotoSpeed is at DOUBLE, while trackingSpeed is INTERLEAVE
          // Note: catchup time is ignored
          incomingConsA += (incomingConsA / gotoSpeed * trackingSpeed / 2);
          }
        stopA();
        // Clockwise or Anticlockwise steps to go
        stepperA.setSpeed(gotoSpeed);
        stepperA.move(incomingConsA);
        Serial.println("Moving Asc Dte");
        break;
        }
      case 'B':
        // Command for Asc Dte in steps
	// Hope we get an integer positive or negative
        incomingConsA = Serial.parseInt();
        // Compensate anticlockwise after goto
        if (incomingConsA < 0 ) {
           mustCompensateA = true;
           incomingConsA -= StepsCompensateA;
           }
        // Add catchup of tracking
        if (isTrackingA) {
          // Get goto duration and add nb of tracking steps missed. 
          // Divide by 2 as gotoSpeed is at DOUBLE, while trackingSpeed is INTERLEAVE
          incomingConsA += (incomingConsA / gotoSpeed * trackingSpeed / 2);
          }
        stopA();
        // Clockwise or Anticlockwise steps to go
        stepperA.setSpeed(gotoSpeed);
        stepperA.move(incomingConsA);
        Serial.println("Moving Asc Dte");
        break;
      case 'D':
        // Command for Dec. in Degres
        // Hope we get a float positive or negative
        {
        float incomingConsDeg = Serial.parseFloat();
        // Convert deg to steps using proper multiplier
        if (incomingConsDeg > 0 ) {
          incomingConsD = steps360DPos / 360 * incomingConsDeg;
          } else {
          // Compensate anticlockwise after goto
          incomingConsD = steps360DNeg / 360 * incomingConsDeg;
          }
        // Compensate anticlockwise after goto
        if (incomingConsD < 0 ) {
           mustCompensateD = true;
           incomingConsD -= StepsCompensateD;
           }
        // Clockwise or Anticlockwise steps to go
        stepperD.move(incomingConsD);
        Serial.println("Moving Dec");
        break;
        }
      case 'E':
        // Command for Dec. in Steps
        // Hope we get an integer positive or negative
        incomingConsD = Serial.parseInt();
        // Compensate anticlockwise after goto
        if (incomingConsD < 0 ) {
           mustCompensateD = true;
           incomingConsD -= StepsCompensateD;
           }
        // Clockwise or Anticlockwise steps to go
        stepperD.move(incomingConsD);
        Serial.println("Moving Dec");
        break;
      case 'F':
        // Set tracking speed
        // Hope we get a positive float
        trackingSpeed = Serial.parseFloat();
        trackerA.setSpeed(trackingSpeed);
        Serial.println("tracking speed set");
        break;
      case 'G':
        // Set goto speed for D
        // Hope we get a positive float
        gotoSpeed = Serial.parseFloat();
        stepperD.setSpeed(gotoSpeed);
        Serial.println("Goto speed set");
        break;

      // H I J K : Fixed steps all 4 directions (for simple pad control)
      // Purely manual, compensation
      case 'H':
        stopA();
        stepperA.move(padSteps);
        break;
      case 'I':
        stopA();
        stepperA.move(-padSteps);
        break;
      case 'J':
        stepperD.move(padSteps);
        break;
      case 'K':
        stepperD.move(-padSteps);
        break;

      case 'Q':
        // Query if target reached and move stopped
        if (( stepperA.distanceToGo() == 0 ) && ( stepperD.distanceToGo() == 0 )) {
          Serial.println("Done");
        } else {
          Serial.println("Moving");
        }
        break;
      case 'R':
        // Set tracking release: time. Use with care !
        // Hope we get a positive int
        slowTrackingReleaseTime = Serial.parseInt();
        Serial.println("tracking release time set");
        break;
      case 'S':
        // Wait so that serial gets the next chars
        delay(200);
        // Set some parameters. Which one:
        {char whichParam = Serial.read();
         switch (whichParam) {
          case 'A':
            // Set both compensate steps StepsCompensateA and D
            // Hope we get positive int
            StepsCompensateA = Serial.parseInt();
            break;
          case 'B':
            StepsCompensateD = Serial.parseInt();
            break;
          case 'C':
            StepsCompensateD = Serial.parseInt();
            break;
          case 'D':
            steps360APos     = Serial.parseInt();
            break;
          case 'E':
            steps360ANeg     = Serial.parseInt();
            break;
          case 'F':
            steps360DPos     = Serial.parseInt();
            break;
          case 'G':
            steps360DNeg     = Serial.parseInt();
            break;
          case 'P':
            padSteps         = Serial.parseInt();
            break;
          default: 
            break;
          }
        } 
        break;
      case 'T':
        // Restore tracking
        inhibitTracking = false;
        stopA();
        trackingA();
        Serial.println("tracking restored");
        break;
      case 'U':
        // Inhibit tracking
        inhibitTracking = true;
        stopA();
        Serial.println("tracking unset");
        break;        
        
      case 'Z':
        // Flush Serial input, reset all, stop all move, still follow up
        // Flush: no more func for that. Anyway switch "default" will flush all inputs.
        resetParam();
        stopD();
        stopA();
        trackingA();
        Serial.println("Reset completed");
        break;
      default:
        Serial.println("Unknown command");
        break;
    }

  }
  else {
    // Nothing from Serial
    // If position reached, stop or reduce speed to tracking

    // Stop D if goto position was reached
    // NOK: if (stepperD.distanceToGo() == 0) {
    // Seems doc in .h is incorrect. Actually return false if target is reached. Some fix in 1.38
    if (stepperD.run() == false) {
      stopD();
      // if goto was anticlockwise, must do 970 steps due to mechanical lag
      if (mustCompensateD) {
        // Force 970 steps
        incomingConsD = StepsCompensateD;
        // Clockwise steps to go
        stepperD.setSpeed(gotoSpeed);
        stepperD.move(incomingConsD);
        mustCompensateD = false;
        }
    }

    // A back to tracking if goto position reached
    if ((isTrackingA == false) && (stepperA.run() == false)) {
      stopA();
      // if goto was anticlockwise, must do 760 steps due to mechanical lag
      if (mustCompensateA) {
        // Force 760 steps
        incomingConsA = StepsCompensateA;
        // Clockwise steps to go
        stepperA.setSpeed(gotoSpeed);
        stepperA.move(incomingConsA);
        mustCompensateA = false;
        }
      else {
        // Redo tracking
        if (inhibitTracking == false ) {trackingA();}
        }
      }
  }

  // For slow speed (tracking) need to disable output to prevent heating
  unsigned long time = millis();
  if ( isTrackingA ) {
          // this command runs the tracking
	  if (trackerA.runSpeed() == true) {
		// a steps occured. Now pins are powered until next step.
		timeStep = time;
	  } else
	  {
		if ( time > (timeStep + slowTrackingReleaseTime) ) {
		  // After slowTrackingReleaseTime, release current
		  motorA.release();
		  // Just to prevent call every loop:
		  timeStep = time;
		  // Next, doesn't work to stop current
		  // stepperA.disableOutputs();
		}
	  }
  }
}
