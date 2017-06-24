// -*- mode: C++ -*-
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

// Requires only:
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

// Minor issue 01: when getting serial data, motors freeze a short instant (~0.5 sec)
// Minor issue 02: atfer goto, decelerate slowly (~0.25 sec) before stopping

// MUST be AccelStepper v1.2
#include <AccelStepper.h>
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
// flag for A anticlockwise goto, must be compensated with 700 steps before tracking works again;
// Set below number of steps to compensate mechanical lag
// (steps)
// Rested 180 instead of 660 !!
#define STEPSCOMPENSATEA 180
unsigned long StepsCompensateA = STEPSCOMPENSATEA;
bool mustCompensateA = false;
// same for D, compensate mechanical lag when direction change
// tested adrien 660 instead of 970 !!
// tested on wall: need much comp. otherwise 1st reverse is less than 2nd. Anyway compensation causes;
// with small values of  10 steps get Up 1110 and Dpwn 860 
// with       values of 100 steps get Up  600 and Down 700
// 600 looks fine
#define STEPSCOMPENSATED 650
unsigned long StepsCompensateD = STEPSCOMPENSATED;
bool mustCompensateD = false;

// Compensation mode (only 1st mode is coded yet)
#define STEPSCOMPENSATEPOS    0        // After any negative move do a positive move to wedge mechanics 
#define STEPSCOMPENSATEBOTH   1        // No additional move: need to store previous move to decide additional compensation before next move
                                       // This mode is not used&coded because tracking requires the 1st mode
bool StepsCompensateMode = STEPSCOMPENSATEPOS;

// Store last move direction
//#define LASTMOVEPOS   0
//#define LASTMOVENEG   1
//bool LastMoveA = LASTMOVEPOS;
//bool LastMoveD = LASTMOVEPOS;

// approximately 6000 steps is 1 round after gear box (SINGLE/DOUBLE mode)
// (steps)
#define ONEROUND 6000

// Parameters below are used to convert degres to steps
// For eg. theoretical value for 360 deg is 6000 steps * (360/2.5) deg = 864000 steps
// But measures show some gaps. Enter below corrected number of steps for doing 360 deg.
// Steps measured to make 360 degres depending on direction A+ A- D+ D- :
// (steps)UU
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

// Calibration phases
// At this stage, use same value for for STEPS360XNEG and XPOS. Otherwise will need calibration with other stars and move in different direction 
#define CALOFF         0 // No calibration performed or even in progress
#define CALSTAR1       1 // This is the 1st step of Calibration
                         // Star1 is pointed, user decides to do a calibration: Star1 coord=0, steps=0 and phase is set to this value.
#define CALSTAR2GO     2 // User decides the Star2. Set this phase and do a Goto. Star2 relative coord&steps are stored.
#define CALSTAR2FIX    3 // Set this phase and then the user fixes the position manually. Star2Steps positions are updated on every move.
#define CALSTAR2DONE   4 // Set this phase when the fix is completed. Star2 is correctly pointed.
                         // STEPS360xxxx are recomputed with coord and fixed steps :
                         //           STEPS360Xxxx = 360 * [(stepfixed(Star2) - step(Star1)] / [coord(Star2)-coord(Star1)]
                         // Note: different sign for coord and steps ! Star1 values are all 0 because we use relative moves.
#define CALDONE        9 // Calibration performed and completed. STEPS360Xxxx have been fixed.

byte CalMode = CALOFF;
float Star2A = 0;
float Star2D = 0;
int   Star2Asteps = 0;
int   Star2Dsteps = 0;

// More info trace to serial
bool Verbose = false;

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

void printParam()
{
        Serial.print("gotoSpeed=");Serial.println(gotoSpeed);
        Serial.print("trackingSpeed=");Serial.println(trackingSpeed);
        Serial.print("slowTrackingReleaseTime=");Serial.println(slowTrackingReleaseTime);
        Serial.print("StepsCompensateA=");Serial.println(StepsCompensateA);
        Serial.print("StepsCompensateD=");Serial.println(StepsCompensateD);
        Serial.print("steps360APos=");Serial.println(steps360APos);
        Serial.print("steps360ANeg=");Serial.println(steps360ANeg);
        Serial.print("steps360DPos=");Serial.println(steps360DPos);
        Serial.print("steps360DNeg=");Serial.println(steps360DNeg);
        Serial.print("padSteps=");Serial.println(padSteps);
}

void printHelp(char h)
{
  switch(h)
  {
  case 'a':
    Serial.println("Command:");
    Serial.println("An; : move RA  (deg)  n=+/-float");
    Serial.println("Bn; : move RA  (step) n=+/-long");
    Serial.println("Dn; : move DEC (deg)  n=+/-float");
    Serial.println("En; : move DEC (step) n=+/-long");
    Serial.println("Cx  Set Calibration steps. C0 to C4. Ch for help.");
    //    Serial.println("      For each phase, proceed with usual commands with goto+manual fix");
    //    Serial.println("      Once fixed, last step recomputes the calibration");
    Serial.println("Fn; : set trackingSpeed (steps/sec) n=float.20pps for 24h roundtrip");
    Serial.println("Gn; : set gotoSpeed     (steps/sec) n=float.Depends on motor performance");
    Serial.println("H J U N : For A+ A- D+ D- respectively in all 4 directions padSteps value");
    //    Serial.println("          These are the letters to set in control pad (For eg. in ArduinoRC Android application)");
    Serial.println("Q;  : All move done? If Verbose, shows all param");
    Serial.println("Rn; : (Expert)Set slowTrackingReleaseTime to release current to prevent L293D heating");
    Serial.println("Sxn;: (Expert)Set a parameter. n=int");
    Serial.println("   A/B: set backlash RA and DEC");
    Serial.println("   D/E/F/G: set steps360APos, _ANeg, _DPos, _ANeg");
    Serial.println("   P: set padSpeed");
    Serial.println("  x defines which parameter x=char. Values are:");
    //    Serial.println("   A: StepsCompensateA - backlash correction value for RA.");
    //    Serial.println("   B: StepsCompensateD - backlash correction value for DEC.");
    //    Serial.println("   D: steps360APos");
    //    Serial.println("   E: steps360ANeg");
    //    Serial.println("   F: steps360DPos");
    //    Serial.println("   G: steps360DNeg");
    //    Serial.println("   P: padSteps");
    Serial.println("T+; T-; : RA tracking (0.25 deg/min)");
    Serial.println("V+; V-; : Verbose");
    Serial.println("Z; : Reset");
    Serial.println("h; : Help");
    break;
  case '0': Serial.println("  C0: Calib. cancelled"); break;
  case '1': Serial.println("  C1: Star1 fixed. Send C2 before Goto to Star2"); break;
  case '2': Serial.println("  C2: Do now a Goto to Star2 with An and Dn. Then send C3"); break;
  case '3': Serial.println("  C3: Proceed now with manual fix. Use pad or send H J U N. Then send C4"); break;
  case '4': Serial.println("  C4: Calib. done"); break;
  default:
    break;
  }
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
  Serial.println("EQ5 Goto! h for help");

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
      // Commands: see printHelp() for all commands.
      case 'A':
        // Command for Asc Dte in degres
	      // Hope we get a float positive or negative
        {
        float incomingConsDeg = Serial.parseFloat();
        // Convert deg to steps using proper multiplier
        // Positive deg do anticlockwise steps, and vice versa
        if (incomingConsDeg > 0 ) {
          incomingConsA = -(steps360APos / 360 * incomingConsDeg);
          } else {
          incomingConsA = -(steps360ANeg / 360 * incomingConsDeg);
          }
        // Are we in Calibration Goto phase ?
        if (CalMode == CALSTAR2GO) { Star2A = incomingConsDeg; Star2Asteps = incomingConsA; }

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
        stepperA.move(incomingConsA);
        Serial.print("RA: Deg/Step");
        Serial.print(incomingConsDeg);
        Serial.print("/");
        Serial.println(incomingConsA);
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
        stepperA.move(incomingConsA);
        Serial.println("RA");
        break;
      case 'C':
        // Wait so that serial gets the next chars
        delay(200);
        // Set some parameters. Which one:
        {char whichParam = Serial.read();
         switch (whichParam) {
          case '0':
            // C0: Any Calib in progress is cancelled
            CalMode = CALOFF;    
            printHelp('0');
            break;
          case '1':
            // C1: The Star1 is pointed. Send C1.
            CalMode = CALSTAR1;
            Star2A = 0;
            Star2D = 0;
            Star2Asteps = 0;
            Star2Dsteps = 0;
            // Nothing more to do. Star 1 coord&steps=0.
            printHelp('1');
            break; 
          case '2':
            // C2: After sending C2, next command is a goto to Star2
            CalMode = CALSTAR2GO;  // From now on, all moves must update steps
            // From now on, next "Cons" will be stored as relative moves to Star2
            // There should be only 1 Deg "Cons" move
            printHelp('2');
            break;
          case '3':
            // C3: The Goto is done. Before manual adjustment, send C3.
            CalMode = CALSTAR2FIX;
            // All manual steps must update Star2 steps
            printHelp('3');
            break;
          case '4':
            // C4: The Star2 is pointed correctly, send C4.
            CalMode = CALSTAR2DONE;
            // Compensation can be computed here:
            // (Note: steps and deg have different signs)
            steps360APos = -360.0 * Star2Asteps / Star2A;
            steps360ANeg = steps360APos;
            steps360DPos = -360.0 * Star2Dsteps / Star2D;
            steps360DNeg = steps360DPos;
            CalMode = CALDONE;
            printHelp('4');
            if (Verbose) {
              Serial.print("Star2A=");Serial.println(Star2A);
              Serial.print("Star2Asteps=");Serial.println(Star2Asteps);
              Serial.print("Star2D=");Serial.println(Star2D);
              Serial.print("Star2Dsteps=");Serial.println(Star2Dsteps);
              printParam(); }
            break;
          default: 
            printHelp('0');
            printHelp('1');
            printHelp('2');
            printHelp('3');
            printHelp('4');
            break;
         }}
      case 'D':
        // Command for Dec. in Degres
        // Hope we get a float positive or negative
        {
        float incomingConsDeg = Serial.parseFloat();
        // Convert deg to steps using proper multiplier
        if (incomingConsDeg > 0 ) {
          incomingConsD = -(steps360DPos / 360 * incomingConsDeg);
          } else {
          // Compensate anticlockwise after goto
          incomingConsD = -(steps360DNeg / 360 * incomingConsDeg);
          }
        // Are we in Calibration Goto phase ?
        if (CalMode == CALSTAR2GO) { Star2D = incomingConsDeg; Star2Dsteps = incomingConsD; }

        // Compensate anticlockwise after goto
        if (incomingConsD < 0 ) {
           mustCompensateD = true;
           incomingConsD -= StepsCompensateD;
           }
        // Clockwise or Anticlockwise steps to go
        stepperD.move(incomingConsD);
        Serial.print("Moving Dec: Deg/Step ");
        Serial.print(incomingConsDeg);
        Serial.print("/");
        Serial.println(incomingConsD);
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

      // U H J N : Fixed steps all 4 directions (for simple pad control)
      // Purely manual, compensation
      case 'J':
        // Go East : Positive steps => will do Negative degs
        stopA();
        stepperA.move(padSteps);
        // Are we in Calibration FIX phase ?
        if (CalMode == CALSTAR2FIX) { Star2Asteps += padSteps; }
        break;
      case 'H':
        // Go West : Negative steps => will do Positive degs
        stopA();
        // Compensate anticlockwise before and after goto
        stepperA.move(-(padSteps + StepsCompensateA));
        mustCompensateA = true;
        // Are we in Calibration FIX phase ?
        if (CalMode == CALSTAR2FIX) { Star2Asteps -= padSteps; }
        break;
      case 'N':
        // Go Down : Positive steps => will do Negative degs
        stepperD.move(padSteps);
        // Are we in Calibration FIX phase ?
        if (CalMode == CALSTAR2FIX) { Star2Dsteps += padSteps; }
        break;
      case 'U':
        // Go Up : Negative steps => will do Positive degs
        // Compensate anticlockwise before and after goto
        stepperD.move(-(padSteps + StepsCompensateD));
        mustCompensateD = true;
        // Are we in Calibration FIX phase ?
        if (CalMode == CALSTAR2FIX) { Star2Dsteps -= padSteps; }
        break;

      case 'Q':
        // Query if target reached and move stopped
        if (( stepperA.distanceToGo() == 0 ) && ( stepperD.distanceToGo() == 0 )) {
          Serial.println("Done");
        } else {
          Serial.println("Moving");
        }
        if (Verbose) { printParam(); }
        break;
      case 'R':
        // Set tracking release: time. Use with care !
        // Hope we get a positive int
        slowTrackingReleaseTime = Serial.parseInt();
        Serial.println("release time set");
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
        Serial.print("set:");
        Serial.println(whichParam);
        } 
        break;
      case 'T':
        // Wait so that serial gets the next chars
        delay(200);
        // Set some parameters. Which one:
        {char whichParam = Serial.read();
         switch (whichParam) {
          case '+':
   		     // Restore tracking
             inhibitTracking = false;
             stopA();
             trackingA();
             Serial.println("tracking set");
             break;
          case '-':
             // Inhibit tracking
             inhibitTracking = true;
             stopA();
             Serial.println("tracking unset");
             break;
          default:
             // Switch tracking
             inhibitTracking != inhibitTracking;
             Serial.println("tracking swap");
             break;		  
		  }
          }
        break;
      case 'V':
        // More/Less feedback to serial
        delay(200);
        // Set some parameters. Which one:
        {char whichParam = Serial.read();
         switch (whichParam) {
          case '+':
              Verbose = true;
	      Serial.println("verbose set");
              break;
          case '-':
              Verbose = false;
              break;
          default:
             break;		  
		  }
		}
        break;
      case 'Z':
        // Flush Serial input, reset all, stop all move, still follow up
        // Flush: no more func for that. Anyway switch "default" will flush all inputs.
        resetParam();
        stopD();
        stopA();
        trackingA();
        Serial.println("Reset");
        break;
      case 'h':
        printHelp('a');
        break;
      case ';':
        break;
      default:
        Serial.println("h for help");
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
