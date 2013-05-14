// Simple throttle
// May 2013 by Ken Shores
// This Arduino sketch (program) is released to the public domain.
//
// This sketch is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
//
// This program implements a simple throttle with a throttle knob, and three switches:
// Run/Stop - In Run, the throttle is live, in Stop the power to the track is disabled after bringing
// any moving trains to a gentle stop.
// EStop/OK - If set to "Estop", the power to the track is immediately cut.
// Forward/Reverse - Sets the direction of motion

#include <TrainMotor.h>

TrainMotor TrainMotor;

// to enable momentum processing:
// - set NO_MOMENTUM false
// - set MIN_THROTTLE to the lowest value that causes the train to move (default 1)
//
#define NO_MOMENTUM false
#define MIN_THROTTLE 161

// microseconds to damp button bounces
const unsigned long BT_BOUNCE_TIME = 20000;

const int MIN_POT = 0;
const int MAX_POT = 1024;

const int throttlePin = A0;
const int runPin = 4;
const int stopPin = 5;
const int directionPin = 2;

boolean runSwitchOn = false;
boolean stopSwitchOn = false;
boolean directionSwitchOn = false;

unsigned long runSwitchWait = 0;
unsigned long stopSwitchWait = 0;
unsigned long directionSwitchWait = 0;

unsigned long avgLoopTime = 0;

int firstRun = true;

int cycles = 0;

// test a button for active status
// This routine will not accept a change from the prior state until the button debounce
// window delay has elapsed without any further changes. Any reversion to the prior state
// during that time will clear the window timer.
//
// Parameters:
// buttonPin - digital pin to read (must already be in INPUT state)
// switchOn - current state of the button - modified when new state accepted
// waitStart - microsecond count of when we entered the window, 0 for not waiting - modified
//
// Although I don't recall for certain, this was probably influenced by:
// http://www.arduino.cc/en/Tutorial/Debounce
// I think mine does a better job though.
void buttonCheck(int buttonPin, boolean& switchOn, unsigned long& waitStart)
{
  boolean nowOn = (digitalRead(buttonPin) == LOW);
  unsigned long readTime = micros();
  
  if ((waitStart > 0) && (nowOn == switchOn)) {
    waitStart = 0; // clear pending change
    int ts = millis();
    Serial.print(ts); Serial.println(": Bounce for "); Serial.println( buttonPin );
  }
  
  if ((nowOn != switchOn) && (waitStart == 0)) { // start pending change
    waitStart = readTime;
  }
  
  // accept pending change as stable
  if ((nowOn != switchOn) && ( (readTime - waitStart) > BT_BOUNCE_TIME )) {
    waitStart = 0;
    switchOn = nowOn;
  }
} // buttonCheck

// read and scale the throttle to 0-255
int readThrottle(int potPin)
{
  int th = constrain ( analogRead(potPin), MIN_POT, MAX_POT );
  
  return( map( th, MIN_POT, MAX_POT, 0, 255) );
} // readThrottle

void setup() {
 
  pinMode(throttlePin, INPUT);
  
  pinMode(runPin, INPUT_PULLUP);
  pinMode(stopPin, INPUT_PULLUP);
  pinMode(directionPin, INPUT_PULLUP);
  
  delay(5000);  // kill some time for the user to open the serial monitor
  Serial.begin(9600);
  while (!Serial);
  
  TrainMotor.motorSetup(TM_ARDUMOTO_MOTOR);  // initialize the motor shield routines
  
  // set accessleration
  // 25 = one second zero to full or full to zero
  // 50 = half-second zero to full or full to zero
  TrainMotor.motorAcc(TM_MOTOR_A, 10, 10);
  TrainMotor.motorAcc(TM_MOTOR_B, 25, 50);   // not used in this sketch
  
  // set the minimum used with momentum
  TrainMotor.motorMinSpeed(TM_MOTOR_A, MIN_THROTTLE);
  
  Serial.print(millis());
  Serial.println(": About to start.");
  
} // setup

void loop() {
  unsigned long loopStart = micros();
  TrainMotor.updateThrottles();
  
  int th = readThrottle(throttlePin);
  
  boolean oldRunOn = runSwitchOn;
  boolean oldStopOn = stopSwitchOn;
  boolean oldDirectionOn = directionSwitchOn;
  buttonCheck(runPin, runSwitchOn, runSwitchWait);
  buttonCheck(stopPin, stopSwitchOn, stopSwitchWait);
  buttonCheck(directionPin, directionSwitchOn, directionSwitchWait);
  
  // report the state of the buttons when they change
  
  if (oldRunOn != runSwitchOn) {
    Serial.print(millis());
    Serial.println(": Run state changed.");
  }
  
  if (oldStopOn != stopSwitchOn) {
    Serial.print(millis());
    Serial.println(": EStop state changed.");
  }
  
  if (oldDirectionOn != directionSwitchOn) {
    Serial.print(millis());
    Serial.println(": Direction changed.");
  }
    
  if (stopSwitchOn && (firstRun || !oldStopOn)) {  // check for emergency stop switch set
    TrainMotor.emergencyStop(true);
    Serial.print(millis());
    Serial.println(": Setting Emergency Stop.");
  } else if ((firstRun || oldStopOn) && !stopSwitchOn) {
    TrainMotor.emergencyStop(false);
    Serial.print(millis());
    Serial.println(": Clearing Emergency Stop.");
  }
  
  if ((firstRun || oldRunOn) && !runSwitchOn) { // parking time
    TrainMotor.throttleA(TM_STOP, NO_MOMENTUM);
    Serial.print(millis());
    Serial.println(": Parking the Train.");
  } else if ((firstRun || runSwitchOn) && !oldRunOn) {
    Serial.print(millis());
    Serial.println(": Starting the Train.");  // actually start is below when throttle changes
  }
  
  if (runSwitchOn) { // we are actually running
    TrainMotor.throttleA(th, NO_MOMENTUM);  // this gets called a lot, and probably could be improved
  }
  
  if (firstRun || (directionSwitchOn != oldDirectionOn)) {
    Serial.print(millis());
    Serial.print(": Changing direction to ");
    Serial.println(directionSwitchOn);
    if (directionSwitchOn) TrainMotor.directionA(TM_EAST);
    else TrainMotor.directionA(TM_WEST);
  }
  
  unsigned long loopEnd = micros();
  avgLoopTime = (loopEnd - loopStart); // its not an average, its the current loop
  if (cycles >= 10000) {
    Serial.print(millis());
    if (runSwitchOn) Serial.print(": Running, ");
    else Serial.print(": Stopped, ");
    if (stopSwitchOn) Serial.print("ESTOP, ");
    if (directionSwitchOn) Serial.print("East, th=");
    else Serial.print("West, th=");
    Serial.println(th);
    Serial.print("Recent loop time is ");Serial.print(avgLoopTime);Serial.println(" microseconds.");
    cycles = 0;
  }
  firstRun = false;
  cycles++;
} // loop
