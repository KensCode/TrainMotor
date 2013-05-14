/*  TrainMotor.cpp
    by Ken Shores
    Version a1 - initial alpha release
    A library for controlling model trains using a motor shield.
    
    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 
    Released into the public domain.
*/

#include "Arduino.h"
#include "TrainMotor.h"

// private variables for our own use
int _a_pwm = 0;
int _b_pwm = 0;
int _a_dir1 = 0;
int _a_dir2 = 0; // not used on some shields
int _b_dir1 = 0;
int _b_dir2 = 0; // not used on some shields

int _shield; // what shield we are using
long _skew_time; // how many msec between throttle updates
float _skew_mod = 1.0; // timer alteration (default = none)
int _throttle_a; // user-specified throttle 
int _speed_a;    // actual current speed based on momentum 
int _throttle_b;
int _speed_b;
int _accel_a;  // how many points (on 0-255 scale) to move throttle A up each _skew_time
int _accel_b;  // and ditto for B
int _deccel_a; // same for moving throttle A down (braking rate)
int _deccel_b; // ditto for B
int _dir_a;    // which direction is the train going right now
int _dir_b;
int _new_dir_a; // which direction is the train going to be going after reversing
int _new_dir_b;
int _min_a = TM_STOP;    // minimum speed needed to move a locomotive
int _min_b = TM_STOP;
boolean _braking_a; // true when speed is being reduced
boolean _braking_b;
boolean _reversing_a; // true when we are going to change direction once stopped
boolean _reversing_b;
boolean _eStop = false; // if true, ignore all throttle commands

unsigned long _last_time; // time last throttle update performed

// class constructor - runs before setup to initialize an instance of the class
// Hardware and global data structures may not be initialized when this is run, put 
// anything like a write in the setup routine instead of here.
TrainMotor::TrainMotor()
{
  // do nothing here beyond constructing the object or initializing simple variables
} // TrainMotor constructor

// provide the user with a safe delay function that calls updateThrottles
// every 20 milliseconds to see if an udpate is needed
void TrainMotor::activeDelay(int msec)
{
  int wt;
  
  #if defined(TM_STATUS)
    Serial.print(millis()); Serial.print(": activeDelay called");
    Serial.print(", msec=");Serial.println(msec);
  #endif
  
  if (msec < 1) return; // returns in about 8 microseconds if delay is 0 or negative
  unsigned long delayStart = millis();
  int waitTime = TM_MIN_UPDATE;
  if (msec < waitTime) waitTime = msec;
  
  do {
    delay(waitTime);
    updateThrottles(); // make sure we update often enough
    wt = msec - int( millis() - delayStart );
    waitTime = min( wt, TM_MIN_UPDATE);
  } while (waitTime > 0);
} // activeDelay

// Internal - Setup the timers for the selected prescale and type of PWM
void TrainMotor::_pwmSetup()
{
  // Set up the timers
  
  int prescale = TM_PRESCALE;
  if (( prescale < 1) || (prescale > 5)) prescale = 1; // ensure 0-7 range
  #if defined(TM_TM_STATUS_VERBOSE)
    Serial.print("Prescale is ");Serial.print(prescale);
    if (TM_FAST_PWM == 1) Serial.println(", Fast PWM");
          else Serial.println(", Phase-Correct PWM");
  #endif
  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328P__)
      #if defined(TM_STATUS_VERBOSE)
        Serial.println("In _pwmSetup - Arduino 168/328 family (Uno).");
      #endif
      switch (_shield) {
        case TM_ARDUINO_MOTOR:
        case TM_ARDUMOTO_MOTOR:
          TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
      
          if (TM_FAST_PWM == 1) TCCR2B = _BV(WGM22) | prescale; // Fast PWM
          else TCCR2B = prescale; // Phase-correct PWM
          
          #if defined(TM_STATUS_VERBOSE)
            Serial.print("Arduino/Ardumoto - TCCR2A is "); Serial.println(TCCR2A, BIN);
          #endif
        break;
     
        case TM_SEEED_MOTOR:
          // Set Motor A (pin 9) to use Timer1A and Motor B (pin 10) to use Timer1B
          TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
      
          if (TM_FAST_PWM == 1) TCCR1B = _BV(WGM12) | prescale; // Fast PWM
          else TCCR1B = prescale; // Phase-correct PWM
          
          #if defined(TM_STATUS_VERBOSE)
            Serial.print("SEEED - TCCR1A is "); Serial.println(TCCR1A, BIN);
          #endif
        break;
        
        case TM_DFROBOT_MOTOR: // unsupported combination, just ignore here
          // Set Motor A (pin 6) to use Timer0A and Motor B (pin5) to use Timer0B

          //TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
      
          //if (TM_FAST_PWM == 1) TCCR0B = _BV(WGM02) | prescale; // Fast PWM
          //else TCCR0B = prescale; // Phase-correct PWM
          
          #if defined(TM_STATUS_VERBOSE)
            Serial.print("DFRobot - TCCR0A is "); Serial.println(TCCR0A, BIN);
            Serial.print("DFRobot - TCCR0B is "); Serial.println(TCCR0B, BIN);
          #endif
        break;
      }
      
      
  #elif defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
      #if defined(TM_STATUS_VERBOSE)
        Serial.println("In _pwmSetup - Arduino Mega family.");
      #endif
      switch (_shield) {
        case TM_ARDUINO_MOTOR:
        case TM_ARDUMOTO_MOTOR:
          // Set Motor A (pin 3) to use Timer3C
          TCCR3A = _BV(COM3C1) |_BV(WGM30);
      
          if (TM_FAST_PWM == 1) TCCR3B = _BV(WGM32) | prescale; // Fast PWM
          else TCCR3B = prescale; // Phase-correct PWM
    
          // Set Motor B (pin 11) to use Timer1A
          TCCR1A = _BV(COM1A1) | _BV(WGM10);
      
          if (TM_FAST_PWM == 1) TCCR1B = _BV(WGM12) | prescale; // Fast PWM
          else TCCR1B = prescale; // Phase-correct PWM
          
          #if defined(TM_STATUS_VERBOSE)
            Serial.print("Arduino/Ardumoto - TCCR3A is "); Serial.println(TCCR3A, BIN);
            Serial.print("Arduino/Ardumoto - TCCR1A is "); Serial.println(TCCR1A, BIN);
          #endif
        break;
      
        case TM_SEEED_MOTOR:
          // Set Motor A (pin 9) to use Timer2B and Motor B (pin 10) to use Timer2A
          TCCR2A = _BV(COM2A1) | _BV(COM2B1) |_BV(WGM20);
      
          if (TM_FAST_PWM == 1) TCCR2B = _BV(WGM22) | prescale; // Fast PWM
          else TCCR2B = prescale; // Phase-correct PWM
          
          #if defined(TM_STATUS_VERBOSE)
            Serial.print("SEEED - TCCR2A is "); Serial.println(TCCR2A, BIN);
          #endif
        break;
        
        case TM_DFROBOT_MOTOR: 
          // Set Motor A (pin 6) to use Timer4A and Motor B (pin 5) to use Timer3A
          
          TCCR4A = _BV(COM4A1) |_BV(WGM40);
          
          if (TM_FAST_PWM == 1) TCCR4B = _BV(WGM42) | prescale; // Fast PWM
          else TCCR4B = prescale; // Phase-correct PWM
    
          TCCR3A = _BV(COM3A1) | _BV(WGM30);
      
          if (TM_FAST_PWM == 1) TCCR3B = _BV(WGM32) | prescale; // Fast PWM
          else TCCR3B = prescale; // Phase-correct PWM
          
          #if defined(TM_STATUS_VERBOSE)
            Serial.print("DFRobot - TCCR4A is "); Serial.println(TCCR4A, BIN);
            Serial.print("DFRobot - TCCR3A is "); Serial.println(TCCR3A, BIN);
          #endif
        break;
      }
  #elif defined(__AVR_ATmega32U4__)
      #if defined(TM_STATUS_VERBOSE)
        Serial.print("In _pwmSetup - Leonardo family, _shield is ");
        Serial.println(_shield);
      #endif
      
      switch (_shield) {
        case TM_ARDUINO_MOTOR:
        case TM_ARDUMOTO_MOTOR: // unsupported combination, just ignore here
          // Since this uses Timer0, changes will affect behavior of delay function and
          // other system timers.  

          // Set Motor A (pin 3) to use Timer0B, Motor B (pin 11) to use Timer0A
          //TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
      
          //if (TM_FAST_PWM == 1) TCCR0B = _BV(WGM02) | prescale; // Fast PWM
          //else TCCR0B = prescale; // Phase-correct PWM
          
          // Adjust _skew_time to account for the different number of ticks per millisecond
           //if (TM_FAST_PWM == 1) _skew_time =
           
           // by default, the Leonardo timer0 is set up to run in phase-correct 10-bit,
           // with a prescale of 3. Changing this to 8-bit should make it run 4x the
           // speed. 
           // But it does not. Or rather, delay now apparently returns immediately rather
           // that waiting, as millis now counts 2 msec for a delay of 3000.
           /*
           switch (prescale) {
             case 1: // x64 timer
               _skew_mod = 64;
             break;
             
             case 2: // x8 timer
               _skew_mod = 8;
             break;
             
             case 3: // default - timer should run normally
               _skew_mod = 1;
             break;
             
             case 4: // divide by four
               _skew_mod = (1/4);
             break;
             
             case 5: // divide by sixteen
               _skew_mod = (1/16);
             break;
           } */
           #if defined(TM_STATUS_VERBOSE)
             Serial.print("Arduino/Ardumoto - TCCR0A is "); Serial.println(TCCR0A, BIN);
             Serial.print("Arduino/Ardumoto - TCCR0B is "); Serial.println(TCCR0B, BIN);
             Serial.print("Arduino/Ardumoto - _skew_time is "); Serial.println(_skew_time);
             Serial.print("Arduino/Ardumoto - _skew_mod is "); Serial.println(_skew_mod);
           #endif
        break;
        
        case TM_SEEED_MOTOR:
          // Set Motor A (pin 9) to use Timer1A and Motor B (pin 10) to use Timer1B
          TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
      
          if (TM_FAST_PWM == 1) TCCR1B = _BV(WGM12) | prescale; // Fast PWM
          else TCCR1B = prescale; // Phase-correct PWM
          
          #if defined(TM_STATUS_VERBOSE)
            Serial.print("SEEED - TCCR1A is "); Serial.println(TCCR1A, BIN);
          #endif
        break;
        
        case TM_DFROBOT_MOTOR: // this code needs to be finished
          // Set Motor A (pin 6) to use Timer4D and Motor B (pin 5) to use Timer3B
          
          // need to fix 4D definition
          //TCCR4C = _BV(COM4D1S) |_BV(WGM40);
          
          //if (TM_FAST_PWM == 1) TCCR4B = _BV(WGM42) | prescale; // Fast PWM
          //else TCCR4B = prescale; // Phase-correct PWM
    
          TCCR3A = _BV(COM3B1) | _BV(WGM30);
      
          if (TM_FAST_PWM == 1) TCCR3B = _BV(WGM32) | prescale; // Fast PWM
          else TCCR3B = prescale; // Phase-correct PWM
          
          #if defined(TM_STATUS_VERBOSE)
            Serial.print("DFRobot - TCCR4A is "); Serial.println(TCCR4A, BIN);
            Serial.print("DFRobot - TCCR3A is "); Serial.println(TCCR3A, BIN);
          #endif
        break;
      }
  #else
  	#error Unsupported Arduino board type for TrainMotor library.
  #endif
} // _pwmSetup

// Internal - change the motor speed
// it does not validate the motor speed is in 0-255 as other routines have done that,
// but it forces any out of bounds values to be clipped to the bounds.
// it must be customized for each Arduino model, as the OCRs used will vary by model.
// Returns: true if motor is valid for this model Arduino (ignored execpt in setup),
// otherwise false and the OCR will not be altered.
boolean TrainMotor::_setMotor(int which_motor, int speed)
{
  int th;
  boolean goodMotor = true;
  
  #if defined(TM_DEBUG)
    Serial.print("_setMotor called, which="); Serial.print(which_motor);
    Serial.print(", speed="); Serial.print(speed);
    Serial.print(", time is "); Serial.println(millis());
  #endif
 
  int spd = constrain(speed, TM_STOP, TM_FULL);  // ensure in bounds
  
  // scale actual shield setting into the range between MIN-1 and 255, stop is always zero
  if (speed != TM_STOP) {
    if (which_motor == TM_MOTOR_A) th = int( map( spd, TM_STOP, TM_FULL, (_min_a - 1), 255) );
    else th = int( map( spd, TM_STOP, TM_FULL, (_min_b - 1), 255) );
  } else th = TM_STOP;

   #if defined(TM_STATUS_VERBOSE)
    Serial.print(millis());
    Serial.print(": _setMotor called, which="); Serial.print(which_motor);
    Serial.print(", speed="); Serial.print(speed);
    Serial.print(", spd="); Serial.print(spd);
    Serial.print(", setting th="); Serial.println(th);
  #endif

  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328P__)
    if (which_motor == TM_MOTOR_A)
    {
      switch (_shield) {
        case TM_ARDUINO_MOTOR:
        case TM_ARDUMOTO_MOTOR:
          OCR2B = th; // Timer2B uses OCR2B
        break;
        
        case TM_SEEED_MOTOR:
          OCR1A = th;
        break;
        
        case TM_DFROBOT_MOTOR: // warning - using timer0
          //OCR0A = th;
          goodMotor = false;
        break;
      }
    } else {
      switch (_shield) {
        case TM_ARDUINO_MOTOR:
        case TM_ARDUMOTO_MOTOR:
          OCR2A = th; // Timer2A uses OCR2A
        break;
        
        case TM_SEEED_MOTOR:
          OCR1B = th;
        break;
        
        case TM_DFROBOT_MOTOR: // warning - using timer0
          //OCR0B = th;
          goodMotor = false;
        break;
      }
    }
  #elif defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
    if (which_motor == TM_MOTOR_A)
    {
      switch (_shield) {
        case TM_ARDUINO_MOTOR:
        case TM_ARDUMOTO_MOTOR:
          OCR3C = th; // Timer3A uses OCR3C (dont ask me why)
        break;
        
        case TM_SEEED_MOTOR:
          OCR2B = th;
        break;
        
        case TM_DFROBOT_MOTOR:
          OCR4A = th;
        break;
      }
    } else {
      switch (_shield) {
        case TM_ARDUINO_MOTOR:
        case TM_ARDUMOTO_MOTOR:
          OCR1A = th; // Timer1A uses OCR1A
        break;
        
        case TM_SEEED_MOTOR:
          OCR2A = th;
        break;
        
        case TM_DFROBOT_MOTOR:
          OCR4B = th;
        break;
      }
    }
  #elif defined(__AVR_ATmega32U4__)
    if (which_motor == TM_MOTOR_A)
    {
      switch (_shield) {
        case TM_ARDUINO_MOTOR: // warning - using timer0
        case TM_ARDUMOTO_MOTOR:
          //OCR0B = th; // Timer0B uses OCR0B (untested)
          goodMotor = false;
        break;
        
        case TM_SEEED_MOTOR: 
          OCR1A = th;
        break;
        
        case TM_DFROBOT_MOTOR:
          OCR4D = th;
        break;
      }
    } else {
      switch (_shield) {
        case TM_ARDUINO_MOTOR: // warning - using timer0
        case TM_ARDUMOTO_MOTOR:
          //OCR0A = th; // Timer0A uses OCR0A (untested)
          goodMotor = false;
        break;
        
        case TM_SEEED_MOTOR: 
          OCR1B = th;
        break;
        
        case TM_DFROBOT_MOTOR:
          OCR3B = th;
        break;
      }
    }
  #endif
  
  return( goodMotor );
} // _setMotor

// calculate the adjustment over time msec to apply and limit it to 0-255
int TrainMotor::_getAdj(int acc, int msec)
{
  return( constrain((int( (float(acc)/TM_SKEW_INTERVAL)*(float)msec )), TM_STOP, TM_FULL) );
} // _getAdj

// This checks to see if enough time has passed, and if so will increment/decrement the 
// motor speed until it reaches the desired throttle setting.
// Note: _skew_time is in timer ticks, presently this equals milliseconds although it 
// may not in future versions of the library.
void TrainMotor::updateThrottles()
{
  unsigned long this_time = millis();  // current time in milliseconds from start
  long interval =  this_time - _last_time; // msec since we last did an update
  //long interval_msec = (long)(interval * _skew_mod);
  long interval_msec = interval;
  int adj_a;  
  int adj_b;
  int old_a = _speed_a; // remember these to know if something changed
  int old_b = _speed_b;
  int th_a = _throttle_a; // make a copy so we can change it here without altering original
  int th_b = _throttle_b;

  #if defined(TM_STATUS_VERBOSE)
    Serial.print(millis()); Serial.println(": updateThrottles called");
  #endif
  
  #if defined(TM_DEBUG)
    Serial.print("Entering updateThrottles");
    Serial.print(", _dir_a="); Serial.print(_dir_a);
    Serial.print(", _dir_b="); Serial.print(_dir_b);
    Serial.print(", interval_msec="); Serial.print(interval_msec);
    Serial.print(", _skew_time="); Serial.print(_skew_time);
    Serial.print(", _skew_mod="); Serial.print(_skew_mod);
    Serial.print(", time="); Serial.println(millis());
    Serial.print("  this_time="); Serial.print(this_time);
    Serial.print("  _last_time="); Serial.println(_last_time);
  #endif

  if (interval_msec > _skew_time) { // if enough time has gone by, adjust things
  
    #if defined(TM_DEBUG)
      Serial.print("Skew time elapsed, time is "); Serial.print(millis());
      if (_braking_a) Serial.print(", BrakesA");
      if (_braking_b) Serial.print(", BrakesB");
      if (_reversing_a) Serial.print(", ReversingA");
      if (_reversing_b) Serial.print(", ReversingB");
      Serial.print(",  interval is "); Serial.print(interval);
      Serial.print(",  interval_msec is "); Serial.print(interval_msec);
      Serial.print(",  _skew_time is ");Serial.println(_skew_time);
      Serial.print("  old_a="); Serial.print(old_a);Serial.print(", old_b="); Serial.print(old_b);
      Serial.print(",  th_a="); Serial.print(_throttle_a);Serial.print(", th_b="); Serial.print(_throttle_b);
      Serial.print(",  sp_a="); Serial.print(_speed_a);Serial.print(", sp_b="); Serial.println(_speed_b);
    #endif
    
    // if the brakes are on, treat the throttle as if set to 0
    if (_braking_a) {
      if (_speed_a > TM_STOP) th_a = TM_STOP;
      else {
        #if defined(TM_STATUS_VERBOSE)
        Serial.print(millis());
        Serial.print(": Clear braking(1), _throttle_a is "); 
        Serial.print(_throttle_a);Serial.print(", _speed_a is "); Serial.println(_speed_a);
      #endif     
      
        _braking_a = false;
        _reversing_a = false;
        _setDirA(_new_dir_a);
      }
    } // braking_a
    if (_braking_b) {
      if (_speed_b > TM_STOP) th_b = TM_STOP;
      else {
        _braking_b = false;
        _reversing_b = false;
        _setDirB(_new_dir_b);
      }
    } // braking_b
      
    // if the (adjusted) throttle is above current speed, accelerate
    if (th_a > _speed_a) {
      adj_a = _getAdj(_accel_a, interval_msec);
      int tspa = _speed_a;
       _speed_a = constrain(_speed_a + adj_a, TM_STOP, _throttle_a);
      
      #if defined(TM_STATUS_VERBOSE)
        Serial.print("Acc: adj_a is "); Serial.print(adj_a);Serial.print(", _throttle_a is "); 
        Serial.print(_throttle_a);
        Serial.print(", tspa is "); Serial.print(tspa);
        Serial.print(", _speed_a is "); Serial.println(_speed_a);
      #endif
      
    // if the (adjusted) throttle is blow the current speed, slow down
    } else if (th_a < _speed_a) {
      adj_a = _getAdj(_deccel_a, interval_msec);
      int tspd = _speed_a;
      _speed_a = constrain(_speed_a - adj_a, TM_STOP, TM_FULL);
      
      #if defined(TM_STATUS_VERBOSE)
        Serial.print("Dec: adj_a is "); Serial.print(adj_a);Serial.print(", _throttle_a is "); 
        Serial.print(_throttle_a);
        Serial.print(", tspd is "); Serial.print(tspd);
        Serial.print(", _speed_a is "); Serial.println(_speed_a);
      #endif
    
    // if the brakes were on but we werent reversing and now speed and throttle match, release the brakes
    } else if (_braking_a && !_reversing_a) {
      _braking_a = false; // done
      
       #if defined(TM_STATUS_VERBOSE)
        Serial.print(millis());
        Serial.print(": Clear braking(2), _throttle_a is "); 
        Serial.print(_throttle_a);Serial.print(", _speed_a is "); Serial.println(_speed_a);
      #endif
     
    }
    
    // if the (adjusted) throttle is above current speed, accelerate
    if (th_b > _speed_b) {
      adj_b = _getAdj(_accel_b, interval_msec);
      _speed_b = constrain(_speed_b + adj_b, TM_STOP, _throttle_b);
      
      #if defined(TM_DEBUG)
        Serial.print("Acc: adj_b is "); Serial.print(adj_b);Serial.print(", _throttle_b is "); 
        Serial.print(_throttle_b);Serial.print(", _speed_b is "); Serial.println(_speed_b);
      #endif
      
    // if the (adjusted) throttle is blow the current speed, slow down
    } else if (th_b < _speed_b) {
      adj_b = _getAdj(_deccel_b, interval_msec);
      _speed_b = constrain(_speed_b - adj_b, TM_STOP, TM_FULL);
      
      #if defined(TM_DEBUG)
        Serial.print("Dec: adj_b is "); Serial.print(adj_b);Serial.print(", _throttle_b is "); 
        Serial.print(_throttle_b);Serial.print(", _speed_b is "); Serial.println(_speed_b);
      #endif
      
    // if the brakes were on but we werent reversing and now speed and throttle match, release the brakes
    }  else if (_braking_b && !_reversing_b) {
      _braking_b = false; // done
    }

  // if we've adjusted things, apply the changes
  if (_speed_a != old_a) {
    #if defined(TM_DEBUG)
      Serial.print(millis()); Serial.print(": about to call _setMotor for A, _speed_a=");
      Serial.print(_speed_a);
      Serial.print(", th_a=");Serial.print(th_a);
      Serial.print(", adj_a=");Serial.println(adj_a);
    #endif

    _setMotor(TM_MOTOR_A, _speed_a); 
  }
  if (_speed_b != old_b) _setMotor(TM_MOTOR_B, _speed_b);  
  
  _last_time = this_time; // we've done an update, note the time
  } // interval GT skew_time
} // updateThrottles

// set the internal target speed (throttle_X) and optionally the current speed (speed_X) 
// variables.
// Note that zero (TM_STOP) is a special case that is always zero, regardless of mapping.
// For non-immediate changes to values other than zero, ignore any change below a small 
// delta, as this will avoid spurious changes to the PWM rate that don't actually affect 
// anything (essentially this is a quick-and-dirty smoothing algorithm, that depends on
// the caller keeping a history if needed).
void TrainMotor::throttleA(int spdpct, boolean doitnow)
{  
  #if defined(TM_STATUS_VERBOSE)
    Serial.print(millis()); Serial.print(": throttleA called");
    if (doitnow) Serial.print(", immediate");
    Serial.print(", speed=");Serial.println(spdpct);
  #endif

  if (_eStop) return; // ignore the throttles when emergency stop is active.

  updateThrottles(); // adjust the other throttle (and set _last_time to current)

  int new_throttle = constrain( spdpct, TM_STOP, TM_FULL );
  int delta = (new_throttle - _throttle_a);

  if ((new_throttle != TM_STOP) && (abs(delta) <= 2) && !doitnow) return; // ignore small changes 
   _throttle_a = new_throttle; // accept the change

  #if defined(TM_STATUS_VERBOSE)
    Serial.print(millis()); Serial.print(": throttleA called");
    if (doitnow) Serial.print(", immediate");
    Serial.print(", _throttle_a=");Serial.print(_throttle_a);
    Serial.print(", speed=");Serial.println(spdpct);
  #endif

  // check to see if this changes the status of the brakes
  if ( (_throttle_a < _speed_a) && !_braking_a) {
    _braking_a = true;
    
    #if defined(TM_STATUS_VERBOSE)
      Serial.print(millis()); Serial.println(": throttleA speed reduction - braking set.");
    #endif
  } else if ( (_throttle_a > _speed_a) && _braking_a && !_reversing_a) {
    _braking_a = false;
    
    #if defined(TM_STATUS_VERBOSE)
      Serial.print(millis()); Serial.println(": throttleA speed increase - braking cleared.");
    #endif
  }

  if (doitnow) {
    _speed_a = _throttle_a;
    _braking_a = false;
    
    // force update and check to see if it is possible to use this motor
    if (!_setMotor(TM_MOTOR_A, _speed_a)) {
      #if defined(TM_STATUS)
        Serial.print(millis()); Serial.print(": Invalid Arduino/Shield combination, ");
        Serial.println("motor A disabled.");
      #endif
    }
  } // doitnow
} // throttleA

void TrainMotor::throttleB(int spdpct, boolean doitnow)
{
  #if defined(TM_STATUS_VERBOSE)
    Serial.print(millis()); Serial.print(": throttleB called");
    if (doitnow) Serial.print(", immediate");
    Serial.print(", speed="); Serial.println(spdpct);
  #endif
  
  if (_eStop) return; // ignore the throttles in when emergency stop is active.
  
  updateThrottles(); // adjust the other throttle (and set _last_time to current)
  
  int new_throttle = constrain( spdpct, TM_STOP, TM_FULL );
  int delta = (new_throttle - _throttle_b);

  if ((new_throttle != TM_STOP) && (abs(delta) <= 2) && !doitnow) return; // ignore small changes 
  _throttle_b = new_throttle; // accept the change
  
  // check to see if this changes the status of the brakes
  if ( (_throttle_b < _speed_b) && !_braking_b) {
    _braking_b = true;
    
    #if defined(TM_STATUS_VERBOSE)
      Serial.print(millis()); Serial.println(": throttleB speed reduction - braking set.");
    #endif
  } else if ( (_throttle_b > _speed_b) && _braking_b && !_reversing_b) {
    _braking_b = false;
    
    #if defined(TM_STATUS_VERBOSE)
      Serial.print(millis()); Serial.println(": throttleB speed increase - braking cleared.");
    #endif
  }
  
  if (doitnow) {
    _speed_b = _throttle_b;
    _braking_b = false;
    
    // force update and check to see if it is possible to use this motor
    if (!_setMotor(TM_MOTOR_B, _speed_b)) {
      #if defined(TM_STATUS)
        Serial.print(millis()); Serial.print(": Invalid Arduino/Shield combination, ");
        Serial.println("motor B disabled.");
      #endif
    } 
  } // doitnow
} // throttleB

// ignore acceleration and braking, and bring it all to a halt
void TrainMotor::emergencyStop(boolean stopItNow)
{
  #if defined(TM_STATUS)
    Serial.print(millis()); Serial.print(": emergencyStop called with stop=");
    Serial.println(stopItNow);
  #endif
  
  if (stopItNow) {
    _speed_a = _throttle_a = TM_STOP;
    _speed_b = _throttle_b = TM_STOP;
    _braking_a = _braking_b = false;
    _reversing_a = _reversing_b = false;
    _dir_a = _new_dir_a; // any reversal in progress needs to be remembered
    _dir_b = _new_dir_b;
    _setMotor(TM_MOTOR_A, _speed_a);
    _setMotor(TM_MOTOR_B, _speed_b);
    _eStop = true;
  } else {
    _eStop = false;
  }
} // emergencyStop

// Set the direction of motion for Motor A
void TrainMotor::_setDirA(int direction)
{
  #if defined(TM_STATUS_VERBOSE)
    Serial.print("_setDirA called, direction="); Serial.println(direction);
  #endif
  
  _dir_a = direction; // remember the change
  
  if (_a_dir2 > 0) { // dual-pin motor
    if (_dir_a == TM_EAST) digitalWrite(_a_dir2, HIGH);
    else digitalWrite(_a_dir2, LOW);
  } 
  
  if (_a_dir1 > 0) {
      if (_dir_a == TM_EAST) digitalWrite(_a_dir1, LOW); 
      else digitalWrite(_a_dir1, HIGH);
  }
} // _setDirA

// Set the direction of motion for Motor B
void TrainMotor::_setDirB(int direction)
{
  #if defined(TM_STATUS_VERBOSE)
    Serial.print("_SetDirB called, direction="); Serial.println(direction);
  #endif
  
  _dir_b = direction; // remember the change
  
  if (_b_dir2 > 0) { // dual-pin motor
    if (_dir_b == TM_EAST) digitalWrite(_b_dir2, HIGH);
    else digitalWrite(_b_dir2, LOW);
  } 
  if (_b_dir1 > 0) {
      if (_dir_b == TM_EAST) digitalWrite(_b_dir1, LOW); 
      else digitalWrite(_b_dir1, HIGH);
  }
} // _setDirB

// Set the direction of motion for Motor A
void TrainMotor::directionA(int direction)
{
  #if defined(TM_DEBUG)
    Serial.print(millis()); Serial.print(": directionA called, direction="); Serial.println(direction);
  #endif
  
  // validate and clean user input
  int new_dir = direction;
  if ((new_dir < TM_REVERSE) || (new_dir > TM_WEST)) new_dir = TM_REVERSE;
  
  if (new_dir == TM_REVERSE)
  {
    if (_new_dir_a == TM_EAST) new_dir = TM_WEST;
    else new_dir = TM_EAST;
  } 
  
  if ((new_dir != _dir_a) && (_speed_a > 0)) {
    #if defined(TM_DEBUG)
      Serial.print("Hit the brakes and put it in reverse - A, new_dir="); Serial.println(new_dir);
    #endif
    _braking_a = true; 
    _reversing_a = true;

    _new_dir_a = new_dir;
    _last_time = millis(); // restart the interval timer
  } else _setDirA(new_dir); // do it now if we're not moving
} // directionA

// Set the direction of motion for Motor B
void TrainMotor::directionB(int direction)
{
  #if defined(TM_DEBUG)
    Serial.print(millis()); Serial.print(": directionB called, direction="); Serial.println(direction);
  #endif

  // validate and clean user input
  int new_dir = direction;
  if ((new_dir < TM_REVERSE) || (new_dir > TM_WEST)) new_dir = TM_REVERSE;
  
  if (new_dir == TM_REVERSE)
  {
    if (_new_dir_b == TM_EAST) new_dir = TM_WEST;
    else new_dir = TM_EAST;
  } 
  
  if ((new_dir != _dir_b) && (_speed_b > 0)) {
    #if defined(TM_DEBUG)
      Serial.print("Hit the brakes and put it in reverse - B, new_dir="); Serial.println(new_dir);
    #endif

    _braking_b = true; 
    _reversing_b = true;
    
    _new_dir_b = new_dir;
    _last_time = millis(); // restart the interval timer
  } else _setDirB(new_dir); // do it now if we're not moving
} // directionB

// Return true if the brakes are applied
boolean TrainMotor::isBraking(int motor)
{
  if (motor == TM_MOTOR_A) {
    return(_braking_a);
  } else if (motor == TM_MOTOR_B) {
    return(_braking_b);
  } // else ignore any bogus input
  return(false);
} // isBraking

// Set up the motor acceleration parameters
void TrainMotor::motorAcc(int motor, int acc, int decc)
{
  #if defined(TM_STATUS)
    Serial.print(millis()); Serial.print(": motorAcc called for motor ");
    Serial.print(motor); Serial.print(", acc="); Serial.print(acc);
    Serial.print(", decc=");Serial.println(decc);
  #endif

  if (motor == TM_MOTOR_A) {
      if ((acc > 0) && (acc < 256)) _accel_a = acc;  // use caller-provided values if clean
      if ((decc > 0) && (decc < 256)) _deccel_a = decc;
  } else if (motor == TM_MOTOR_B) {
      if ((acc > 0) && (acc < 256)) _accel_b = acc;
      if ((decc > 0) && (decc < 256)) _deccel_b = decc;
  } // else ignore any bogus input
} // motorAcc

// minspd is the lowest throttle setting that makes the train move, anything less 
// needs to be treated as zero.
void TrainMotor::motorMinSpeed(int motor, int minspd)
{
  #if defined(TM_STATUS)
    Serial.print(millis()); Serial.print(": motorMinSpeed called for motor ");
    Serial.print(motor);
    Serial.print(", minspd=");Serial.println(minspd);
  #endif

  if (minspd >= TM_FULL) minspd = TM_FULL;  // this would be a silly minimum
  else if (minspd == TM_STOP) minspd = TM_STOP+1;
  
  if (motor == TM_MOTOR_A) {
    _min_a = minspd;
  } else if (motor == TM_MOTOR_B) {
    _min_b = minspd;
  } // else ignore any bogus input
} // motorMinSpeed

// define what we need to interact with the motor shield
// the accel and deccel rates are expressed in points (out of 255) to add/remove
// each 100 msec (1 = 25.5 seconds to full speed, 10 = 2.55 seconds, 100 = 0.3 seconds)
void TrainMotor::motorSetup(int shield) 
{
  #if defined(TM_STATUS_VERBOSE)
    Serial.print(millis()); Serial.print(": motorSetup called");
    Serial.print(", shield=");Serial.println(shield);
  #endif

  // Setup the default acceleration and braking rates - motorAcc can override these
  _accel_a = 25; // one second to full speed
  _deccel_a = 50; // a half-second to stop from full
  _accel_b = 25;
  _deccel_b = 50;
  
  _shield = shield;  // what motor shield are we using

  // make the pin assignments for the specified shield
  switch (_shield) {
  
    case TM_ARDUINO_MOTOR:
      pinMode( 8, OUTPUT); digitalWrite(8, LOW); // disable the brake function
      pinMode( 9, OUTPUT); digitalWrite(9, LOW);
    case TM_ARDUMOTO_MOTOR:
        _a_pwm =   3;
        _b_pwm =  11;
        _a_dir1 = 12;
        _a_dir2 =  -1; // unused
        _b_dir1 = 13;
        _b_dir2 =  -1; // unused
        #if defined(TM_STATUS)
          Serial.print(millis());
          Serial.println(": motorSetup called - Arduino/Ardumoto shield");
        #endif
    break;
    
    // this applies for both V1 and V2 seeed motor shields, but not their "2A motor shield",
    // which appears to be a DFRobot.com model.
    case TM_SEEED_MOTOR:
        _a_pwm =   9;
        _b_pwm =  10;
        _a_dir1 =  8;
        _a_dir2 = 11;
        _b_dir1 = 12;
        _b_dir2 = 13;
        #if defined(TM_STATUS)
          Serial.print(millis());
          Serial.println(": motorSetup called - Seeed shield");
        #endif
    break;
    
    case TM_DFROBOT_MOTOR:
        _a_pwm = 6;
        _b_pwm = 5;
        _a_dir1 = 7;
        _a_dir2 = -1;
        _b_dir1 = 4;
        _b_dir2 = -1;    
        #if defined(TM_STATUS)
          Serial.print(millis());
          Serial.println(": motorSetup called - DFRobot shield");
        #endif
    break;
    
    default:
        _a_pwm = 0;
        _b_pwm = 0;
        _a_dir1 = 0;
        _a_dir2 = 0;
        _b_dir1 = 0;
        _b_dir2 = 0;
        #if defined(TM_STATUS)
          Serial.print(millis());
          Serial.println(": motorSetup called - bad shield parameter specified!");
        #endif
  }
  
  // set up the acceleration rates to a default
  
  _skew_time = TM_SKEW_INTERVAL; // milliseconds between adjustments (alter if we modify Timer0)

  // Open the necessary pins for output use, ignore ones not in use
  if ( _a_pwm  > 0) pinMode( _a_pwm, OUTPUT);
  if ( _b_pwm  > 0) pinMode( _b_pwm, OUTPUT);
  if ( _a_dir1  > 0) pinMode( _a_dir1, OUTPUT);
  if ( _a_dir2  > 0) pinMode( _a_dir2, OUTPUT);
  if ( _b_dir1  > 0) pinMode( _b_dir1, OUTPUT);
  if ( _b_dir2  > 0) pinMode( _b_dir2, OUTPUT);
  
  // Set up the timers for our chosen PWM frequency given the pins in use
  _pwmSetup();
  
  // Initialize the motors to a safe state
  _braking_a = false; _reversing_a = false;
  _braking_b = false; _reversing_b = false;
  _new_dir_a = TM_EAST;
  _new_dir_b = TM_EAST;
  _setDirA(_new_dir_a); // ensure everything agrees which way we are moving
  _setDirB(_new_dir_b);
  throttleA(TM_STOP, true); // using the Throttle command to initialize time counter
  throttleB(TM_STOP, true); // this also ensures the check for valid arduino/shield combos is made
} // motorSetup
