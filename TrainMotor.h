/*  TrainMotor.h
    Version a1 - initial alpha release
    A library for controlling model trains using a motor shield.
    
    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
    
    Released into the public domain.
*/

#include "Arduino.h"

#ifndef TrainMotor_h
#define TrainMotor_h

// the following constants may be changed

// Print settings:
//
// uncomment to enable debug print statements
// TM_DEBUG - vebose status for debugging the library
// TM_STATUS - helpful status messages for debugging a sketch using the library
// note that print and println statements take time, which can affect the sketch
// each status message is preceded by the current time from millis()
// TM_STATUS_VERBOSE - reports each function call (which may be far too much information
// in a busy program.
//#define TM_DEBUG
#define TM_STATUS
//#define TM_STATUS_VERBOSE

// PWM Frequency Settings:
//
// It is important to use "ultrasonic" PWM (16 MHz or 32 MHz) to avoid risk of damage
// to coreless motores and reduce audible noise.  See the README file.
//
// Settings to choose pwm frequency (16 kHz or 31 kHz are preferred)
//   prescale 1 is 31 kHz on phase-correct pwm with 16 MHz clock
//   prescale 1 is 16 kHz on fast pwm with 8 MHz clock
// set prescale 1-5 (1 is normal)
// set fast to 1 for fast pwm, 0 for phase-correct pwm (as above)
//
// On most Arduinos, set prescale=1, fast=0; on 8 MHz models set prescale=1, fast=1.

#define TM_PRESCALE 1
#define TM_FAST_PWM 0

// the following constants should not be changed unless you know what you're doing

// direction of motion for trains
#define TM_REVERSE 1
#define TM_EAST 2
#define TM_WEST 3

// initialization values for shield type
// The Arduino V3 shield uses the same pins as the Ardumoto, but also adds brake pins
#define TM_CUSTOM_MOTOR 0
#define TM_ARDUINO_MOTOR 1
#define TM_ARDUMOTO_MOTOR 2
#define TM_SEEED_MOTOR 3
#define TM_DFROBOT_MOTOR 4

// speed constants - note that user input is always 0-255, even if we're scaling
// these are used internally, as well as by user code and relate to register field width - do not change
#define TM_STOP 0
#define TM_FULL 255

// motor names (for shield outputs) - used internally
#define TM_MOTOR_A 1
#define TM_MOTOR_B 2

// SKEW_INTERVAL = milliseconds between throttle updates - value interacts with timer0 changes, alter carefully
// MIN_UPDATE = milliseconds between checking for changes to the throttle
#define TM_SKEW_INTERVAL 100
#define TM_MIN_UPDATE 20

// Do not use the internal variables in external code unless you understand what they
// are doing; the values in them may not match what the calling program uses.  They are
// private for a reason.

class TrainMotor
{
  public:
    TrainMotor();
    void activeDelay(int msec);
    void directionA(int direction);
    void directionB(int direction);
    void emergencyStop(boolean stopItNow);
    boolean isBraking(int motor);
    void motorAcc(int motor, int acc, int decc);
    void motorMinSpeed(int motor, int minspd);
    void motorSetup(int shield);
    void throttleA(int spdpct, boolean doitnow);
    void throttleB(int spdpct, boolean doitnow);
    void updateThrottles();
  private:
    int _shield;
    long _skew_time;
    float _skew_mod;
    int _a_pwm;
    int _b_pwm;
    int _a_dir1;
    int _a_dir2;
    int _b_dir1;
    int _b_dir2;
    int _throttle_a;
    int _speed_a;
    int _throttle_b;
    int _speed_b;
    int _accel_a;
    int _accel_b;
    int _deccel_a;
    int _deccel_b;
    int _dir_a;
    int _dir_b;
    int _new_dir_a;
    int _new_dir_b;
    int _min_a;
    int _min_b;
    boolean _braking_a;
    boolean _braking_b;
    boolean _reversing_a;
    boolean _reversing_b;
    unsigned long _last_time;
    int _getAdj(int adj, int msec);
    void _pwmSetup();
    void _setDirA(int direction);
    void _setDirB(int direction);
    boolean _setMotor(int which_motor, int speed);
};

#endif