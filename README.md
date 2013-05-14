# TrainMotor-Library
---
Version a1 - initial version, May 2013

by Ken Shores

A library for use with an Arduino that supports DC model train control for simple small models, such as N-Scale or efficient HO/OO, using one of several common motor shields. Up to two trains on separate tracks can be driven, within the power limits of the motor shield.

This code is released to the public domain.

At present this library, while working, is still under development and includes a number of status *println* commands for use with the serial monitor.  To disable these, comment out the TM_STATUS *#define* in *TrainMotor.h*.

**
```
Note: the only shield I presently know to work is the Ardumoto on an Arduino Uno, although limited testing on a Mega has been done as well. The Seeed Studios V2 shield has problems with isolating the shield from the Arduino power, and mine stopped working during testing after getting quite hot. I haven't yet tested the Arduino R3 or a DFRobot shield.
```
**

## Caveats:
---
This code is under development and testing has been limited. While it seems to work, there is no guarantee that it won't harm your train. Use with caution. More formally: This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  

Motor shields may be very hot, and PWM can produce heat sufficient to weld metal via arcing (see the *Power Warnings* section). Exercise due caution and do not allow small children to operate trains powered by motor shields without supervision.


## Overview:
----

This library implements a set of functions for controlling up to two independent H-bridge motor control systems on L293 or L298-based Arduino motor shields suitable for small DC motors, such as are used in model trains. It provides for direction, throttle and momentum control, along with an Emergency Stop feature. It uses ultrasonic PWM to reduce motor noise and limit heating in the motor armature.

This library was developed for creating an automated control system for a small N-Scale light-rail line within a larger model railroad. Its functions, other than *activeDelay()*, do not use "delay" functions and will immediately return, making it suitable for a constantly-cycling program reading and reacting to sensors.

Using supersonic PWM requires adjusting the speed of the timer associated with the two pins in use for PWM signaling to the shield, which is done by the *motorSetup()* routine. For this reason, it will not work with CPU/shield combinations that need to use Timer0 for one of the shield PWM pins, see the *Limitations* section.

Two example program sketches are provided:

* *oscillate* - A simple program that moves a train back and forth a few inches under timer control. This will work with just an Arduino and a motor shield (an external power supply for the motor shield is highly recommended).

* *simple_throttle* - A non-automated program that provides a single-train DC power pack (this additionally requires a potentiometer and three SPST switches).

## Features:
---
  * Supports motor shields from:
  
      * Arduino (R3 version tested), 
    
      * Seeed Studio (both version 1 and version 2 tested), and 
  
      * Sparkfun (Ardumoto, tested).
  
  * Provides "ultrasonic" 32 kHz PWM.
  * Provides two software throttles with independent direction control and momentum support to ramp track power up/dowm at configurable rates.
  * Provides a "starting speed" setting to allow configuration of the minimum track voltage that will move a specific motor.
  * Provides for an Emergency Stop switch that will immediately cut track power from both motor outputs.

Notes:

1. The DFRobot motor shield code needs some work, and isn't fully implemented at this time.

2. "tested" means that shields been at least connected and used in a limited manner. Of these, the Ardumoto and Seeed Studios V2.0 shield had the most extensive evaluation, and were the only ones used with the *simple_throttle* program.


## Power Warnings:
---

Although PWM acts like DC, it is essentially a form of AC and can heat objects via arcing if a short circuit occurs (e.g., if a train derails or shorts a turnout by running into it the wrong way, or even while placing a train on the tracks). This can be intense enough to melt plastic models or even weld wheels to track. With a smaller power supply, the risk is lesser but not absent. With a 2 Amp or larger supply the risk of fire or damage to the model or to a person attempting to remove it is higher. Always turn the power off before touching a train. Don't leave a motor shield powered up unattended. **Small children should not use motor shields unsupervised.**

It is probably not the best idea to use power from the Arduino to drive a train, particularly if the Arduino is powered via USB (even without USB, electromagnetic noise from the shield could affect the Arduino's operation). Use of an external supply connected to the motor shield is recommended. Some shields do not allow for a separate power supply, and require you to connect any external supply to the Arduino.

Even for those that do have a separate connector, using a separate supply from that of the Arduino may require setting jumpers on the motor shield, or even cutting traces or bending/cutting pins (some of these shields are not very well designed). It's a good idea to read up on the details before purchasing a shield.

While many shields claim "2 Amp" support, it may be overly optimistic to assume these can run larger model trains that actually need this amount of power for an extended period, particularly with shields that lack heat sinks on the L298 chip. 

With the Ardumoto, and possibly also with other shields, you may need to take additional steps when using an external power supply to isolate the shield's Vin pin from the Arduino. I cut the Vin pin off my Ardumoto. On the Seeed V2 shield, the two-pin jumper behind the power terminal needs to be removed, but it's not clear that this actually isolates the shield from the Arduino (it seems to be a 5/12 selection jumper for the output).

In general, it is a very good idea to connect the grounds of the power supplies together if separate ones are used for the Arduino and the shield. Some shields will do this internally, others appear not to do so. See your shield's documentation.


## Motor Shields:
---

A motor shield is essentially an amplifier that converts the Arduino's 5 Volt PWM signal to a higher voltage (typically 12 Volts). Most of the ones available support two motors (and the TrainMotor library supports at most two motors) using either two or three digital pins per shield. See the manufacturer's documentation for specifics.

The Arduino (R3) motor shield has been sold by Radio Shack as part 276-131. It is [documented](http://arduino.cc/de/Main/ArduinoMotorShieldR3 "Arduino Motor Shield R3") by Arduino.

```
When using the Arduino v3 motor shield, the current sense pins are not used by this library and are not initialized (you may initialize and use them if desired). 
```

The [DFRobot](http://www.dfrobot.com/ "DFRobot.com") shields (both 1 Amp and 2 Amp versions) have not been tested, but should work per the documented features once the code is fixed. The 2A shield must be jumpered for PWM mode (see their manual, [PDF](http://www.dfrobot.com/image/data/Common/Arduino%20Shield%20Manual.pdf "DFRobot Shield Manual") ). Note that DFRobot makes a number of other motor shields (for high amperage and other specialized purposes), and those have not been evaluated at all.

The seeed V1 shield is sold by Radio Shack as part 276-242. This lacks headers for external controls and cannot be stacked with other shields above it, making it less useful. Documentation is available on the [seeed wiki](http://www.seeedstudio.com/wiki/Motor_Shield_V1.0 "seeed Motor Shield V1.0"). There is also a V2 shield, which can be stacked, available [from the manufacturer](http://www.seeedstudio.com/depot/motor-shield-v20-p-1377.html "Motor Shield V2.0"). This is also documented on the [seeed wiki](http://www.seeedstudio.com/wiki/Motor_Shield_V2.0 "seeed Motor Shield V2.0").

The [SparkFun](https://www.sparkfun.com/ "SparkFun") Ardumoto motor shield is similar in function to the Arduino shield, but not identical. Documentation is available on their [product page](https://www.sparkfun.com/products/9896 "Ardumoto").

The SEEED "2 Amp" motor shield has not been tested, but should work using the DFRobot shield definition once that is ready.

See *Limitations* for notes on supported Arduino/shield combinations, and the *How it Works* section for further details.

## Arduino Output Use:
---

This library works using the default output pins for each model of shield:

*Arduino R3*

* 3 (Motor A PWM) 
* 12 (Motor A direction)
* 8 (Motor A brake - set LOW and not used)
* 11 (Motor B PWM)
* 13 (Motor B direction)
* 9 (Motor B brake - set LOW and not used)

Additionally there are current sense pins on this shield, which are not used, and pins for a set of TinkerKit sensors (5, 6, A2, A3).

```
If you have cut the "brake" jumper, this shield is identical to the Ardumoto, as both use the same basic four pins for motor control; use the Ardumoto shield type if you want to avoid the library attempting to use those pins.
```

*SparkFun Ardumoto*

* Same as Arduino R3 for PWM and direction, no brakes or current sense.

*Seeed Studios* - Version 1.0 and 2.0 shields

* 9 (Motor A PWM)
* 8, 11 (Motor A direction)
* 10 (Motor B PWM)
* 12, 13 (Motor B direction)

*DFRobot.com* - 1 Amp and 2 Amp shields

* 6 (Motor A PWM)
* 7 (Motor A direction)
* 5 (Motor B PWM)
* 4 (Motor B direction)


## Limitations:
---

Use of 16 MHz Arduino boards is assumed. The library can be adjusted to provide 16 kHz PWM when used on an 8 MHz CPU, but this has not been tested.

The allowed combinations of Arduino and motor shield are limited to the following due to Arduino timer issues:

* Uno (or other 168/328-based model) and Arduino (R3), Sparkfun (Ardumoto) or Seeed (V1/V2) shields (not DFRobot).
* Leonardo (or other 32U4-based model) and DFRobot or Seeed shields (not Sparkfun Ardumoto or Arduino R3 shields).
* Mega and Arduino, DFRobot, Seeed or Sparkfun shields.


## How it Works:
---

Pulse-Width Modulation (PWM) allows a fixed DC voltage to act like a variable DC voltage. It does this by "pulsing" the voltage on and off, with longer "on" states (duty cycles) corresponding to a higher average voltage, and hence acting like a higher DC voltage. Arduino's natively support PWM, normally at a pulse cycle (frequency) of 490 Hz. This is an audible frequency, and a motor driven by this form of PWM will tend to "buzz" as the changing magnetic field in the armature causes it to vibrate at the same frequency.

For supported combinations of Arduino and motor shield, the TrainMotor library changes the PWM prescale factor of the timer(s) associated with the PWM output pins used by the shield to yield 32 kHz PWM instead of the default 490 Hz PWM. This provides quieter operation, and less risk of damage to coreless motors.

Note that "less" does not necessarily mean "none"; coreless motors are very vulnerable to heating and PWM will always cause some degree of heating.

Use of high-frequency PWM shifts the vibrations created in the motor by the oscillating voltage outside of the range audible by people, making the motor operation quieter. It also produces less heat in the motor armature, which with low-frequency PWM can be enough to damage coreless motors (and some small trains use coreless motors). There may be a cost in reduced torque (pulling power), but for the application this was intended for (a light rail line) that was not an issue.

This is similar to the method used in so-called "ultrasonic" DCC decoders, which also use PWM, typically at 15 kHz or higher, but this does not use DCC in any manner. And in addition, the Arduino lacks the adaptive mechanisms found in most modern DCC decoders for low-speed or varying-load (hills) operation. It's a very basic form of PWM.

Because TrainMotor adjusts the timer operation, it will not work with Arduino/shield combinations that put the PWM pins on the same timer used for delay and other functions, as doing so would cause time-based operations to break. On such combinations the throttles simply won't do anything, and if a serial output monitor is active the *motorSetup()* routine will print an error message to note the conflict.

One thing to be aware of: while some of these shields at least nominally support voltages higher than 12V (up to 18V), such voltages will produce more heat in the regulator circuits, and probably also in the L298 chip. Additionally, Arduinos generally do not support more than 12V, so you'd need to be sure that you had good isolation of the two supplies (grounds still need to be connected). I haven't tested that and don't plan to, as 12V is sufficient for my needs.

A train on 12V PWM will run about 15% slower than one on pure DC at 12V, but most "N-Scale" power packs put out closer to 15V, and HO-spec ones may put out 18V or more. Maximum train speed on 12V may be considerably less than you are used to. It's still, at least for my N-scale test model, about 90 scale MPH (~140 km/h), considerably more than is needed for most small trains.  HO scale speed will be much less, of course, but probably acceptable for a light rail line.

## Library Usage:
---
  * Download the library files.
  * Place the "TrainMotor" folder containing the files in your Arduino1.0+ "libraries" folder.
  * Open one of the example sketches: "oscillate", or "simple_throttle", or write your own.
  * Edit the setup() routine to use the name of the motor shield in use (see the *shield* section below)
  * Connect the shield to the Arduino using the standard pins for that shield.
  * For larger trains (and recommended for all), connect an external regulated 9V or 12V power supply to the shield providing at least 600 mA (12V preferred) and isolate the shield power from the Arduino as per its documentation.
  * Connect one or both motor shield outputs to (separate) tracks.
  * Compile & upload code
  * Play with trains
  
  Note: it is recommended to power off the Arduino before placing a train on the track or removing it. At a minimum ensure that the throttle is set to zero or the "Emergency Stop" feature is active before doing so (both will cut track power, assuming they're working correctly). See the *Power Warnings* section.
  
### Example Sketches:

`ocsillate` runs a train back and forth over a short length of track (about 8 inches or 20cm at a non-full throttle setting) under timer control. This is useful for testing equipment operation without additional control circuits.

`simple_throttle` With a potentiometer and three SPST switches added, this provides for a simple DC "Power Pack" using the motor A outputs.  The potentiometer (Radio Shack 271-1715 or other 10K ohm linear-taper model) should be connected to analog pin A0, and the switches (Radio Shack 275-634 or equivalent) to digital pins 2 (direction), 4 (run/park) and 5 (emergency stop). See the link below for an illustration of how to connect the controls.

Note: momentum is disabled in *simple_throttle* to allow determination of the lowest throttle setting that makes the train move. Once this is known, change the two constants at the top of the program as instructed there to enable momentum processing.

[Simple Throttle wiring](simple_throttle.jpg)


## Function Reference:
---
`TrainMotor()` Creates the motor controller object. Must be done first (above the main program code).

`void activeDelay(int msec)` will wait *msec* milliseconds but during that time will update throttle settings per momentum rules. Don't use this if you also need to do something else during that time. If you need to wait less than a millisecond, the ordinary *delayMicroseconds()* function may be used instead, but calling *updateThrottles()* immediately before or after doing so is recommended.

`void directionA(int direction)` Sets motor A direction (see below).

`void directionB(int direction)` Sets motor B direction (see below).

`void emergencyStop(boolean stopItNow)` If called with *stopItNow* set to *true*, will enter *emergency stop* mode, cancelling all momentum, updating saved direction to the one that would apply at the end of momentum processing, and setting throttles (and thus track power) to zero. While in *emergency stop* mode, all throttle commands will be ignored. If called with *stopItNow* set to false, clears *emergency stop* mode but leaves throttles set to zero.

`boolean isBraking(int motor)` Called with *motor* set to TM_MOTOR_A or TM_MOTOR_B it will return true if momentum is currently in effect to decrease train speed (this is true if the direction has been reversed but the train is still slowing down prior to reversing, or if the throttle has been set below the current speed).

`void motorAcc(int motor, int acc, int decc)` Called with *motor* set to TM_MOTOR_A or TM_MOTOR_B it will set the *acceleration* and *deceleration* rates for that motor to the specified values. The two rates are specificed in units of throttle points (on a 1-255 scale) to change every tenth of a second, thus 25 will take one full second to change from full (255) throttle to stopped (0) throttle. Setting either to 255 disables that momentum setting.

`void motorMinSpeed(int motor, int minspd)` Called with *motor* set to TM_MOTOR_A or TM_MOTOR_B it will set the minimum speed to *minspd* (1-255). This is the speed that, without momentum, is the lowest that causes the train to move. With this set above 1, entered throttle values are mapped to the range from minspd to 255 before setting the shield, meaning that a throttle of 64 be treated as a command to set the throttle to 25% (64/255) of the way from minspd to 255. Setting this to 1 cancels any minimum speed in effect. Once minspd is set, a throttle value of 1 should cause the train to move at its slowest speed.

`void motorSetup(int shield)` This function initializes the library and must be called once with the name of the shield in use (see below).  It is typically called from *Setup()*.

`void throttleA(int speed, boolean now)` When called it sets the target speed for motor A to be *speed* (0-255). If *now* is false, momentum is applied. If *now* is true, the change is immediate.

`void throttleB(int speed, boolean now)` When called it sets the target speed for motor B to be *speed* (0-255). If *now* is false, momentum is applied. If *now* is true, the change is immediate.

`void updateThrottles()` Must be called periodically to apply momentum effects to motor speeds and do related processing. It is recommended to call this at least once each time around *loop()*. Calling it more than needed will not affect operation. Note: required even if momentum feature is not used.

## Constants:
---

### Shield:

`TM_ARDUINO_MOTOR` The Arduino R3 motor shield, or any compatible shield with brakes using pins 3 and 11 for PWM, 12 and 13 for direction and 8 and 9 for brakes. The *brake* pins are not used in the current code, other than to make them *OUTPUT* and set them to *LOW* to turn the brakes off. User code may manipulate them after *motorSetup()* has been run. A future version of this library may, however, make use of them.

`TM_ARDUMOTO_MOTOR` The Sparkfun Ardumoto shield. This is identical to the Arduino motor shield in function except that it lacks brake pins. This type can also be used for an Arduino shield that has had the brake pin jumpers cut.

`TM_SEEED_MOTOR` The Seeed Studios motor shield, version 1 or 2, or any compatible shield using pins 9 and 10 for PWM, and 8, 11, 12 and 13 for direction.

`TM_DFROBOT_MOTOR` **Not fully implemented yet.** The DFRobot.com motor shield. This will probably also work for the Seeed Studios "2 Amp" motor shield as it uses the same pins. Works for shields using pins 6 and 5 for PWM, and 7 and 4 for direction.

### Direction:
`TM_EAST` An arbitrary direction.

`TM_WEST` The opposite of *TM_EAST*.

`TM_REVERSE` Reverse current direction set. Note that if the train is braking due to a prior direction change, but has not actually stopped and reversed yet, this cancels the pending change.

### Motor:
`TM_MOTOR_A` The first motor on the shield ("A" or #1).


`TM_MOTOR_B` The second motor on the shield ("B" or #2).

### Speed:
A speed may be any number from 0 (stop) to 255 (full throttle). Two constants are defined for convenience:

`TM_STOP` 0

`TM_FULL` 255

## Version History:
---
  * Version a1: current/initial version (May 2013)
