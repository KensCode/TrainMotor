// Tram Controller - Example Program "Oscillate"
// V1.00 - May 2013 - Ken Shores
// This Arduino sketch (program) is released to the public domain.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
//
// This program causes a train on track connected to the Motor A outputs of the motor
// shield to move back and forth several inches at moderate speed, demonstrating the 
// momentum feature.  Although motorAcc is called here, the values it sets are the 
// default ones, so it is redundant.
//
// It assumes use of an Ardumoto shield (which will also work for the Arduino Motor Shield),
// change the parameter on motorSetup for other shields.
//
// This sketch assumes the Arduino is connected to a computer and the Serial Monitor is active.

#include <TrainMotor.h>

TrainMotor TrainMotor;

const int portSpeed = 9600; // USB tty speed

int dir = TM_EAST; // direction indicator

// One-time initial Setup code - executed on power-up or Arduino reset
void setup()
{
  delay(5000); // kill some time 
  
  Serial.begin(portSpeed); // debug
    while (!Serial) ; // wait for serial port to connect (mainly needed for Leonardo)

  TrainMotor.motorSetup(TM_ARDUMOTO_MOTOR);  // initialize the motor shield routines
  TrainMotor.motorAcc(TM_MOTOR_A, 25, 50);
  TrainMotor.motorAcc(TM_MOTOR_B, 25, 50);   // not used in this sketch
  
  Serial.print(millis());
  Serial.println(": About to start.");
}

// Main execution Path - executed repeatedly while Arduino powered
void loop()
{  
  TrainMotor.updateThrottles(); // this is redundant with activeDelay below, but good practice
    
  // proceed at slow pace
  TrainMotor.throttleA(192); // since we're never changing it, this could be called in setup

  // now throw it into reverse - the train will slow down and eventually reverse
  // The first time, this change happens before the train has a chance to start moving.
  if (dir == TM_EAST)
    {
      dir = TM_WEST;
 
      Serial.print(millis());
      Serial.println(": Set direction WEST.");
      TrainMotor.directionA(TM_WEST);
    } else {
      dir = TM_EAST;
     
      Serial.print(millis());
      Serial.println(": Set direction EAST.");
      TrainMotor.directionA(TM_EAST);
    }
    
  // kill time while we wait for the momentum to take effect, and allow some time for it to run
  TrainMotor.activeDelay(3000); 
}
