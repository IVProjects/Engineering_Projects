/*NOTE: This sketch produces a basic back and forth motion. The code is all very simple, so feel free to modify it to produce whatever movements you would like.
 *      Here is the Class Reference for the AccelStepper library if you would like to take a look:  
 *      https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html#a68942c66e78fb7f7b5f0cdade6eb7f06
 */

#include<AccelStepper.h> // Includes the AccelStepper library

#define HALFSTEP 8 // Sets the driving mode to half-step

AccelStepper wheelStepper(HALFSTEP , 3 , 5 , 4, 6); // Creates an instance


float travelDist = 400; // Distance the dolley will travel before reversing (in units of mm)
int stepsPerRevolution = 2038; // Change this if your motor has a different number of steps per revolution
float wheelDia = 60; // Wheel Diameter in mm
float wheelCirc;

void setup() {
  
  wheelStepper.setMaxSpeed(1600); //Maxixmum value I found works well: 1600 steps/sec
  wheelStepper.setAcceleration(800); // Maxixmum value I found works well: 800 steps/sec^2
  wheelStepper.setSpeed(600); 
  
  wheelCirc = wheelDia*3.14159;
 
  wheelStepper.moveTo((travelDist/wheelCirc)*stepsPerRevolution); // Commands the dolley to move the requested travel distance
  
}

void loop() {
  
  if (wheelStepper.distanceToGo() == 0){
    wheelStepper.moveTo(-wheelStepper.currentPosition()); // Reverses the stepper when it reaches its endpoint
  }

  wheelStepper.run(); // Commands the motor to run one step if needed
  
}
