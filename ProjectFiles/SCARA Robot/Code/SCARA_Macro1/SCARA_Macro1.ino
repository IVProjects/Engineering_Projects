#include <AccelStepper.h>
#include <Servo.h>

float stepsPerDegree = 35.56; // 200 steps/rev motor,  16:1 gearbox, and quarter-step driving
float stepsPerDegree3 = 11.11; // 200 steps/rev motor, 5:1 gearbox, and quarter-step driving
int stepsPermm = 30.3; // Steps per mm travel for Z-axis
int joint1Home = -105; // Angle from straight in degrees, CCW positive
int joint2Home = -137;
int joint3Home = 142;
int zHome = -50; // distance in mm down from top of Z-travel

Servo servo1;
int servoPos;
int openPos = 175;
int gripPos = 10;


////// STEPPER DIRECTION AND STEP PINS:
const int dirPin1 = 5; //// X on CNC shield
const int stepPin1 = 2;

const int dirPin2 = 6; ///// Y on CNC shield
const int stepPin2 = 3;

const int dirPin3 = 7; ///// Z on CNC sheild
const int stepPin3 = 4;

const int dirPin4 = 13; ///// A on CNC shield (put jumpers between pins)
const int stepPin4 = 12;

const int enablePin = 8;


/////// LIMIT SWITCHES: (1 by default, 0 when pressed. Connect outer pins to input pin and ground on arduino)
int limit1 = 9; // X limit on CNC sheild
int limit2 = 10; // Y
int limit3 = A3; // Coolant Enable
int limit4 = 11; // Z

int limState1;
int limState2;
int limState3;
int limState4;

long homing1 = 1; // Make (-) for CCW homing, and (+) for CW homing (also need to change increment in homing function)
long homing2 = 1;
long homing3 = -1;
long homing4 = 1;

// Define motor interface type (type 1 means an external stepper driver with Step and Direction pins)
#define motorInterfaceType 1

// Define steppers and the pins they use
AccelStepper stepper1(motorInterfaceType, stepPin1, dirPin1);// First Joint
AccelStepper stepper2(motorInterfaceType, stepPin2, dirPin2);// Second Joint
AccelStepper stepper3(motorInterfaceType, stepPin3, dirPin3);//Third Joint (end effector rotation)
AccelStepper stepper4(motorInterfaceType, stepPin4, dirPin4);// Z-axis


////// HOMING FUCNTION:
void homing(){ 
  
  limState1 = digitalRead(limit1);
  limState2 = digitalRead(limit2);
  limState3 = digitalRead(limit3);
  limState4 = digitalRead(limit4);

  while (limState1 == 1){
    stepper1.moveTo(homing1);
    homing1++;
    stepper1.run();
    delay(2);
    limState1 = digitalRead(limit1);
    //Serial.println(limState1);
  }
  delay (500);


  while (limState2 == 1){
    stepper2.moveTo(homing2);
    homing2++;
    stepper2.run();
    limState2 = digitalRead(limit2);
    delayMicroseconds(700);
    //Serial.println(limState2);
  }
  delay (500);
  

  while (limState3 == 1){
    stepper3.moveTo(homing3);
    homing3--;
    stepper3.run();
    limState3 = digitalRead(limit3);
    delayMicroseconds(200);
    //Serial.println(limState3);
  }
  delay (500);

    while (limState4 == 1){
      stepper4.moveTo(homing4);
      homing4++;
      stepper4.run();
      limState4 = digitalRead(limit4);
      delayMicroseconds(700);
      //Serial.println(limState4);
  }  
  delay (500); 
  
} /// Homing function end



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  servo1.attach (A2); // Declare Servo Object (A2 = "Resume" pin)
  servoPos = openPos;

  pinMode (limit1, INPUT);
  pinMode (limit2, INPUT);
  pinMode (limit3, INPUT);
  pinMode (limit4, INPUT);

  digitalWrite(limit1, HIGH);
  digitalWrite(limit2, HIGH);
  digitalWrite(limit3, HIGH);
  digitalWrite(limit4, HIGH);

  pinMode(enablePin, OUTPUT); // CNC shield enable pin
  digitalWrite(enablePin, LOW);


  /////////// STEPPER PROPERTIES:

  stepper1.setMaxSpeed(14000); //// From Accelstepper: "Max possible speed depends on clock speed. Caution: Speeds that exceed the maximum speed supported by the processor may Result in non-linear accelerations and decelerations"
  stepper1.setAcceleration(10000);

  stepper2.setMaxSpeed(14000);
  stepper2.setAcceleration(10000);

  stepper3.setMaxSpeed(10000);
  stepper3.setAcceleration(10000);

  stepper4.setMaxSpeed(12000);
  stepper4.setAcceleration(10000);

  homing(); ///////////////////////// RUN ARM HOMING FUNCTION


  stepper1.setCurrentPosition(0); // Set current stepper position after homing and becoming straight as zero (THIS SETS MOTOR SPEED TO 0)
  stepper2.setCurrentPosition(0); 
  stepper3.setCurrentPosition(0); 
  stepper4.setCurrentPosition(0); 

  
  stepper1.move(stepsPerDegree*joint1Home); // Moves joint one to straight ahead (arm will crash if signs are wrong)
  stepper2.move(stepsPerDegree*joint2Home);
  stepper3.move(stepsPerDegree3*joint3Home);
  stepper4.move(stepsPermm*zHome);

  while (stepper1.distanceToGo() != 0){
    stepper1.run();
  }

  while (stepper2.distanceToGo() != 0){
    stepper2.run();
  }

  while (stepper3.distanceToGo() != 0){
    stepper3.run();
  }
  
  while (stepper4.distanceToGo() != 0){
    stepper4.run();
  }

  stepper1.setCurrentPosition(0); // Set current stepper position after homing and becoming straight as zero (THIS SETS MOTOR SPEED TO 0)
  stepper2.setCurrentPosition(0); 
  stepper3.setCurrentPosition(0); 
  stepper4.setCurrentPosition(0); 
 
} // Setup End


void loop() {
 

//////////////////POSITION 1:

  servo1.write(gripPos);
  delay(800); // MAKE AS SMALL AS POSSIBLE
  
  stepper1.moveTo(60*stepsPerDegree);  
  stepper2.moveTo(80*stepsPerDegree);  
  stepper3.moveTo(45*stepsPerDegree3);  
  stepper4.moveTo(-200*stepsPermm);
  
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0){
  if (stepper1.distanceToGo() != 0){
    stepper1.run();
  }

  if (stepper2.distanceToGo() != 0){
    stepper2.run();
  }

  if (stepper3.distanceToGo() != 0){
    stepper3.run();
  }

  if (stepper4.distanceToGo() != 0){
    stepper4.run();
  }
  }
  delay(100);

//////////////////POSITION 2:

  servo1.write(openPos);
  delay(800); // MAKE AS SMALL AS POSSIBLE
  
  stepper1.moveTo(-80*stepsPerDegree);  
  stepper2.moveTo(-60*stepsPerDegree); 
  stepper3.moveTo(-45*stepsPerDegree3);   
  stepper4.moveTo(-100*stepsPermm);
  
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0){
    if (stepper1.distanceToGo() != 0){
      stepper1.run();
    }
  
    if (stepper2.distanceToGo() != 0){
      stepper2.run();
    }
  
    if (stepper3.distanceToGo() != 0){
      stepper3.run();
    }
  
    if (stepper4.distanceToGo() != 0){
      stepper4.run();
    }
  }
  delay(100);

//////////////////POSITION 3:
  stepper1.moveTo(0*stepsPerDegree);  
  stepper2.moveTo(0*stepsPerDegree); 
  stepper3.moveTo(0*stepsPerDegree3);   
  stepper4.moveTo(-50*stepsPermm);

  servo1.write(gripPos);
  delay(800); // MAKE AS SMALL AS POSSIBLE
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0){

  if (stepper1.distanceToGo() != 0){
    stepper1.run();
  }

  if (stepper2.distanceToGo() != 0){
    stepper2.run();
  }

  if (stepper3.distanceToGo() != 0){
    stepper3.run();
  }

  if (stepper4.distanceToGo() != 0){
    stepper4.run();
  }
  }
  delay(100);

}
