#include <AccelStepper.h>
#include <MultiStepper.h>

int xPin = A2; // "Resume" pin on CNC shield
int xVal;


const int dirPin = 5;
const int stepPin = 2;
const int enablePin = 8;

unsigned long currentMillis;
long previousMillis = 0;    // Set up timer
long smoothInterval = 5;        // Time constant for timer

// Define motor interface type (type 1 means an external stepper driver with Step and Direction pins)
#define motorInterfaceType 1

// Creates an instance 
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);


void setup() {
  
  Serial.begin(9600);
  pinMode(xPin, INPUT);

  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);

  // set the maximum speed, acceleration factor,
  // initial speed and the target position
  myStepper.setMaxSpeed(1000);
 // myStepper.setAcceleration(100);
  
}

void loop() {

  static unsigned long timer = 0;
  unsigned long interval = 10;
   if (millis() - timer >= interval){
      timer = millis();
      xVal = analogRead(xPin);
      //Serial.println(xVal); // DEBUG
     
   }
 

  if (xVal >= 530){
   // myStepper.setSpeed(100);
 
    myStepper.setSpeed(2*(xVal-530));
    myStepper.run();
    
  }

  if (xVal <= 490){
  //  myStepper.setSpeed(-100);
  
    myStepper.setSpeed(2*(-1*(490-xVal)));
    myStepper.run();
    
  }

}
