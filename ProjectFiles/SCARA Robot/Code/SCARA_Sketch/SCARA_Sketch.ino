#include <AccelStepper.h>
#include <Servo.h>

String value;
const char delim = ';';
int data[5]; //Holds most recent angles for first and second joint, followed by the z-height
int* angles;
int* height;
int* gripper;

char buffer[5];
int bufferIndex;

int dataIndex = 0;

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

int limitPin = 12;


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

bool target_updated;

// Define motor interface type (type 1 means an external stepper driver with Step and Direction pins)
#define motorInterfaceType 1

// Define steppers and the pins they use
AccelStepper stepper1(motorInterfaceType, stepPin1, dirPin1);// First Joint
AccelStepper stepper2(motorInterfaceType, stepPin2, dirPin2);// Second Joint
AccelStepper stepper3(motorInterfaceType, stepPin3, dirPin3);//Third Joint (end effector rotation)
AccelStepper stepper4(motorInterfaceType, stepPin4, dirPin4);// Z-axis

bool finished[5];

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
  Serial.setTimeout(1); 

  servo1.attach (A2); // Declare Servo Object (A2 = "Resume" pin)
 

  angles = data;
  height = data+3;
  gripper = data+4;
  
  bufferIndex = 0;
  target_updated = false;
  

  for (int i = 0; i < sizeof(finished)/sizeof(*finished); i++)
  {
    finished[i] = true;
  }

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

  stepper1.setMaxSpeed(5000); //// From Accelstepper: "Max possible speed depends on clock speed. Caution: Speeds that exceed the maximum speed supported by the processor may Result in non-linear accelerations and decelerations"
  stepper1.setAcceleration(5000);

  stepper2.setMaxSpeed(5000);
  stepper2.setAcceleration(5000);

  stepper3.setMaxSpeed(8000);
  stepper3.setAcceleration(5000);

  stepper4.setMaxSpeed(3000);
  stepper4.setAcceleration(3000);

  homing(); ///////////////////////// RUN ARM HOMING FUNCTION


  stepper1.setCurrentPosition(0); // Set current stepper position after homing and becoming straight as zero (THIS SETS MOTOR SPEED TO 0)
  stepper2.setCurrentPosition(0); 
  stepper3.setCurrentPosition(0); 
  stepper4.setCurrentPosition(0); 

  
  stepper1.move(stepsPerDegree*joint1Home); // Moves joint one to straight ahead (arm will crash if signs are wrong)
  stepper2.move(stepsPerDegree*joint2Home);
  stepper3.move(stepsPerDegree3*joint3Home);
 // stepper4.move(stepsPermm*zHome);

  while (stepper1.distanceToGo() != 0){
    stepper1.run();
  }

  while (stepper2.distanceToGo() != 0){
    stepper2.run();
  }

  while (stepper3.distanceToGo() != 0){
    stepper3.run();
  }
  /*
  while (stepper4.distanceToGo() != 0){
    stepper4.run();
  }
*/
  stepper1.setCurrentPosition(0); // Set current stepper position after homing and becoming straight as zero (THIS SETS MOTOR SPEED TO 0)
  stepper2.setCurrentPosition(0); 
  stepper3.setCurrentPosition(0); 
  stepper4.setCurrentPosition(0); 

  servo1.write(gripPos);
  delay(1000);
  servo1.write(openPos);
  delay(1000);
 
} // Setup End

void readBuffer()
{
  if (Serial.available())
  {
    char c = Serial.read();
    buffer[bufferIndex] = c;
    bufferIndex++;

  }
}

void parseBuffer()
{
  static int boundary1 = 110;
  static int boundary2 = 135;
  static int boundary3 = 120;
  
  if (buffer[bufferIndex-1] == ';')
  {
    /*
    for (int i = 0; i < sizeof(finished)/sizeof(*finished); i++)
    {
      finished[i] = false;
    }*/
    
    target_updated = true;
  
    buffer[bufferIndex-1] = '\0';

    int newData = atoi(buffer);

    boolean changed = (data[dataIndex] != newData);
    
    data[dataIndex] = newData;

    if (data[0] < -boundary1)
    {
      data[0] = -boundary1;
      changed = false;
    }

    else if (data[0] > boundary1)
    {
      data[0] = boundary1;
      changed = false;
    }

    if (data[1] < -boundary2)
    {
      data[1] = -boundary2;
      changed = false;
    }

    else if (data[1] > boundary2)
    {
      data[1] = boundary2;
      changed = false;
    }
    
    if (data[2] < -boundary3)
    {
      data[2] = -boundary3;
      changed = false;
    }

    else if (data[2] > boundary3)
    {
      data[2] = boundary3;
      changed = false;
    } 
    
    if (changed)
      finished[dataIndex] = false;
    
    dataIndex++;
    dataIndex %= sizeof(data)/sizeof(*data);
        
    bufferIndex = 0;

    for (int j = 0; j < sizeof(buffer)/sizeof(*buffer); j++)
    {
      buffer[j] = 1;
    }
    
  }
}



void loop() {
  static unsigned long last_update = millis();
  static bool moving = false;
  static unsigned long gripTimer = millis();
  static unsigned long loopTimer = 0;
  
  // put your main code here, to run repeatedly:
  readBuffer();
  parseBuffer(); 

  if (millis() - last_update > 250)
  {       
    last_update = millis();
    target_updated = false;
    
    stepper1.moveTo((angles[0])*stepsPerDegree);  
    stepper2.moveTo(-(angles[1])*stepsPerDegree);    
    stepper3.moveTo((angles[2])*stepsPerDegree3);
    stepper4.moveTo(stepsPermm*-(*height));  
    
    finished[0] = (stepper1.distanceToGo() == 0);
    finished[1] = (stepper2.distanceToGo() == 0);
    finished[2] = (stepper3.distanceToGo() == 0);
    finished[3] = (stepper4.distanceToGo() == 0);

    if (finished[0] && finished[1] && finished[2] && finished[3] && finished[4])
    {     
      if (moving)
        Serial.println("Ready"); //Actually does something
      moving = false;
    }
    else
      moving = true;    

    /*
    Serial.print(finished[0]);
    Serial.print(", ");
    Serial.print(finished[1]);
    Serial.print(", ");
    Serial.print(finished[2]);
    Serial.print(", ");
    Serial.print(finished[3]);
    Serial.print(", ");
    Serial.println(finished[4]);*/

    /*
    Serial.print(angles[0]);
    Serial.print(", ");
    Serial.print(angles[1]);
    Serial.print(", ");
    Serial.print(angles[2]);
    Serial.print(", ");
    Serial.print(*height);
    Serial.print(", ");
    Serial.println(*gripper);*/
    
  }

  if (gripTimer <= millis())
  {
    if (!finished[0])
      stepper1.run();
  
    else if (!finished[1])
      stepper2.run();
  
    else if (!finished[2])
      stepper3.run();
      
    else if (!finished[3])
      stepper4.run();
  
    else if (!finished[4])
     {
      if (*gripper == 0)
      {
        servo1.write(openPos);
        //Serial.println("Open");
      }
      else
      {
        servo1.write(gripPos);
        //Serial.println("Close");
      }
  
       finished[4] = true;
       gripTimer = millis() + 1000;
     }
  }

  boolean readyToRemove = digitalRead(limitPin);

  if (readyToRemove && loopTimer == 0)
  {
    loopTimer = millis() + 1000 * 60 * 5;
  }

  if (loopTimer != 0 && millis() > loopTimer)
  {
    loopTimer = 0;
    Serial.write("Replay");
  }

}
