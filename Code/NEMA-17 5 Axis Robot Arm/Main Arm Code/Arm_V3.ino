#include <AccelStepper.h>
#include <Servo.h>
int potPin1 = A2; // Resume A2
int potPin2 = A1; // Hold
int potPin3 = A0; // Abort
int potPin4 = A3; // Coolant Enable

unsigned long pot1; // Values from potentiometers
unsigned long pot2;
unsigned long pot3;
unsigned long pot4;

int bPin = 11; // User Pushbutton for gripper
int bState;

int pot1Scaled; // Potentiometer values mapped to relevant angles for desired range of allowable joint angles
int pot2Scaled;
int pot3Scaled;
int pot4Scaled;
int bStateScaled;



/*
#define WINDOW_SIZE 10

int INDEX1 = 0;
int VALUE1 = 0;
int SUM1 = 0;
int READINGS1[WINDOW_SIZE];
int AVERAGED1 = 0;

int INDEX2 = 0;
int VALUE2 = 0;
int SUM2 = 0;
int READINGS2[WINDOW_SIZE];
int AVERAGED2 = 0;

int INDEX3 = 0;
int VALUE3 = 0;
int SUM3 = 0;
int READINGS3[WINDOW_SIZE];
int AVERAGED3 = 0;

int INDEX4 = 0;
int VALUE4 = 0;
int SUM4 = 0;
int READINGS4[WINDOW_SIZE];
int AVERAGED4 = 0;
*/


int restPos = 5; // Gripper servo positions
int gripPos = 60;


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
int limit3 = 11; // Z
int limit4 = A4; // SDA (SDA seems connected to both A4 and A5)

int limState1;
int limState2;
int limState3;
int limState4;

long homing1 = -1; // Make (-) for CCW homing, and (+) for CW homing (also need to change increment in homing function)
long homing2 = 1;
long homing3 = -1;
long homing4 = 1;


// Define motor interface type (type 1 means an external stepper driver with Step and Direction pins)
#define motorInterfaceType 1

// Define steppers and the pins they use
AccelStepper stepper1(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2(motorInterfaceType, stepPin2, dirPin2);
AccelStepper stepper3(motorInterfaceType, stepPin3, dirPin3);
AccelStepper stepper4(motorInterfaceType, stepPin4, dirPin4);


////// HOMING FUCNTION:
void homing(){ 

  
  limState1 = digitalRead(limit1);
  limState2 = digitalRead(limit2);
  limState3 = digitalRead(limit3);
  limState4 = digitalRead(limit4);

  while (limState1 == 1){
    stepper1.moveTo(homing1);
    homing1--;
    stepper1.run();
    delay(1);
    limState1 = digitalRead(limit1);
    //Serial.println(limState1);
  }
  delay (500);


    while (limState2 == 1){
    stepper2.moveTo(homing2);
    homing2++;
    stepper2.run();
    limState2 = digitalRead(limit2);
    delay(1);
    //Serial.println(limState2);
  }
  delay (500);
  

    while (limState3 == 1){
    stepper3.moveTo(homing3);
    homing3--;
    stepper3.run();
    limState3 = digitalRead(limit3);
    delay(2);
    //Serial.println(limState3);
  }
  delay (500);

    while (limState4 == 1){
    stepper4.moveTo(homing4);
    homing4++;
    stepper4.run();
    limState4 = digitalRead(limit4);
    delay(6);
    //Serial.println(limState4);
  }  
  delay (500); 
} /// Homing function end



void setup() {

  Serial.begin(9600);
  
  // Potentiometer input pins
  pinMode(potPin1, INPUT); // Resume
  pinMode(potPin2, INPUT); // Hold
  pinMode(potPin3, INPUT); // Abort
  pinMode(potPin4, INPUT); // Coolant Enable

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

  stepper1.setMaxSpeed(4000); //// From Accelstepper: "Max possible speed depends on clock speed. Caution: Speeds that exceed the maximum speed supported by the processor may Result in non-linear accelerations and decelerations"
  stepper1.setAcceleration(5500);

  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(5500);

  stepper3.setMaxSpeed(4000);
  stepper3.setAcceleration(5500);

  stepper4.setMaxSpeed(1000);
  stepper4.setAcceleration(3000);



  homing(); ///////////////////////// RUN ARM HOMING FUNCTION



  stepper1.setCurrentPosition(0); // Set current stepper position after homing as zero (THIS SETS MOTOR SPEED TO 0)
  stepper2.setCurrentPosition(0); 
  stepper3.setCurrentPosition(0); 
  stepper4.setCurrentPosition(0); 

/////////// OFFSET JOINTS FROM LIMIT SWITCHES AFTER HOMING:
  
  stepper1.moveTo(200);
  for (int i = 0; i < 200 ; i++) { 
    stepper1.run();
    delay(10);
  }

  stepper2.moveTo(-400);
  for (int i = 0; i < 400 ; i++) { 
    stepper2.run();
    delay(10);
  }

  stepper3.moveTo(200);
  for (int i = 0; i < 400 ; i++) { 
    stepper3.run();
    delay(10);
  }

    stepper4.moveTo(-60);
  for (int i = 0; i < 60 ; i++) { 
    stepper4.run();
    delay(10);
  }
  
} /// End of joint offsetting


void loop() { // Start of the main loop

//////////////// INPUT READING AND SCALING:
  static unsigned long timer = 0;
  unsigned long interval = 200;
   if (millis() - timer >= interval){
      timer = millis();
      
   
      for(int i=0; i< 10; i++){ 
      pot1 += analogRead(potPin1);
      }
      pot1 /= 10;
     
      pot1 = 1023-pot1;
      

      for(int i=0; i< 10; i++){ 
      pot2 += analogRead(potPin2);
      }
      pot2 /= 10;
      
      if(pot2 < 340){ // Bottom limit on shoulder travel
        pot2 = 340;
      }
      

      for(int i=0; i< 10; i++){ 
      pot3 += analogRead(potPin3);
      }
      pot3 /= 10;
      
      if(pot3 > 885){ // Bottom limit on elbow travel (bent 90)
        pot3 = 885;
      }

      if(pot3 < 600){ // Upper limit on elbow travel (straight)
        pot3 = 600;
      }

      for(int i=0; i< 10; i++){ 
      pot4 += analogRead(potPin4);
      }
      pot4 /= 10;
     
    //  if(pot4 < 360){ // Bottom limit on wrist travel
       // pot4 = 360;
      


  /*    SUM1 = SUM1 - READINGS1[INDEX1];
      VALUE1 = analogRead(potPin1);
      READINGS1[INDEX1] = VALUE1;
      SUM1 = SUM1 + VALUE1;
      INDEX1 = (INDEX1 + 1) % WINDOW_SIZE;

      pot1 = SUM1 / WINDOW_SIZE;


      
      SUM2 = SUM2 - READINGS2[INDEX2];
      VALUE2 = analogRead(potPin2);
      READINGS2[INDEX2] = VALUE2;
      SUM2 = SUM2 + VALUE2;
      INDEX2 = (INDEX2 + 1) % WINDOW_SIZE;

      pot2 = SUM2 / WINDOW_SIZE;


      
      SUM3 = SUM3 - READINGS3[INDEX3];
      VALUE3 = analogRead(potPin3);
      READINGS3[INDEX3] = VALUE3;
      SUM3 = SUM3 + VALUE3;
      INDEX3 = (INDEX3 + 1) % WINDOW_SIZE;

      pot3 = SUM3 / WINDOW_SIZE;


      
      SUM4 = SUM4 - READINGS4[INDEX4];
      VALUE4 = analogRead(potPin4);
      READINGS4[INDEX4] = VALUE4;
      SUM4 = SUM4 + VALUE4;
      INDEX4 = (INDEX4 + 1) % WINDOW_SIZE;

      pot4 = SUM4 / WINDOW_SIZE;
*/
     
   }


      limState1 = digitalRead(limit1);
      limState2 = digitalRead(limit2);
      limState3 = digitalRead(limit3);
      limState4 = digitalRead(limit4);
   
  /*
    Serial.println(pot1);    //// DEBUG (controller arm pot values)
    Serial.print("   Pot2:  ");
    Serial.print(pot2);
    Serial.print("   Pot3:  ");
    Serial.print(pot3);
    Serial.print("   Pot4:  ");
    Serial.print(pot4);
    Serial.print("   Pot1:  ");
    delay(200);
  */  

   /* Serial.println(limState1);
    Serial.print("   Lim2:  ");
    Serial.print(limState2);
    Serial.print("   Lim3:  ");
    Serial.print(limState3);
    Serial.print("   Lim4:  ");
    Serial.print(limState4);
    Serial.print("   Lim1:  ");
*/

    
   
    pot1Scaled = map(pot1 , 160. , 850. , 0. , 175); // Mapping potentiometer values to number of steps
    pot2Scaled = 175- map(pot2 , 250. , 700. , 0. , 175);
    pot3Scaled = map(pot3 , 400. , 800. , 0. , 175);
    pot4Scaled = map(pot4 , 0. , 850. , 0. , 175);
    bStateScaled = map(bState , 0 , 1 , restPos , gripPos);
   

   /* 
    Serial.println(pot1Scaled);    //// DEBUG (Stepper motor positions)
    Serial.print("  Pos 2:  ");
    Serial.print(pot2Scaled);
    Serial.print("  Pos 3:  ");
    Serial.print(pot3Scaled);
    Serial.print("  Pos 4:  ");
    Serial.print(pot4Scaled);
    Serial.print("    Pos 1:  ");
      delay(100);
*/

 /////////// STEPPER STEP COMMANDS:

  if (pot1Scaled <0){  // Exclude negative values
    pot1Scaled =0;
  }

   if (pot2Scaled <0){
    pot2Scaled =0;
  }

   if (pot3Scaled <0){
    pot3Scaled =0;
  }

   if (pot4Scaled <0){
    pot4Scaled =0;
  }

  if (pot1Scaled < 100000) { 
    stepper1.moveTo(pot1Scaled*60);
    stepper1.run();

    if(limState1 == LOW){
      pot1Scaled =0;
    }
  }

  if (pot2Scaled < 100000){  
    stepper2.moveTo(-pot2Scaled*104);
    stepper2.run();
    
    if(limState2 == LOW){
      pot2Scaled =0;
    }
  }

  if (pot3Scaled < 10000){  
    stepper3.moveTo((pot3Scaled*25)-1900);
    stepper3.run();

     if(limState3 == LOW){
      pot3Scaled =1900;
    }
  }

  if (pot4Scaled < 10000){  
    stepper4.moveTo((-pot4Scaled*10)-300);
    stepper4.run();

      if(limState4 == LOW){
      pot4Scaled =0;
    }
  }
 
}
