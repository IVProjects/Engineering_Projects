#include<Servo.h>

int bPin = 3;
int bState;

int tPin = 10;
int tState;

int servoPos;
int restPos = 170;
int gripPos = 20;

Servo servo1;

void setup() {

  Serial.begin(9600);
  pinMode(bPin,INPUT); // User Gripper Pushbutton
  digitalWrite(bPin,HIGH);
  
  pinMode(tPin,INPUT); // Tension Sensor
  digitalWrite(tPin,HIGH);

 servo1.attach (7); // Declare Servo Object
 servoPos = 160;
}

void loop() {
  // put your main code here, to run repeatedly:

  tState = digitalRead(tPin); // Read Tension Sensor
  bState = digitalRead(bPin);

  //////////DEBUG:
 /* Serial.print ("tState   ");
  Serial.println(tState);   
  Serial.print ("bState   ");
  Serial.print(bState);
*/
  
  
  if (bState == LOW && tState == HIGH) // Gripper conditions
  {
    if(servoPos > 35){
      servoPos--;
    }
    
    servo1.write(servoPos);
    delay(2);
  }

  else if (bState == LOW && tState == LOW)
  {
    servo1.write(servoPos);
    delay(10);
  }

  else if (bState == HIGH)
  {
    servo1.write(restPos);
    servoPos = 160;
    delay(10);
  }

}
