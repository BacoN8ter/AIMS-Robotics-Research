#include <VarSpeedServo.h>
//#include <Servo.h>


 // create servo object to control a servo
// twelve servo objects can be created on most boards
 
int pos = 0;    // variable to store the servo position
VarSpeedServo myservo;
void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.begin(115200);
}

void loop() {
  //this code will switch back and forth between 0 and 180 degrees 
  //int val = analogRead(A1);
  //val = map(val,0,1023,0,180);
  myservo.write(90,50,true);
//  Serial.println(val);
  //delay(1000);
  //val = analogRead(A1);
  //val = map(val,0,1023,0,180);
  //myservo.write(10,50,true);
  //Serial.println(val);
  //delay(1000);
  delay(100);
}

