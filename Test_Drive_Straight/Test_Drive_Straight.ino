#include <Servo.h>
#include <MINDSi.h>

Servo drive,frontSteer;
void setup() {
  // put your setup code here, to run once:
  pinMode(13, INPUT);
  pinMode(12, INPUT);

  frontSteer.attach(13);
  drive.attach(3);
  frontSteer.write(45);//orientates the servo to be straight forward
  drive.write(90);
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
 
  drive.write(130);//0 stop. 90 neutral. 0-180 straight power
  
}
