#include "VarSpeedServo.h"

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
}
VarSpeedServo servo;
void loop() {
  // put your main code here, to run repeatedly:
  int val = digitalRead(A1);
    //val = map(val,0,1023,0,180);
    Serial.println(val);
  
  delay(100);
}
