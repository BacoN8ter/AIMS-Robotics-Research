#include <Servo.h>
#include <MINDSi.h>

/***************************************************
/ Example provided by MINDS-i
/ Try checking out our arduino resource guide at
/ http://mindsieducation.com/programming-resources
/ Questions? Concerns? Bugs? email code@mymindsi.com
/
/ This code expects a ping sensor plugged into pin 10
/ Or, Swap out the comments below to read three sensors
/ on pins 9,10, and 11
/***************************************************/
#define echo 7
float time = 0, duration = 0, distance = 0;

void setup() {
  //start a serial connection
  Serial.begin(9600);
}

void loop() {
  //send the value read by the ping sensor out serial
  Serial.println( getPing(7) );

  //optional code for displaying three QTI sensors, make sure the previous code
  //is commented out when using the code below.
  /*
    Serial.print(getPing(9));
    Serial.print("\t"); //insert some tabs
    Serial.print(getPing(10));
    Serial.print("\t");
    Serial.println(getPing(11));
  */
  //time = pulseln(echo, HIGH);
  duration = (getPing(7));
  distance  = duration * .034/2;

  Serial.print("Distance: ");
  Serial.println(distance);
}
