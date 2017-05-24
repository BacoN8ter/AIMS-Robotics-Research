#include <Servo.h>
#include <Encoder.h>
#include <MINDSi.h>

Servo drive, frontSteer;
int middle = 5;
int left = 6;
int right =7;
int turn = 45;
int dangerZone = 80; //cm

void setup() 
{
  pinMode(13,INPUT);
  pinMode(12,INPUT);
  frontSteer.attach(3);
  drive.attach(2);
  frontSteer.write(90);
  delay(2000);
}

void loop() 
{
 
  //if something is in the way of the robot
  if(getSonarDistance(left) < dangerZone || getSonarDistance(right) < dangerZone || getSonarDistance(middle) < dangerZone)
  {
    if(getSonarDistance(left) > getSonarDistance(right))//something is closer on the right side
    {
      frontSteer.write(90-turn); //turn left
      delay(2000);
      //frontSteer.write(90+turn);
      //delay(2000);
    }
    else if(getSonarDistance(left) < getSonarDistance(right))// something closer on the left side
    {
      frontSteer.write(90+turn);
      delay(2000);
      //frontSteer.write(90-turn);
     // delay(2000);
    }
    frontSteer.write(90);
  }
  
  drive.write(105);
}

float getSonarDistance(int sonarPin)
{
  float distance = (getPing(sonarPin)*.034)/2;
}
