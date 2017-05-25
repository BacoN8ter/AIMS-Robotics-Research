#include <Servo.h>
#include <Encoder.h>
#include <MINDSi.h>

Servo drive, frontSteer;
int middle = 5;
int left = 6;
int right = 7;
int turn = 45;
int center = 80;
int dangerZone = 80; //cm
int proximityTolerance = 100;

void setup() 
{
  pinMode(13,INPUT);
  pinMode(12,INPUT);
  frontSteer.attach(3);
  drive.attach(9);
  frontSteer.write(center);
  drive.write(90);
  delay(2000);
}

void loop() 
{
  
 
  if(getSonarDistance(middle) > 20)
  {
    drive.write(110);
    if(getSonarDistance(left) < dangerZone || getSonarDistance(right) < dangerZone)// if something is to the side of the robot
   {
      if(getSonarDistance(left) > getSonarDistance(right) && (getSonarDistance(right) < proximityTolerance || getSonarDistance(left) < proximityTolerance))//something is closer on the right side
      {
        frontSteer.write(center-turn); //turn left
      }
      else if(getSonarDistance(left) < getSonarDistance(right) && (getSonarDistance(left) < proximityTolerance || getSonarDistance(right) < proximityTolerance) )// something closer on the left side
      {
        frontSteer.write(center+turn);//turn right
      }
    }

   else if(getSonarDistance(middle) < dangerZone )//if something is in the way of the robot (front)
    {
      if(getSonarDistance(left) > getSonarDistance(right) && (getSonarDistance(right) < proximityTolerance || getSonarDistance(left) < proximityTolerance))//something is closer on the right side
      {
        frontSteer.write(center-turn); //turn left
      }
      else 
      {
        frontSteer.write(center+turn);//turn right
      }
    } 
    else
    {
      frontSteer.write(center);
    }
  } 
  else
  {
      //drive.write(0);
      //delay(1000);
      frontSteer.write(center);
      drive.write(70);
      delay(1000);
     // drive.write(0);
  }
  
}



float getSonarDistance(int sonarPin)
{
  return (getPing(sonarPin)*.034)/2;
}
