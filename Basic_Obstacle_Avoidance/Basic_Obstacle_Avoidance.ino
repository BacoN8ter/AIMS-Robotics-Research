#include <Servo.h>
#include <Encoder.h>
#include <MINDSi.h>

Servo drive, frontSteer;
int middle = 5;
int left = 6;
int right =7;
int turn = 45;
int center = 80;
int dangerZone = 80; //cm
int proximityTolerance = 100;

void setup() 
{
  pinMode(13,INPUT);
  pinMode(12,INPUT);
  frontSteer.attach(3);
  drive.attach(2);
  frontSteer.write(center);
  delay(2000);
}

void loop() 
{
  Serial.print("middle");
 Serial.println(getSonarDistance(middle));
 
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
  
  drive.write(102);
}

float getSonarDistance(int sonarPin)
{
  return (getPing(sonarPin)*.034)/2;
}
