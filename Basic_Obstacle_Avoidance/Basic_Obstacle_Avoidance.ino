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
  Serial.print("middle");
 Serial.println(getSonarDistance(middle));
  //if something is in the way of the robot
  if(getSonarDistance(left) < dangerZone || getSonarDistance(right) < dangerZone)
  {
    if(getSonarDistance(left) > getSonarDistance(right) && (getSonarDistance(right) < 100 || getSonarDistance(left) < 100))//something is closer on the right side
    {
      frontSteer.write(90-turn); //turn left
      //frontSteer.write(90+turn);
      //delay(2000);
    }
    else if(getSonarDistance(left) < getSonarDistance(right) && (getSonarDistance(left) < 100 || getSonarDistance(right) < 100) )// something closer on the left side
    {
      frontSteer.write(90+turn);
      //frontSteer.write(90-turn);
     // delay(2000);
    }
   
  }
  else if(getSonarDistance(middle) < dangerZone)
  {
    if(getSonarDistance(left) > getSonarDistance(right))//something is closer on the right side
    {
      frontSteer.write(90-turn); //turn left
    }
    else if(getSonarDistance(left) < getSonarDistance(right) )// something closer on the left side
    {
      frontSteer.write(90+turn);
    }
  } 
  else
    {
      frontSteer.write(90);
    }
  
  drive.write(102);
}

float getSonarDistance(int sonarPin)
{
  return (getPing(sonarPin)*.034)/2;
}
