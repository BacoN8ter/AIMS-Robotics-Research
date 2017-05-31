#include <Servo.h>
#include <MINDSi.h>

Servo drive, frontSteer;
const int middle = 5;
const int left = 6;
const int right = 7;
const int center = 80;
const float angleKp = 0.9;
const float powerKp = 0.2;
int dangerZone = 100; //all distance readings are in cm
int proximityTolerance = 100;
int angle = center;
int power = 105;
int angleError = 0;
int basePower = 120;
int powerError =0;
int powerAdjustment =0;

float getSonarDistance(int sonarPin);

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
int prevLeftDist = getSonarDistance(left);
int prevRightDist = getSonarDistance(right);
void loop() 
{
 // Serial.print("middle: ");
 // Serial.println(averageArray(distanceArrayMid,middle));
  Serial.print("left: ");
  Serial.println(getSonarDistance(left));
  //Serial.print("right: ");
  //Serial.println(averageArray(distanceArrayRight,right));
  
  if(getSonarDistance(middle) > 10) //if you are not going to hit something
  {
    powerAdjustment =0;
    if(getAverage(getSonarDistance(left),prevLeftDist) < dangerZone || getAverage(getSonarDistance(right),prevRightDist) < dangerZone)// if something is to the side of the robot
    {
      angleError = getAverage(getSonarDistance(right),prevRightDist) - getAverage(getSonarDistance(left),prevLeftDist);
      angleError = angleError<-45?-45:angleError;
      angleError = angleError>45?45:angleError;
      angle = (int)(angleKp * angleError);
      frontSteer.write(center+angle);
      
      if(getAverage(getSonarDistance(left),prevLeftDist) < dangerZone) //change power based on which side is within the dangerZone
      {
        powerError = dangerZone-getAverage(getSonarDistance(left),prevLeftDist) ;
        powerAdjustment = (int)( powerKp * powerError);
        
      }
      else if(getAverage(getSonarDistance(right),prevRightDist) < dangerZone)
      {
        powerError = dangerZone-getAverage(getSonarDistance(right),prevRightDist) ;
        powerAdjustment = (int)( powerKp * powerError);
      }
      
    }

   else if(getSonarDistance(middle) < 80 )//if something is in the way of the robot (front)
    {
      if(getAverage(getSonarDistance(left),prevLeftDist) > getSonarDistance(right) && (getAverage(getSonarDistance(right),prevRightDist) < proximityTolerance || getAverage(getSonarDistance(left),prevLeftDist) < proximityTolerance))//something is closer on the right side
      {
        frontSteer.write(center-45); //turn left
      }
      else 
      {
        frontSteer.write(center+45);//turn right
      }
    } 
    else
    {
      frontSteer.write(center);
    }

    drive.write(basePower-abs(powerAdjustment));
    
    prevLeftDist = getSonarDistance(left); // old dist = new distance
    prevRightDist = getSonarDistance(right);
  } 
//  else
//  {
//      drive.write(0);
//      delay(1000);
//      drive.write(90);
//      delay(2000);
//      
//      frontSteer.write(center);
//      drive.write(70);
//      delay(1000);
//      
//      drive.write(90);
//      delay(2000);     
//  }
  
}
float getAverage(int a, int b)
{
  return (a+b)/2;
}
float getSonarDistance(int sonarPin)
{
  return (getPing(sonarPin)*.034)/2;
}
