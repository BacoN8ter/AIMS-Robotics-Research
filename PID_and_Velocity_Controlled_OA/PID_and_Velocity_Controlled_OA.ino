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
float distanceArrayMid[10];
float distanceArrayLeft[10];
float distanceArrayRight[10];
float distanceAverage;
int angle = center;
int power = 105;
int angleError = 0;
int basePower = 120;
int powerError =0;
int powerAdjustment =0;

float getSonarDistance(int sonarPin);
void lowPass(float *numbers[],int sonarPin);

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
  
  lowPass(&distanceArrayLeft,left);
  lowPass(&distanceArrayRight,right);
  lowPass(&distanceArrayMid,middle);
  
 // Serial.print("middle: ");
 // Serial.println(averageArray(distanceArrayMid,middle));
 // Serial.print("left: ");
 // Serial.println(averageArray(distanceArrayLeft,left));
  //Serial.print("right: ");
  //Serial.println(averageArray(distanceArrayRight,right));
  
  if(averageArray(distanceArrayMid,middle) > 10) //if you are not going to hit something
  {
    if(averageArray(distanceArrayLeft,left) < dangerZone || averageArray(distanceArrayRight,right) < dangerZone)// if something is to the side of the robot
    {
      angleError = getSonarDistance(right) - getSonarDistance(left);
      angleError = angleError<-45?-45:angleError;
      angleError = angleError>45?45:angleError;
      angle = (int)(angleKp * angleError);
      frontSteer.write(center+angle);
      
      if(averageArray(distanceArrayLeft,left) < dangerZone) //change power based on which side is within the dangerZone
      {
        Serial.println("this is running");
        powerError = dangerZone-averageArray(distanceArrayLeft,left) ;
        powerAdjustment = (int)( powerKp * powerError);
        
      }
      else if(averageArray(distanceArrayRight,right) < dangerZone)
      {
        powerError = dangerZone-averageArray(distanceArrayRight,right) ;
        powerAdjustment = (int)( powerKp * powerError);
      }
      else
      {
        powerAdjustment = 0;
      }
    }

   else if(averageArray(distanceArrayMid,middle) < 80 )//if something is in the way of the robot (front)
    {
      if(averageArray(distanceArrayRight,right) > averageArray(distanceArrayLeft,left) && (averageArray(distanceArrayRight,right) < proximityTolerance || averageArray(distanceArrayLeft,left) < proximityTolerance))//something is closer on the right side
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
    Serial.println(basePower-abs(powerAdjustment));
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


float averageArray(float numbers[], int sonarPin)
{
  for(int i = sizeof(numbers)-1; i>0; i--)
  {
    numbers[i] = numbers[i-1];
  }
  numbers[0] = getSonarDistance(sonarPin);
  
  float average=0;
  for(int i = 0;i<sizeof(numbers);i++)
  {
    average= average + numbers[i];
  }
  return average/sizeof(numbers);
}

void lowPass(float (*numbers)[10], int sonarPin) //this works as long as something doesnt randomly appear in front of it really close
{
  int low = 100;
  for(int i = sizeof(numbers)-1; i>0; i--)//shift array to make way for new sonar reading
  {
    *numbers[i] = *numbers[i-1];
  }
  *numbers[0] = getSonarDistance(sonarPin);
  
  //check if anything changes dramatically and replace it
  for(int i = 1; i< sizeof(numbers);i++)
  {
    if(abs(numbers[i] - numbers[i]-1) > low)
    {
      if(numbers[i] > numbers[i-1])
      {
        *numbers[i-1] = *numbers[i];
      }
      else
      {
        *numbers[i] = *numbers[i-1];
      }
    }
  }
}

float getSonarDistance(int sonarPin)
{
  return (getPing(sonarPin)*.034)/2;
}
