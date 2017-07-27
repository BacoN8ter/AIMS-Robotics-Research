#include <NewPing.h>
#include <LIDARLite.h>
#include <CommunicationUtils.h>
#include <DebugUtils.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <FreeSixIMU.h>
#include <Wire.h>
#include <Servo.h>
#include <MINDSi.h>
#include "FSM.h"

void setup()
{
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  
  Serial.begin(115200);
  frontSteer.attach(13);
  drive.attach(3);

  //lidar baud causes twitchyness
  lidarSteer.attach(12);
  lidar.begin(0, true);
  lidar.configure(0);

  frontSteer.write(center);
  drive.write(90);
  lidarSteer.write(currentLidarData.lidarAngle);
  delay(2000);
  //sixDOF.init();
}


void loop() {

  //  sixDOF.getEuler(angles);//radians
  leftDist = getSonarDistance(left);
  rightDist = getSonarDistance(right);
  rightMidDist = getSonarDistance(rightMid);
  leftMidDist = getSonarDistance(leftMid);
  midDist = getSonarDistance(mid);
  rearDist = getSonarDistance(rear);

  unsigned long currentMillis = millis();
  switch (state)
  {
    case Idle:
      Serial.println("Beginning Obstacle Avoidance");
      state = ActiveNavigation;
      break;

    case ActiveNavigation:
      Serial.println("No obstacles detected. Moving forward");
      powerAdjustment = 0;
      
      if (getMode(leftDist, prevLeftDist) < dangerZone || getMode(rightDist, prevRightDist) < dangerZone ||
          getMode(leftMidDist, prevLeftMidDist) < dangerZone || getMode(rightMidDist, prevRightMidDist) < dangerZone || getMode(midDist, prevMidDist) < dangerZone)
      {
        state = ReactiveNavigation;
      }
      
     //active is untested 
      else if(farthestLidarValue.lidarDistX - lidarArray[sizeof(lidarArray)/2].lidarDistX > LIDAR_THRESHOLD)
      {
        //the farthest distance is on the right, apply a force on the left
        //positive error
        angleError = farthestLidarValue.lidarDistX - lidarArray[sizeof(lidarArray)/2];//the farther the X distance the more it should correct based on the Y distance. (closer -> more error)
        netAngleError += angleError * 0.025;//a random constant I came up with that needs to be tuned
      }
      else if(farthestLidarValue.lidarDistX - lidarArray[sizeof(lidarArray)/2].lidarDistX < -LIDAR_THRESHOLD)
      {
        //negative error
        angleError = farthestLidarValue.lidarDistX - lidarArray[sizeof(lidarArray)/2];//the farther the X distance the more it should correct based on the Y distance. (closer -> more error)
        netAngleError += angleError * 0.025;
      }
//      if (getMode(midDist, prevMidDist) < 10 && midDist != 0)
//      {
//        state = Stop;
//      }
      netAngleError = netAngleError < -45 ? -45 : netAngleError;
      netAngleError = netAngleError > 45 ? 45 : netAngleError;
      angle = (int)(angleKp * netAngleError);
      //angle < 90 turn to the left (right side triggered)
      //angle > 90 turn to the right (left side triggered) 
      frontSteer.write(center + angle);
      break;

    case ReactiveNavigation:
      Serial.println("Side collision detected");
      if (!proximityIsClear())
      {
        //30 degrees off horizontal
        if (getMode(leftDist, prevLeftDist) < dangerZone && getMode(leftDist,prevLeftDist) != 0) //change power based on which side is within the dangerZone
        {
          powerError = dangerZone - getMode(leftDist, prevLeftDist);
          angleError = dangerZone - getMode(leftDist, prevLeftDist);
          netAngleError += angleError * cos(0.523599); //get the x value to turn
          netPowerError += powerError * sin(0.523599); //get the y value to slow down
        } //30 degrees off straight
        if (getMode(leftMidDist, prevLeftMidDist) < dangerZone && getMode(leftMidDist,prevLeftMidDist) != 0)
        {
          powerError = dangerZone - getMode(leftMidDist, prevLeftMidDist) ;
          angleError = dangerZone - getMode(leftMidDist, prevLeftMidDist);
          netAngleError += angleError * cos(1.0472);
          netPowerError += powerError * sin(1.0472);
        }
        if (getMode(rightDist, prevRightDist) < dangerZone && getMode(rightDist,prevRightDist) != 0)
        {
          powerError = dangerZone - getMode(rightDist, prevRightDist) ;
          angleError = dangerZone - getMode(rightDist, prevRightDist);
          netAngleError += angleError * cos(2.0944);
          netPowerError += powerError * sin(2.0944);
        }
        if (getMode(rightMidDist, prevRightMidDist) < dangerZone && getMode(rightMidDist,prevRightMidDist) != 0)
        {
          powerError = dangerZone - getMode(rightMidDist, prevRightMidDist) ;
          angleError = dangerZone - getMode(rightMidDist, prevRightMidDist);
          netAngleError += angleError * cos(2.61799);
          netPowerError += powerError * sin(2.61799);
        }
        if (getMode(midDist, prevMidDist) < dangerZone && getMode(leftMidDist,prevLeftMidDist) != 0 && getMode(rightMidDist,prevRightMidDist) != 0 && getMode(midDist, prevMidDist) != 0)
        {
          if(compareSonarDist(getMode(leftMidDist, prevLeftMidDist), getMode(rightMidDist, prevRightMidDist)))//if the distances between the two are almost the same
          {
            //check the sides and turn to the more favorable one
              if (getMode(rightDist, prevRightDist) < getMode(leftDist,prevLeftDist) && getMode(rightDist,prevRightDist) != 0 && getMode(leftDist,prevLeftDist) != 0)
              {
                powerError = dangerZone - getMode(midDist, prevMidDist) ;
                angleError = dangerZone - getMode(rightDist, prevRightDist);
                netAngleError += angleError * cos(2.0944);
                netPowerError += powerError * sin(2.0944);
              }
              else if(getMode(leftDist, prevLeftDist) < dangerZone && getMode(leftDist,prevLeftDist) != 0) //change power based on which side is within the dangerZone
              {
                powerError = dangerZone - getMode(midDist, prevMidDist);
                angleError = dangerZone - getMode(leftDist, prevLeftDist);
                netAngleError += angleError * cos(0.523599); //get the x value to turn
                netPowerError += powerError * sin(0.523599); //get the y value to slow down
              }
          }
          
          else//one side is clearly longer more open than the other
          { 
            if (getMode(rightDist, prevRightDist) < dangerZone && getMode(rightDist,prevRightDist) != 0)
            {
              powerError = dangerZone - getMode(midDist, prevMidDist) ;
              angleError = dangerZone - getMode(rightDist, prevRightDist);
              netAngleError += angleError * cos(2.0944);
              netPowerError += powerError * sin(2.0944);
            }
            if (getMode(leftDist, prevLeftDist) < dangerZone && getMode(leftDist,prevLeftDist) != 0) //change power based on which side is within the dangerZone
            {
              powerError = dangerZone - getMode(midDist, prevMidDist);
              angleError = dangerZone - getMode(leftDist, prevLeftDist);
              netAngleError += angleError * cos(0.523599); //get the x value to turn
              netPowerError += powerError * sin(0.523599); //get the y value to slow down
            } 
          }
          //turn left
//          powerError = dangerZone - getMode(rightMidDist, prevRightMidDist) ;
//          angleError = dangerZone - getMode(rightMidDist, prevRightMidDist);
//          netAngleError += angleError * cos(3.14159);
//          netPowerError += powerError * sin(1.5708);
        }
//        else if (!compareSonarDist(getMode(leftMidDist, prevLeftMidDist), getMode(rightMidDist, prevRightMidDist)) && getMode(midDist, prevMidDist) < dangerZone && 
//                 getMode(leftMidDist,prevLeftMidDist) != 0 && getMode(rightMidDist,prevRightMidDist) != 0)
//        {
//          //turn right
//          powerError = dangerZone - getMode(leftMidDist, prevLeftMidDist) ;
//          angleError = dangerZone - getMode(leftMidDist, prevLeftMidDist);
//          netAngleError += angleError * cos(0);
//          netPowerError += powerError * sin(1.5708);
//        }
      }
      else
      {
        state = ActiveNavigation;
      }
      if (getMode(midDist, prevMidDist) < 10 && midDist != 0)
      {
        state = Stop;
      }
      netAngleError = netAngleError < -45 ? -45 : netAngleError;
      netAngleError = netAngleError > 45 ? 45 : netAngleError;
      angle = (int)(angleKp * netAngleError);
      //angle < 90 turn to the left (right side triggered)
      //angle > 90 turn to the right (left side triggered) 
      frontSteer.write(center + angle);
      break;
      
    case LidarUpdate:
      Serial.println("updating lidar");
      if(leftTrigger && rightTrigger)
      {
        state = ActiveNavigation;
        previousMillis = currentMillis;
      }
      if(currentLidarData.lidarAngle == minAngle+1)
      {
        rightTrigger = true;
      }
      if(currentLidarData.lidarAngle == minAngle + sweepDegrees-1)
      {
        leftTrigger = true;
      }
      break;
      
    case Stop:
      Serial.println("Imminent collision. Stopping");
      drive.write(0);
      delay(1000);
      drive.write(90);
      delay(2000);
      state = Reverse;
      //state = Straight;
      break;

    case Reverse:
      Serial.println("Creating Space");
      frontSteer.write(center);
      
      for(int i =0;i<100;i++)//this is bad should be changed later to be non blocking
      {
        drive.write(80);
        if(getMode(rearDist,prevRearDist) < 50)
        {
          break;
        }
        delay(10);
      }
      drive.write(90);
      delay(2000);
      state = ActiveNavigation;
      break;
  }
 
//  lidarProcess();
  if(currentMillis - previousMillis > 4000 && state != LidarUpdate)
  {
    state = LidarUpdate;
    leftTrigger = false;
    rightTrigger = false;
  }
  if(state == LidarUpdate)
  {
      drive.write(0);
  }
  else
  {
    powerAdjustment = (basePower - abs(powerKp * powerError)) > 100 ? (int)( powerKp * powerError) : 15 ; //minimum speed setting
    drive.write(basePower - abs(powerAdjustment));
  }
  turnServo();
  Serial.println(millis());
  powerAdjustment = 0;
  netAngleError = 0;
  angle =0;
  
  prevLeftDist = leftDist; // old dist = current distance
  prevRightDist = rightDist;
  prevRightMidDist = rightMidDist;
  prevLeftMidDist = leftMidDist;
  prevMidDist = midDist;
  prevRearDist = rearDist;
  delay(10);
}

Sonar setupSonar(int trigPin, int echoPin)
{
  Sonar temp;
  temp.trigPin = trigPin;
  temp.echoPin = echoPin;
  return temp;
}
float getAverage(int a, int b)
{
  return (a + b) / 2;
}
float getMode(int a, int b)
{
  return a > b ? (float)a : (float)b;
}
float getSonarDistance(int sonarPin)
{
  return (getPing(sonarPin) * .034) / 2;
}

float getSonarDistance(Sonar sonar)
{
  digitalWrite(sonar.trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(sonar.trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(sonar.trigPin, LOW);
  sonar.duration = pulseIn(sonar.echoPin, HIGH);
 // sonar.dist = (sonar.duration / 2) / 29.1;
  sonar.dist = (sonar.duration*.034)/2;
  return (float)(sonar.dist);
}

float getSonarDistance(NewPing sonar)
{
  int tempDist = sonar.ping_cm();
  for(int i =0;i<100;i++)
  {
    if(tempDist == 0)
    {
      tempDist = sonar.ping_cm();
    }
    else
    {
      return tempDist;
    }
  }
  return tempDist;
}


bool compareSonarDist(float dist1, float dist2)
{
  if(abs(dist1-dist2) < sonarThreshold)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool proximityIsClear()
{
  if (getAverage(leftDist, prevLeftDist) < dangerZone || getAverage(leftMidDist, prevLeftMidDist) < dangerZone ||
      getAverage(rightDist, prevRightDist) < dangerZone || getAverage(rightMidDist, prevRightMidDist) < dangerZone || getAverage(midDist, prevMidDist) < dangerZone) //change power based on which side is within the dangerZone
  {
    return false;
  }
  else
  {
    return true;
  }
}

void turnServo()
{
  if (currentLidarData.lidarAngle > minAngle+sweepDegrees)
  {
    lidarSpeed = -abs(lidarSpeed);
  }
  else if (currentLidarData.lidarAngle < minAngle)
  {
    lidarSpeed = abs(lidarSpeed);
  }
  currentLidarData.lidarAngle += lidarSpeed;
  lidarSteer.write(currentLidarData.lidarAngle);
}
void lidarProcess()
{
  lidarArray[currentLidarData.lidarAngle - minAngle] = currentLidarData;//fill the array as it moves in the arc
  if(lidarArray[currentLidarData.lidarAngle - minAngle].lidarDist > farthestLidarValue.lidarDist)//save the farthest distance
  {
    farthestLidarValue.lidarDist = lidarArray[currentLidarData.lidarAngle].lidarDist;
    farthestLidarValue.lidarAngle = lidarArray[currentLidarData.lidarAngle].lidarAngle;
    farthestLidarValueIndex = currentLidarData.lidarAngle - minAngle;
  }
}




