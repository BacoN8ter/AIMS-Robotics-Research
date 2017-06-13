#include <LIDARLite.h>
#include <CommunicationUtils.h>
#include <DebugUtils.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <FreeSixIMU.h>
#include <Wire.h>
#include <Servo.h>
#include <MINDSi.h>
LIDARLite myLidarLite;
Servo drive, frontSteer;

typedef enum
{
  Idle,
  Straight,
  SideAlarm,
  FrontAlarm,
  Stop,
  Reverse
}State;

typedef struct
{
  int echoPin;
  int trigPin;
  double dist;
  double duration;
}Sonar;

float getSonarDistance(int sonarPin);
float getSonarDistance(Sonar sonar);
Sonar setupSonar(int trigPin, int echoPin);

Sonar leftMid = setupSonar(6,7);
Sonar rightMid = setupSonar(4,5);
Sonar mid = setupSonar(8,9);
const int right = 10;
const int left = 11;

const int center = 80;//degrees
float angleKp = 0.9;
float powerKp = 0.2;
int dangerZone = 100; //all distance readings are in cm
int proximityTolerance = 100;
int angle = center;
int power = 105;
int angleError = 0;
int basePower = 120;
int powerError =0;
int powerAdjustment =0;
//IMU plugged into A4 (SDA) and A5 (SCL)

int prevLeftDist = getSonarDistance(left);
int prevRightDist = getSonarDistance(right);
int prevRightMidDist = getSonarDistance(rightMid);
int prevLeftMidDist = getSonarDistance(leftMid);
int prevMidDist = getSonarDistance(mid);

int leftDist = prevLeftDist;
int rightDist = prevRightDist;
int rightMidDist = prevRightMidDist;
int leftMidDist = prevLeftMidDist;
int midDist = prevMidDist;
State state = Idle;

float angles[3];
FreeSixIMU sixDOF = FreeSixIMU();

void setup() 
{
  pinMode(leftMid.trigPin, OUTPUT);
  pinMode(leftMid.echoPin, INPUT);
  pinMode(rightMid.trigPin, OUTPUT);
  pinMode(rightMid.echoPin, INPUT);
  pinMode(mid.trigPin, OUTPUT);
  pinMode(mid.echoPin, INPUT);
  pinMode(10,INPUT);
  pinMode(11,INPUT);
  
  frontSteer.attach(1);
  drive.attach(13);
  
 /* myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
  myLidarLite.write(0x02, 0x0d); // Maximum acquisition count of 0x0d. (default is 0x80)
  myLidarLite.write(0x04, 0b00000100); // Use non-default reference acquisition count
  myLidarLite.write(0x12, 0x03); // Reference acquisition count of 3 (default is 5)*/
  
  frontSteer.write(center);
  drive.write(90);
  delay(2000);
//  sixDOF.init();
}


void loop() {

//  sixDOF.getEuler(angles);//radians
  leftDist = getSonarDistance(left);
  rightDist = getSonarDistance(right);
  rightMidDist = getSonarDistance(rightMid);
  leftMidDist = getSonarDistance(leftMid);
  midDist = getSonarDistance(mid);
  
  switch(state)
  {
    case Idle:
      Serial.println("Beginning Obstacle Avoidance");
      state = Straight;
      break;
      
    case Straight:
      Serial.println("No obstacles detected. Moving forward");
      frontSteer.write(center);
      powerAdjustment = 0;
      if(midDist < 60)
      {
        state = FrontAlarm;
      }
      if(getAverage(leftDist, prevLeftDist) < dangerZone || getAverage(rightDist,prevRightDist) < dangerZone ||
      getAverage(leftMidDist,prevLeftMidDist) < dangerZone || getAverage(rightMidDist,prevRightMidDist) < dangerZone)
      {
        state = SideAlarm;
      }
      if(midDist < 10)
      {
        state = Stop;
      }
      break;
      
    case SideAlarm:
      Serial.println("Side collision detected");
      angleError = getAverage(rightDist,prevRightDist) - getAverage(leftDist,prevLeftDist);
      angleError = angleError<-45?-45:angleError;
      angleError = angleError>45?45:angleError;
      angle = (int)(angleKp * angleError);
      frontSteer.write(center+angle);
      
      //30 degrees off horizontal
      if(getAverage(leftDist,prevLeftDist) < dangerZone) //change power based on which side is within the dangerZone
      {
        powerKp = 0.2;
        powerError = dangerZone-getAverage(leftMidDist,prevLeftDist) ;
      }
      else if(getAverage(rightDist,prevRightDist) < dangerZone)
      {
        powerKp = 0.2;
        powerError = dangerZone-getAverage(rightDist,prevRightDist) ;
      } 
      else
      {
        state = Straight;
      }
      
      //30 degrees off straight
      if(getAverage(leftMidDist,prevLeftMidDist) < dangerZone)
      {
        powerKp = 0.5;
        powerError = dangerZone-getAverage(leftMidDist,prevLeftMidDist) ;
      }
      else if(getAverage(rightMidDist,prevRightMidDist) < dangerZone)
      {
        powerKp = 0.5;
        powerError = dangerZone-getAverage(rightMidDist,prevRightMidDist) ;
      }
      else
      {
        state = Straight;
      }
      if(getSonarDistance(mid) < 10)
      {
        state = Stop;
      }
     
      break;
      
    case FrontAlarm:
      Serial.println("Forward Collision detected");
      if(getAverage(leftDist,prevLeftDist) > getAverage(rightDist,prevRightDist)) 
     // (getAverage(getSonarDistance(right),prevRightDist) < proximityTolerance || getAverage(getSonarDistance(left),prevLeftDist) < proximityTolerance))//something is closer on the right side
      {
        frontSteer.write(center-45); //turn left
      }
      else if(getAverage(leftDist,prevLeftDist) <= getAverage(rightDist,prevRightDist))
      //(getAverage(getSonarDistance(right),prevRightDist) < proximityTolerance || getAverage(getSonarDistance(left),prevLeftDist) < proximityTolerance))
      {
        frontSteer.write(center+45);//turn right
      }
      else
      {
        state = Straight;
      }
      break;
      
    case Stop:
      Serial.println("Imminent collision. Stopping");
      drive.write(0);
      delay(1000);
      drive.write(90);
      delay(2000);
      state = Reverse;
      break;
      
    case Reverse:
      Serial.println("Making Space");
      frontSteer.write(center);
      drive.write(70);
      delay(1000);
      drive.write(90);
      delay(2000);    
      state = Straight; 
      break;
    
  }
    Serial.println(midDist);
    
    powerAdjustment = (int)( powerKp * powerError);
    drive.write(basePower-abs(powerAdjustment));
    
    prevLeftDist = leftDist; // old dist = current distance
    prevRightDist = rightDist;
    prevRightMidDist = rightMidDist;
    prevLeftMidDist = leftMidDist;
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
  return (a+b)/2;
}
float getSonarDistance(int sonarPin)
{
  return (getPing(sonarPin)*.034)/2;
}
float getSonarDistance(Sonar sonar)
{
  digitalWrite(sonar.trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(sonar.trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(sonar.trigPin, LOW);
  sonar.duration = pulseIn(sonar.echoPin, HIGH);
  //Serial.write(test.duration);
  sonar.dist =(sonar.duration/2)/29.1;
  return (float)sonar.dist;
}

// Read distance. The approach is to poll the status register until the device goes
// idle after finishing a measurement, send a new measurement command, then read the
// previous distance data while it is performing the new command.
int getLidarDistance(bool biasCorrection)
{
  byte isBusy = 1;
  int distance;
  int loopCount;

  // Poll busy bit in status register until device is idle
  while(isBusy)
  {
    // Read status register
    Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
    Wire.write(0x01);
    Wire.endTransmission();
    Wire.requestFrom(LIDARLITE_ADDR_DEFAULT, 1);
    isBusy = Wire.read();
    isBusy = bitRead(isBusy,0); // Take LSB of status register, busy bit

    loopCount++; // Increment loop counter
    // Stop status register polling if stuck in loop
    if(loopCount > 9999)
    {
      break;
    }
  }

  // Send measurement command
  Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
  Wire.write(0X00); // Prepare write to register 0x00
  if(biasCorrection == true)
  {
    Wire.write(0X04); // Perform measurement with receiver bias correction
  }
  else
  {
    Wire.write(0X03); // Perform measurement without receiver bias correction
  }
  Wire.endTransmission();

  // Immediately read previous distance measurement data. This is valid until the next measurement finishes.
  // The I2C transaction finishes before new distance measurement data is acquired.
  // Prepare 2 byte read from registers 0x0f and 0x10
  Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
  Wire.write(0x8f);
  Wire.endTransmission();

  // Perform the read and repack the 2 bytes into 16-bit word
  Wire.requestFrom(LIDARLITE_ADDR_DEFAULT, 2);
  distance = Wire.read();
  distance <<= 8;
  distance |= Wire.read();

  // Return the measured distance
  return distance;
}
