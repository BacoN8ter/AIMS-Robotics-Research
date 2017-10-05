#define LIDAR_THRESHOLD 10
typedef enum
{
  Idle,
  ActiveNavigation,
  ReactiveNavigation,
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
typedef struct
{
  int lidarDist;
  int lidarAngle;
  int lidarDistY;
  int lidarDistX;
}LidarValue;

float getAverage(int a,int b);//get the average value of two inputs
float getMode(int a, int b);//get the mode of two inputs
float getSonarDistance(int sonarPin);//get the distance from a sonar sensor
float getSonarDistance(Sonar sonar);
bool compareSonar(float dist1, float dist2);//comapre two sonar distances 
float getLidarDistance();//get the distance from a Lidar Sensor
void turnServo();//turn servo motors
void updateCurrentSensorValues();//update all current sensor variables to their current values
void updatePrevSensorValues();//update the previous sensor values 
void printSensorValues();//display sensor or servo information in serial terminal

NewPing leftMid(6,7,1000);
NewPing rightMid(4,5,1000);
NewPing mid(9,8,1000);
NewPing rear(2,A3);
const int right = 10;
const int left = 11;

const int center = 80;//degrees
float angleKp = .85;
float powerKp =0.04;
int dangerZone = 125; //all distance readings are in cm
int proximityTolerance = 150;
int angle = 0;
int angleError = 0;
int netAngleError = 0;

int basePower = 120;
int powerError =0;
int netPowerError = 0;
int powerAdjustment =0;
//lidar plugged into A4 (SDA) and A5 (SCL)

int prevLeftDist = getSonarDistance(left);
int prevRightDist = getSonarDistance(right);
int prevRightMidDist = rightMid.ping_cm();
int prevLeftMidDist = leftMid.ping_cm();
int prevMidDist = mid.ping_cm();
int prevRearDist = rear.ping_cm();

int leftDist = prevLeftDist;
int rightDist = prevRightDist;
int rightMidDist = prevRightMidDist;
int leftMidDist = prevLeftMidDist;
int midDist = prevMidDist;
int rearDist = prevRearDist;
State state = Idle;

LIDARLite lidar;
LidarValue currentLidarData;
LidarValue arrayValues;
int minAngle = 45;
int lidarSpeed = 1;
const int sweepDegrees = 90;
LidarValue lidarArray[sweepDegrees];
LidarValue farthestLidarValue = lidarArray[0];
int farthestLidarValueIndex = (int)(sweepDegrees - minAngle)/2;//midpoint

long previousMillis = 0;
bool leftTrigger = false;
bool rightTrigger = false;
const int sonarThreshold = 10;
FreeSixIMU sixDOF = FreeSixIMU();
Servo drive, frontSteer, lidarSteer;



