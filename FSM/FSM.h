
typedef enum
{
  Idle,
  Straight,
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

float getAverage(int a,int b);
float getMode(int a, int b);
float getSonarDistance(int sonarPin);
float getSonarDistance(Sonar sonar);
Sonar setupSonar(int trigPin, int echoPin);
float getLidarDistance();
void turnServo();

Sonar leftMid = setupSonar(6,7);
Sonar rightMid = setupSonar(4,5);
Sonar mid = setupSonar(9,8);
const int right = 10;
const int left = 11;

const int center = 80;//degrees
float angleKp = 1.5;
float powerKp = 0.1;
int dangerZone = 100; //all distance readings are in cm
int proximityTolerance = 100;
int angle = center;
int angleError = 0;
int netAngleError = 0;

int basePower = 110;
int powerError =0;
int netPowerError = 0;
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

LIDARLite lidar;
int lidarAngle=90;
int lidarSpeed = 5;

FreeSixIMU sixDOF = FreeSixIMU();
Servo drive, frontSteer, lidarSteer;
