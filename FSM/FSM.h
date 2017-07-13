
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
bool compareSonar(float dist1, float dist2);
Sonar setupSonar(int trigPin, int echoPin);
float getLidarDistance();
void turnServo();

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
int lidarAngle=75;
int lidarSpeed = 1;

const int sonarThreshold = 10;
FreeSixIMU sixDOF = FreeSixIMU();
Servo drive, frontSteer, lidarSteer;



