
typedef struct
{
  int echoPin;
  int trigPin;
  double dist;
  double duration;
}Sonar;

Sonar setupSonar(int trigPin, int echoPin);

Sonar test = setupSonar(8,9);
Sonar test2 = setupSonar(6,7);
Sonar test3 = setupSonar(4,5);
void setup() {
  // put your setup code here, to run once:
  pinMode(test.trigPin, OUTPUT);
  pinMode(test.echoPin, INPUT);
  pinMode(test2.trigPin, OUTPUT);
  pinMode(test2.echoPin, INPUT);
  pinMode(test3.trigPin, OUTPUT);
  pinMode(test3.echoPin, INPUT);
}

void loop() {
  digitalWrite(test.trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(test.trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(test.trigPin, LOW);
  test.duration = pulseIn(test.echoPin, HIGH);
  //Serial.write(test.duration);
  test.dist =(test.duration/2)/29.1;
  if(test.dist<300)
  {
  Serial.println("first ");
  Serial.print(test.dist);
  }
  digitalWrite(test2.trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(test2.trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(test2.trigPin, LOW);
  test2.duration = pulseIn(test2.echoPin, HIGH);
  //Serial.write(test.duration);
  test2.dist =(test2.duration/2)/29.1;
  if(test2.dist<300)
  {
    
//  Serial.println("second  ");
//  Serial.print(test2.dist);
  }

   digitalWrite(test3.trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(test3.trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(test3.trigPin, LOW);
  test3.duration = pulseIn(test3.echoPin, HIGH);
  //Serial.write(test.duration);
  test3.dist =(test3.duration/2)/29.1;
  if(test3.dist<300)
  {
//  Serial.println("third ");
//  Serial.print(test3.dist);
  }
}

Sonar setupSonar(int trigPin, int echoPin)
{
  Sonar temp;
  temp.trigPin = trigPin;
  temp.echoPin = echoPin;
  return temp;
}

//float getSonarDistance(int sonarPin)
//{
//  return (getPing(sonarPin)*.034)/2;
//}
