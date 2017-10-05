#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <LIDARLite.h>
#include <Wire.h>

float angles[3]; // yaw pitch roll
double fudgeConst;
double error;
// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();
LIDARLite myLidarLite;

void setup() 
{ 
  Serial.begin(115200);
  Wire.begin();
  
  myLidarLite.configure(0); // Change this number to try out alternate configurations
  myLidarLite.begin(0, true);
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
  
  float prevAngles[3];
  prevAngles[3] = angles[3];
  for(int i = 0;i<100;i++)
  {
    error += angles[3] - prevAngles[3];
    delay(100);
  }
  error = error/10;
}

int A = 1;
void loop() { 
  
  sixDOF.getEuler(angles);
  angles[0] = angles[0]-(float)error;
  angles[1] = angles[1]-(float)error;
  angles[2] = angles[2]-(float)error;

  Serial.print(A);
  Serial.print(" ");
  Serial.println(myLidarLite.distance(false));
  A = A + 1;
  
  Serial.print(angles[0]);
  Serial.print(" | "); 
  Serial.print(angles[1]);
  Serial.print(" | ");
  Serial.print(angles[2]);
  Serial.print(" | ");
  Serial.println(millis());
  
  delay(100); 
}
