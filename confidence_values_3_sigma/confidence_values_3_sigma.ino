#include <NewPing.h>
 NewPing leftMid(6,7,1000);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  //calculate the confidence interval using 3 sigma test
  //goal: get a 0,1,2,3,etc to find the confidence of that an object is there.
  // mean = 1/n*(sum(x,n))
  //std dev = sqrt((1/(n-1))*sum((x-mean)^2)
  int value = leftMid.ping_cm();
  if(value != 0)
  {
    Serial.print(value);
    Serial.print(" | ");
    Serial.println(getConfidence(leftMid.ping_cm(), 130)); //get the confidence value of the distance if it is within the valid zone.
  }
  delay(100);
}

double getConfidence(double value,double validRange)
{
  int mean;
  int N = 20;
  int stdDev;
  int data[N];
  for(int i =0;i<N;i++)
  {
    data[i] = leftMid.ping_cm();
    mean+=data[i];
  }
  mean /= N;
  for(int i = 0;i<N;i++)
  {
    stdDev += (data[i]-mean)^2;
  }
  stdDev = sqrt(stdDev/(N-1));
  
  if(value < mean+stdDev && value > mean-stdDev && value < validRange) //if value is within 68.3% of all values within the valid obstacle range
  {
    return 3;
  }
  else if(value < mean+2*stdDev && value > mean - 2*stdDev && value < validRange) //if value is within 95.4% of all values within the valid obstacle range
  {
    return 2;
  }
  else if(value < mean+2*stdDev && value > mean - 2*stdDev && value < validRange) //if value is within 99.7% of all values within the valid obstacle range
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

