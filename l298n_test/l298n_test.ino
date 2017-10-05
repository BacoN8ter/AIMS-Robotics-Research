void setup() {
  // put your setup code here, to run once:
  pinMode(A0, OUTPUT);
  pinMode(A1,OUTPUT);
  pinMode(A2,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(A1,HIGH);
  digitalWrite(A0,LOW);
  analogWrite(A2,255);
  delay(10000);
  digitalWrite(A1,LOW);
  digitalWrite(A0,HIGH);
  analogWrite(A2,255);
  delay(10000);
  digitalWrite(A0,LOW);
  digitalWrite(A1,HIGH);
  analogWrite(A2,0);
  delay(1000);
}
