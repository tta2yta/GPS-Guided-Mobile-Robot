#include <Servo.h>

Servo servo;
const int  trig = 12, echo = 13 ;
int pos = 0, distcm = 0;
void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  servo.attach(11);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

}

void loop() {
  //servo.write(0);
  int i;
  for (i=0; i <= 180; i++){
    servo.write(i);
    Serial.println(radar());  
    delay(10);
  }
//  for (i=180; i >= 0; i--){
//    servo.write(i);
//    //delay(300);
//  }
}

long radar(void)
{
  digitalWrite(trig, HIGH);
  delayMicroseconds(15);
  digitalWrite(trig, LOW);
  long  dur = pulseIn(echo, HIGH);
  distcm = dur / 58;
  return distcm;
}
