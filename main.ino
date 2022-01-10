#include <NMEAGPS.h>
#include <Servo.h>
#include <NeoSWSerial.h>
#include <Adafruit_MotorShield.h>
#include "compass_LSM303.h"
#include "gpsdata.h"
#include <NewPing.h>
#define MAX_DISTANCE 300

NMEAGPS gps;
NeoSWSerial gpsPort(8, 7);

static const int RXPin = 3, TXPin = 4;
static const uint32_t GPSBaud = 9600;
int wayPointCount;
float distanceToGoal = 0.0;
float bearing = 0.0;
float compassheading = 0.0;
float azimuth = 0.0;
gpsdata::gpsval currGPSData;
gpsdata::gpsval wayPointsDes;
gpsdata::gpsval disTrav;
gpsdata::gpsval wayPointsStart;
bool flag = false;
bool initialize_flag = true;
unsigned long lastDisplayTime = 0;
unsigned long startTime = 0;

//servo
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Servo servo1;
Servo servo2;

//servo with ultrasonic sensor
Servo servo;
const int  trig = 12, echo = 13 ;
int pos = 0, distcm = 0;
NewPing sonar(trig, echo, MAX_DISTANCE);

// test with constant bearing
float bearing1=175.0;
float bearing2=75.0;
float distanceTraveled=0.0;


// function to direct robot according to azimuth

int navigateRobot1(float azimuth, char dir) {
    if (azimuth <= 10){
            servo1.write(90);
            servo2.write(90);
            return 0;
    }
  if (dir == 'L') {
    //avoidobstacle();
    servo1.write(90);  // turn left
    servo2.write(50);
  }
  else if (dir == 'R') {
   // avoidobstacle();
    servo1.write(140);  //turn right
    servo2.write(90);
  }
  else if (dir == 'F') {
    avoidobstacle();
  }
}


void avoidobstacle(void){
  for (int i=0; i <= 80; i++){
    servo.write(i);
    delay(10);
    int uS = sonar.ping();
    if(uS / US_ROUNDTRIP_CM > 0 && uS / US_ROUNDTRIP_CM <= 25){
      Serial.print("dist i");Serial.println(uS / US_ROUNDTRIP_CM);
      servo1.write(90);
      servo2.write(90); 
      servo1.write(130);
      servo2.write(90);
      delay(1000);
    }
    
  }
  for (int j=80; j >= 0; j--){
    servo.write(j);
    delay(10);
    int uS = sonar.ping();
      if(uS / US_ROUNDTRIP_CM > 0 && uS / US_ROUNDTRIP_CM <= 25 ){
        Serial.print("dist j"); Serial.println(uS / US_ROUNDTRIP_CM); 
        servo1.write(90);
        servo2.write(90); 
        servo1.write(130);
        servo2.write(90);
        delay(1000);
    }
  }
}

void ultrasonic(void){
  int i;
  for (i=0; i <= 100; i++)
    servo.write(i);

  for (int j=100; j >= 0; j--)
    servo.write(j);
}

//detrmine duartion
unsigned long previousTime = 0;
byte seconds ;
int durations ()
{
// I  think using microseconds is even more accurate
  if (millis() >= (previousTime)) 
  {
     previousTime = previousTime + 1000;  // use 100000 for uS
     seconds = seconds +1;
  }
  return seconds;
}

void setup()
{
  Serial.begin(9600);
  gpsPort.begin(9600);
  gps.send_P( &gpsPort, F("PMTK220,1000") );   // 1 Hz update rate

  compass::setupCompass();

  //servo pins
  AFMS.begin();
  servo1.detach();
  servo2.detach();
  wayPointCount = 1;

  servo.attach(11);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  servo.write(20);

  startTime= millis();

}

void loop()
{
 // ultrasonic();
 //float waypoints[3][2] = {{29.827997,62.614990}, {29.827184,62.614791}, {29.75086,62.60212}}; //outside home checkpoint
 // float waypoints[3][2] = {{29.743558,62.598533}, {29.742828,62.598495}, {29.75086,62.60212}}; //university checkpoint
 float waypoints[3][2] = {{29.828125,62.615303}, {29.743356,62.600078}, {29.828092,62.615077}}; // inside home
  while (gps.available(gpsPort)) 
  {
  
    //Get the latest info from the gps object which it derived from the data sent by the GPS unit
    gps_fix fix = gps.read();
    if (fix.valid.location && fix.valid.satellites && fix.satellites >= 5) {
        //ultrasonic();
      flag = true;
      if (flag)
      {
        servo1.attach(9);
        servo2.attach(10);
        flag = false;
      }
      //way to destination
      currGPSData.latitude = fix.latitude();
      currGPSData.longitude = fix.longitude();

      wayPointsStart.latitude= waypoints[wayPointCount -1 ][1];
      wayPointsStart.longitude= waypoints[wayPointCount -1 ][1];
      
      wayPointsDes.latitude = waypoints[wayPointCount][1];
      wayPointsDes.longitude = waypoints[wayPointCount][0];

      disTrav.latitude = waypoints[wayPointCount -1][1];
      disTrav.longitude = waypoints[wayPointCount -1][0];

      

      distanceToGoal = gpsdata::getDistance(currGPSData, wayPointsDes);
      distanceTraveled= gpsdata::getDistance(disTrav, currGPSData);

      Serial.print("waypoint = "); Serial.print(wayPointCount);
      Serial.print("Latitude = "); Serial.print(fix.latitude(), 6);

      Serial.print(" Longitude = "); Serial.println(fix.longitude(), 6);
      Serial.print("# of satallites="); Serial.println(fix.satellites);

      Serial.print("\n");
      Serial.print("distanceToGoal = "); Serial.print(distanceToGoal);
      Serial.print("dist traveled = "); Serial.println(distanceTraveled);

      //determine azimuth
      compassheading = compass::getHeading();
      bearing = gpsdata::getGPSBearing(currGPSData, wayPointsDes);
      //bearing = gpsdata::getGPSBearing(wayPointsDes, currGPSData);
      Serial.print("compass = "); Serial.println(compassheading);
      azimuth = compassheading - bearing1;
      azimuth = azimuth < 0 ? azimuth * -1 : azimuth;
      
      Serial.print("\n");
      Serial.print("bearing = "); Serial.print(bearing);
      Serial.print("\n");
      Serial.print("azimuth = "); Serial.print(azimuth);

    if (compassheading < bearing1 && azimuth > 10){
      bool ctrLopp=true;
      while(ctrLopp){
      currGPSData.latitude = fix.latitude();
      currGPSData.longitude = fix.longitude();
      compassheading = compass::getHeading();
      bearing = gpsdata::getGPSBearing(currGPSData, wayPointsDes);
      azimuth = abs(compassheading - bearing1);
      //azimuth = azimuth < 0 ? azimuth * -1 : azimuth;

      Serial.print("inside Latitude = "); Serial.print(fix.latitude(), 6);
      Serial.print("inside Longitude = "); Serial.println(fix.longitude(), 6);
      Serial.print("inside bearing = "); Serial.println(bearing);
      Serial.print("inside azimuth = "); Serial.println(azimuth);
      Serial.print("inside compass = "); Serial.println(compassheading);
      if (azimuth < 10){
        servo1.write(90);
        servo2.write(90);
        ctrLopp=false;
        continue; 
      }
 
      else
        navigateRobot1(azimuth, 'R');
      }
      }
     
    else if (compassheading > bearing1 && azimuth > 10){
      bool ctrLopp=true;
      while(ctrLopp){
      currGPSData.latitude = fix.latitude();
      currGPSData.longitude = fix.longitude();
      compassheading = compass::getHeading();
      bearing = gpsdata::getGPSBearing(currGPSData, wayPointsDes);
      azimuth = abs(compassheading - bearing1);
      //azimuth = azimuth < 0 ? azimuth * -1 : azimuth;
      
      Serial.print("inside Latitude = "); Serial.print(fix.latitude(), 6);
      Serial.print("inside Longitude = "); Serial.println(fix.longitude(), 6);
      Serial.print("inside bearing = "); Serial.println(bearing);
      Serial.print("inside azimuth = "); Serial.println(azimuth);
      Serial.print("inside copmass = "); Serial.println(compassheading);
      if (azimuth < 10){
        servo1.write(90);
        servo2.write(90);
        ctrLopp=false;
        continue; 
      }
      else
        navigateRobot1(azimuth, 'L');
      }
    } 
  if(azimuth < 10){
      avoidobstacle();
      Serial.println("forward");
      servo1.write(130);
      servo2.write(45);
    }
     
      if(durations()  >= 25){
      servo1.detach();
      servo2.detach();
      exit(0);
      }
     if(durations()  >= 12){
        bearing1= bearing2;
        Serial.println("bearing 2");
        //seconds=0;
      }


      //determine distance to target
      if (distanceToGoal < 5 ) {
        wayPointCount++;
        if (wayPointCount > 2) {
          Serial.print("Destination");
          servo1.detach();
          servo2.detach();
        }

      }

  }
  
Serial.print("duration = ");Serial.println(durations());
}

}
