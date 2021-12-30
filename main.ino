#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include "compass_LSM303.h"
#include "gpsdata.h"

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
int wayPointCount;
float distanceToGoal=0.0;
float bearing=0.0;
float compassheading=0.0;
float azimuth=0.0;
gpsdata::gpsval currGPSData;
gpsdata::gpsval wayPointsDes;
bool flag= false;
bool initialize_flag= true;
// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);


//servo
Adafruit_MotorShield AFMS=Adafruit_MotorShield();
Servo servo1;
Servo servo2;


// function to direct robot according to azimuth

void navigateRobot1(float azimuth, char dir){
  if(dir == 'L'){
      servo1.write(90);  // turn left
      servo2.write(0);
      delay(500);
  }
  else if(dir == 'R'){
          // turn right
      servo1.write(180);  //turn right
      servo2.write(90);
      delay(500);
  }
  else {
       servo1.write(180);
       servo2.write(0);
  }
}


void navigateRobot(float azimuth){

  if (azimuth > 45  && azimuth <= 135){
      // turn right
      servo1.write(180);  //turn right
      servo2.write(90);
      delay(1200);
  }
  else if(azimuth > 135 && azimuth <= 225){
      servo1.write(0);   // backward
      servo2.write(180);
      delay(2000);
  }
   else if(azimuth > 225 && azimuth < 315){
      servo1.write(90);  // turn left
      servo2.write(0);
      delay(1200);
  }

  else{
    servo1.write(180);
    servo2.write(0);
  }
}


void setup(){
  Serial.begin(9600);
  ss.begin(GPSBaud);
  compass::setupCompass();

  //servo pins
   AFMS.begin();
  servo1.detach();
  servo2.detach();
  wayPointCount=1;
}

void loop(){
// if(initialize_flag)
// {
//  delay(2000);
//  initialize_flag= false;
// }
  
  // This sketch displays information every time a new sentence is correctly encoded.
//    byte gpsData = ss.read();
//    Serial.write(gpsData);
//float waypoints[3][2] = {{29.74396,62.59734}, {29.74507,62.59856}, {29.75086,62.60212}};
float waypoints[3][2] = {{29.828016,62.615333}, {29.828104,62.615139}, {29.745006,62.603649}};
  while (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){
      flag=true;

           if(flag)
     {
        servo1.attach(9);
        servo2.attach(10);
        flag= false;
     }

      //way to destination
      currGPSData.latitude= gps.location.lat();
      currGPSData.longitude= gps.location.lng();
      wayPointsDes.latitude= waypoints[wayPointCount][1];
      wayPointsDes.longitude= waypoints[wayPointCount][0];

      distanceToGoal = gpsdata::getDistance(currGPSData, wayPointsDes);

      Serial.print("waypoint = ");Serial.print(wayPointCount);
      Serial.print("Latitude = "); Serial.print(gps.location.lat(), 6);
      
      Serial.print(" Longitude = "); Serial.println(gps.location.lng(), 6);
      
      Serial.print("\n");
      Serial.print("distanceToGoal = ");Serial.print(distanceToGoal);


     // if(wayPointCount == 0 && !flag){
     //   flag = true;
        compassheading=compass::getHeading();
       // bearing = gpsdata::getGPSBearing(currGPSData, wayPointsDes); 
        bearing = gpsdata::getGPSBearing(wayPointsDes, currGPSData); 
        azimuth = compassheading - bearing;
        if(azimuth < 0){
          azimuth *= -1;
        }
        Serial.print("\n");
        Serial.print("bearing = ");Serial.print(bearing);
        Serial.print("\n");
        Serial.print("azimuth = ");Serial.print(azimuth);
      //}

        while(azimuth > 10){
          currGPSData.latitude= gps.location.lat();
          currGPSData.longitude= gps.location.lng();
          distanceToGoal = gpsdata::getDistance(currGPSData, wayPointsDes);
          Serial.print("Latitude = "); Serial.print(gps.location.lat(), 6);
      
          Serial.print(" Longitude = "); Serial.println(gps.location.lng(), 6);
          Serial.print("distanceToGoal = ");Serial.println(distanceToGoal);
          Serial.print("Inside loop azimuth = ");Serial.print(azimuth);
          Serial.print("\n");
          if (compassheading < bearing)
           navigateRobot1(azimuth, 'R');
          else if (compassheading > bearing)
           navigateRobot1(azimuth, 'L');
           
          bearing = gpsdata::getGPSBearing(wayPointsDes, currGPSData); 
          compassheading=compass::getHeading();
          azimuth = compassheading - bearing;

          Serial.print("inside loop bearing = ");Serial.print(bearing);
          Serial.print("\n");
          Serial.print("inside loop compass = ");Serial.print(compassheading);
          Serial.print("\n");

          if(azimuth < 0){
            azimuth *= -1;
          }
          Serial.print("Inside loop azimuth = ");Serial.print(azimuth);
          Serial.print("\n");
        }
      
      
      Serial.print("\n");
      Serial.print("compassheading");Serial.print(compassheading);
      Serial.print("\n");
      
      if(distanceToGoal < 1 ){
        wayPointCount++;
        if (wayPointCount >= 2){
          Serial.print("Destination");
          servo1.detach();
          servo2.detach();
        }

      }
      servo1.write(180);
      servo2.write(0);
      }
      
    }
//     servo1.write(90);
//     servo2.write(90);
  
}
