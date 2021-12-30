#include "constants.h"


namespace gpsdata
{
  struct gpsval{
  float latitude;
  float longitude;
};


float getDistance(gpsval a, gpsval b){
  const float earthRadius=6371000;
  float pnt1= a.latitude * toRadian;
  float pnt2= b.latitude * toRadian;
  float latDifference= (b.latitude - a.latitude) * toRadian;
  float lonDifference= (b.longitude - a.longitude) * toRadian;
  float x=sin(latDifference/2)*sin(latDifference/2) + cos(pnt1)*cos(pnt2)*sin(lonDifference/2)*sin(lonDifference/2);
  float y=2*atan2(sqrt(x), sqrt(1-x));
  return earthRadius * y;
}

float getGPSBearing(gpsval a, gpsval b){
  float y= sin((b.longitude * toRadian) - (a.longitude * toRadian)) * cos(b.latitude * toRadian);
  float x= cos(a.latitude * toRadian) * sin(b.latitude * toRadian) - sin(a.latitude * toRadian)*
  cos(b.latitude * toRadian) * cos((b.longitude * toRadian) - (a.longitude * toRadian));
  float bearing= atan2(y, x) * toDegree;
  if ( bearing < 0){
    bearing += 360;
  }
  return bearing;
}

}
