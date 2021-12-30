#include <Wire.h>
#include <HMC5883L_Simple.h>

namespace compass
{
    HMC5883L_Simple Compass;

  void setupCompass()
  {
    //configure compass
    Wire.begin();
    Compass.SetDeclination(2, 31, 'E');
    Compass.SetSamplingMode(COMPASS_CONTINUOUS);
    Compass.SetScale(COMPASS_SCALE_130); //Sensitivity
    Compass.SetOrientation(COMPASS_HORIZONTAL_Y_NORTH);
    Serial.println("Compass initialized.");
  }

  float getHeading()
  {
    return Compass.GetHeadingDegrees()-180.0;
  }

}
