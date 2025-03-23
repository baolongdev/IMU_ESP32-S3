#include "SensorManager.h"

SensorManager sensorManager;
SensorData data;

void setup()
{
  sensorManager.begin(true, true, false);
}

void loop()
{
  sensorManager.calculateAngles(data);
  Serial.print((int)data.roll);
  Serial.print(",");
  Serial.print((int)data.pitch);
  Serial.print(",");
  Serial.println((int)data.yaw);

  delay(10);
}
