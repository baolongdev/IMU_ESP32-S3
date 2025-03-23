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
  Serial.print(data.roll);
  Serial.print(",");
  Serial.print(data.pitch);
  Serial.print(",");
  Serial.println(data.yaw);

  delay(10);
}
