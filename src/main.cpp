#include "SensorManager.h"
#include "RGBLed.h"

#define LED_R_PIN 5
#define LED_G_PIN 6
#define LED_B_PIN 7

RGBLed led(LED_R_PIN, LED_G_PIN, LED_B_PIN);
SensorManager sensorManager;
SensorData data;

void setup()
{
  sensorManager.begin(true, true, false);
  led.begin();
}

void loop()
{
  sensorManager.calculateAngles(data);
  Serial.print(data.roll);
  Serial.print(",");
  Serial.print(data.pitch);
  Serial.print(",");
  Serial.println(data.yaw);
  led.setColor(1, 0, 0);

  delay(10);
  led.setColor(1, 1, 0);
}
