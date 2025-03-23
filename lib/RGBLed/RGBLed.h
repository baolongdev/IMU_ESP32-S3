#ifndef RGB_LED_H
#define RGB_LED_H

#include <Arduino.h>

class RGBLed
{
public:
    RGBLed(uint8_t r_pin, uint8_t g_pin, uint8_t b_pin);
    void begin();
    void setColor(bool red, bool green, bool blue);
    void turnOff();

private:
    uint8_t redPin, greenPin, bluePin;
};

#endif // RGB_LED_H
