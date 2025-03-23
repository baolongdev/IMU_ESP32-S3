#include "RGBLed.h"

RGBLed::RGBLed(uint8_t r_pin, uint8_t g_pin, uint8_t b_pin)
    : redPin(r_pin), greenPin(g_pin), bluePin(b_pin) {}

void RGBLed::begin()
{
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);
    turnOff();
}

void RGBLed::setColor(bool red, bool green, bool blue)
{
    digitalWrite(redPin, red ? HIGH : LOW);
    digitalWrite(greenPin, green ? HIGH : LOW);
    digitalWrite(bluePin, blue ? HIGH : LOW);
}

void RGBLed::turnOff()
{
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);
}
