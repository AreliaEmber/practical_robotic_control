#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#ifndef NUMPIXELS
#define NUMPIXELS 4
#endif

#ifndef RGB_PIN
#define RGB_PIN 3
#endif

class RGB : public Adafruit_NeoPixel
{
public:
  RGB() : Adafruit_NeoPixel(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800) {}

  void initialize(uint8_t bright = 50)
  {
    begin();
    setBrightness(bright);
    clear();
    show();
  }

  void blueOn()
  {
    fill(Color(0, 0, 255));
    show();
  }

  void off()
  {
    clear();
    show();
  }

  void blueBlinkOnce(uint16_t ms_on = 120)
  {
    blueOn();
    delay(ms_on);
    off();
  }
};

extern RGB rgb;




