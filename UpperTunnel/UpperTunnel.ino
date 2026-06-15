#include <stdlib.h>
#include <FastLED.h>
//#include <utility>
template<
    class T1,
    class T2
> struct pair
{ pair(T1 f = 0, T2 s = 0) : f_first(f), f_second(s) {}
  pair(const pair &p) : f_first(p.f_first), f_second(p.f_second) {}
  T1 first() const {return f_first;}
  T2 second() const {return f_second;}
  T1 f_first;
  T2 f_second;};

#define NUM_LEDS 8
#define DATA_PIN 6
#define BRIGHTNESS  32

CRGB warmFlash() {return CRGB(255/8, (180 + random8(60))/8, (80 + random8(80))/8);}

CRGB leds[NUM_LEDS];

pair<int,int> address(int num) {return pair<int,int>(NUM_LEDS/2-1-num,NUM_LEDS/2+num);}

void setup() {
  delay(3000);
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );
  Serial.begin(9600);
  for(int j=0;j<NUM_LEDS/2;++j) leds[j] = CRGB(100,255,255);
  for(int j=NUM_LEDS/2;j<NUM_LEDS;++j) leds[j] = CRGB(255,0,0);
  FastLED.show();
  delay(5000);
  for(int i=0; i<NUM_LEDS; ++i) leds[i] = CRGB(0,0,0);
  FastLED.show();
}

void loop() {

  const CRGB rgb[3] = {CRGB(255,0,0), CRGB(0,255,0), CRGB(0,0,255)};
  const CRGB wwhites[3] = {CRGB(253, 244, 220), CRGB(255, 197, 143), CRGB(255, 212, 38)};

  static int8_t jmax = 0;
  pair<int,int> ad;

  ++jmax;
  jmax = min(jmax,NUM_LEDS/2);

 for (int8_t i = 0; i < jmax; i++) {
    ad = address(i);
    leds[ad.first()] = wwhites[random8(3)];
    leds[ad.second()] = wwhites[random8(3)];
 
//    leds[i] = wwhites[random8(3)];
  }
/*
  static int j = 0;
  for (int i = 0; i < NUM_LEDS/2; i++) {
    pair<int,int> ad = address(i);
    ++j;
    leds[ad.first()] = rgb[j%3];
    leds[ad.second()] = rgb[j%3];
  }
  */
//
  FastLED.show();
  delay(random16(5000));

  int8_t fside = random8(2);
  int8_t r1 = random8(2);
  int8_t r2 = random8(2);
  if(r1 == 1) {
    ad = address(random8(jmax));
    if(fside == 0) leds[ad.first()] = warmFlash();
    else leds[ad.second()] = warmFlash();
    FastLED.show();
    delay(random16(1000));
  }
  if(r2 == 1 || r1 ==0) {
    ad = address(random8(jmax));
    if(fside == 0) leds[ad.second()] = warmFlash();
    else leds[ad.first()] = warmFlash();
    FastLED.show();
    delay(random16(1000));
  }

}
