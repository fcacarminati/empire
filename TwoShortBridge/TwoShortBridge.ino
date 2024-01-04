//
// Sketch for two bridges
//
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// Pin configuration
//
// Bridge 1
// D2  driver right red lights
// D3  driver bridge lights
// D4  driver read red lights
// D5  digital input for sensor 1R
// D6  digital input for sensor 1L
// D12 led for train in from right
// A0  led for train in from left
//
// Bridge 2
// D7  driver right red lights
// D9  driver bridge lights
// D8  driver left red lights
// D10 digital input for sensor 2R
// D11 digital input dor sensor 2L
// A1  led for train in from right
// A2  let for train in from left
//
// A4 SDA for OLED
// A5 SCL for OLED

// Bridge 1
int redR_b1 = 2;   // pin for right red lights
int lights_b1 = 3; // pin white lights
int redL_b1 = 4;   // pin for left red lights
int sensR_b1 = 5;  // right sensor
int sensL_b1 = 6;  // left sensor
int tInR_b1 = 12;  // led for train in from right
int tInL_b1 = A0;  // led for train in from left
int brightness_b1 = 0;
unsigned long fademils_b1 = millis();
unsigned long blinkred_b1 = millis();
int highlow_b1 = 0;
short curR_b1 = 0;    // current state of right sensor
short curL_b1 = 0;    // current state of left sensor
short preR_b1 = 0;    // previous state of right sensor
short preL_b1 = 0;    // previous state of left sensor
short stateR_b1 = 0;  // logical state of right side
// 0 off; 1 cleared; 2 set; 3 on
short stateL_b1 = 0;  // logical state of left side
short fade_b1 = 0;    // fading direction
bool trainInL_b1 = false;
bool trainInR_b1 = false;
float timeStart_b1 = 0;
float timeDelta_b1 = 0;

// Bridge 2
int redR_b2 = 7;
int lights_b2 = 9;
int redL_b2 = 8;
int sensR_b2 = 10;
int sensL_b2 = 11;
int tInR_b2 = A1;  // led for train in from right
int tInL_b2 = A2;  // led for train in from left
int brightness_b2 = 0;
unsigned long fademils_b2 = millis();
unsigned long blinkred_b2 = millis();
int highlow_b2 = 0;
short curR_b2 = 0;
short curL_b2 = 0;
short preR_b2 = 0;
short preL_b2 = 0;
short stateR_b2 = 0;  // logical state of right side
// 0 off; 1 cleared; 2 set; 3 on
short stateL_b2 = 0;  // logical state of left side
short fade_b2 = 0;
bool trainInL_b2 = false;
bool trainInR_b2 = false;
float timeStart_b2 = 0;
float timeDelta_b2 = 0;

const int fadeAmount = 5;    // how many points to fade the LED by
const float bridgeLength = 20; // bridge length in cm
const float speedConv = bridgeLength * 87 * 1e-5 * (3600 * 1e3); 

enum State {kOn, kClear, kSet, kOff};

//------------------------------------------- OLED Stuff -----------------------------------------
Adafruit_SSD1306 display(128, 64, &Wire, -1);

const unsigned char TrainLogoR [] PROGMEM = {
  0x00, 0x00, 0x4f, 0xc0, 0x00, 0x00, 0x00, 0xff, 0xe0, 0x00, 0x00, 0x03, 0xff, 0xf0, 0x00, 0x00, 
  0x03, 0xff, 0xf0, 0x00, 0x00, 0x0f, 0xff, 0xf0, 0x00, 0x00, 0x1f, 0xff, 0xf0, 0x00, 0x00, 0x3f, 
  0xff, 0xe0, 0x00, 0x00, 0x3f, 0xff, 0xf0, 0x00, 0x00, 0x3f, 0xff, 0xf0, 0x00, 0x00, 0x7f, 0xff, 
  0xe0, 0x00, 0x01, 0xff, 0xff, 0xc0, 0x00, 0x03, 0xff, 0xfc, 0x00, 0x00, 0x03, 0xfe, 0x00, 0x00, 
  0x00, 0x07, 0xf8, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 
  0x03, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x1f, 0xfe, 0x03, 
  0x80, 0x00, 0x1f, 0xfe, 0x03, 0x80, 0x00, 0x1f, 0xc0, 0x03, 0x83, 0xc0, 0x1f, 0xc0, 0x03, 0x87, 
  0xc0, 0x1f, 0xc0, 0x03, 0x83, 0xc0, 0x1f, 0xc0, 0x3f, 0xff, 0xff, 0xff, 0xc0, 0x7f, 0xff, 0xff, 
  0xff, 0xc0, 0x7f, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 
  0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 
  0xcf, 0xe7, 0xf9, 0xfc, 0x3f, 0xcf, 0xe7, 0xf9, 0xfc, 0x3f, 0xcf, 0xe7, 0xf9, 0xfc, 0x1f, 0x8f, 
  0xe3, 0xf1, 0xfc, 0x0f, 0x07, 0xc1, 0xf0, 0xf8
};

// 'trainLogoR', 40x40px
const unsigned char TrainLogoL [] PROGMEM = {
  0x00, 0x03, 0xf2, 0x00, 0x00, 0x00, 0x07, 0xff, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xc0, 0x00, 0x00, 
  0x0f, 0xff, 0xc0, 0x00, 0x00, 0x0f, 0xff, 0xf0, 0x00, 0x00, 0x0f, 0xff, 0xf8, 0x00, 0x00, 0x07, 
  0xff, 0xfc, 0x00, 0x00, 0x0f, 0xff, 0xfc, 0x00, 0x00, 0x0f, 0xff, 0xfc, 0x00, 0x00, 0x07, 0xff, 
  0xfe, 0x00, 0x00, 0x03, 0xff, 0xff, 0x80, 0x00, 0x00, 0x3f, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x7f, 
  0xc0, 0x00, 0x00, 0x00, 0x1f, 0xe0, 0x00, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0xe0, 
  0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x7f, 0xf8, 0x00, 0x03, 0xe0, 0x7f, 
  0xf8, 0x00, 0x01, 0xc0, 0x03, 0xf8, 0x00, 0x01, 0xc0, 0x03, 0xf8, 0x03, 0xc1, 0xc0, 0x03, 0xf8, 
  0x03, 0xe1, 0xc0, 0x03, 0xf8, 0x03, 0xc1, 0xc0, 0x03, 0xff, 0xff, 0xff, 0xfc, 0x03, 0xff, 0xff, 
  0xff, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xff, 
  0xfe, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xfe, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 
  0x9f, 0xe7, 0xf3, 0xfc, 0x3f, 0x9f, 0xe7, 0xf3, 0xfc, 0x3f, 0x9f, 0xe7, 0xf3, 0xfc, 0x3f, 0x8f, 
  0xc7, 0xf1, 0xf8, 0x1f, 0x0f, 0x83, 0xe0, 0xf0
};

#define DISPLAY_B1 0x3C
#define DISPLAY_B2 0x3D

void startScroll(unsigned char addr, bool left, bool right) {
  display.begin(SSD1306_SWITCHCAPVCC, addr); // Default OLED address, usually  
  display.clearDisplay(); 
  delay(100);
  if(left) {
    display.drawBitmap(0, 0, TrainLogoR, 40, 40, SSD1306_WHITE);
    display.startscrollleft(0x00,0x04);
  } else {
    display.drawBitmap(0, 0, TrainLogoL, 40, 40, SSD1306_WHITE);
    display.startscrollright(0x00,0x04);
  }
  delay(100);
  display.display();
}

void speedWrite(unsigned char addr, float speed, bool left, bool right) {
  char cstring[16];
  char cspeed[6];

  display.begin(SSD1306_SWITCHCAPVCC, addr); // Default OLED address, usually  
  display.clearDisplay();
  startScroll(addr, left, right);

  display.setTextSize(2);
  display.setTextColor(WHITE);
  dtostrf(speed, 4, 1, cspeed);
  cspeed[4] = '\0';
  snprintf(cstring,11,"V %s kmh",cspeed);
  cstring[11]='\0';
  display.setCursor(2,51);
  display.println(cstring);
  display.display();
}

void stopDisplay(unsigned char addr) {
  display.begin(SSD1306_SWITCHCAPVCC, addr); // Default OLED address, usually  
  display.stopscroll();
  display.clearDisplay();
  display.display();
}

//------------------------------------------- OLED Stuff -----------------------------------------

// the setup routine runs once when you press reset:
void setup() {
  // initialize pins
  // Bridge 1
  pinMode(redR_b1, OUTPUT);
  pinMode(lights_b1, OUTPUT);
  pinMode(redL_b1, OUTPUT);
  pinMode(sensR_b1, INPUT);
  pinMode(sensL_b1, INPUT);
  pinMode(tInR_b1, OUTPUT);
  pinMode(tInL_b1, OUTPUT);

  digitalWrite(redR_b1, HIGH);
  digitalWrite(lights_b1, HIGH);
  digitalWrite(redL_b1, HIGH);
  digitalWrite(tInR_b1, LOW);
  digitalWrite(tInL_b1, LOW);

  // Bridge 2
  pinMode(redR_b2, OUTPUT);
  pinMode(lights_b2, OUTPUT);
  pinMode(redL_b2, OUTPUT);
  pinMode(sensR_b2, INPUT);
  pinMode(sensL_b2, INPUT);
  pinMode(tInR_b2, OUTPUT);
  pinMode(tInL_b2, OUTPUT);

  digitalWrite(redR_b2, HIGH);
  digitalWrite(lights_b2, HIGH);
  digitalWrite(redL_b2, HIGH);
  digitalWrite(tInR_b2, LOW);
  digitalWrite(tInL_b2, LOW);

  Serial.begin(57600);
}

// the loop routine runs over and over again forever:
void loop() {
  const int nrep = 5;
  const double hnorm = 1. / nrep;

// Debouncing the sensors
  curR_b1 = 0;
  for (int i = 0; i < nrep; ++i) {
    curR_b1 += digitalRead(sensR_b1);
    delay(10);
  }
  curR_b1 = curR_b1 * hnorm + 0.5;
  stateR_b1 = curR_b1 + 2 * preR_b1;
  preR_b1 = curR_b1;

  curL_b1 = 0;
  for (int i = 0; i < nrep; ++i) {
    curL_b1 += digitalRead(sensL_b1);
    delay(10);
  }
  curL_b1 = curL_b1 * hnorm + 0.5;
  stateL_b1 = curL_b1 + 2 * preL_b1;
  preL_b1 = curL_b1;

  if (!(trainInL_b1 || trainInR_b1)) {

    // Is the train entering?
    trainInR_b1 = (stateR_b1 == kSet) && (stateL_b1 == kOff);
    trainInL_b1 = (stateR_b1 == kOff) && (stateL_b1 == kSet);
    
    if (trainInR_b1 || trainInL_b1) {
      fade_b1 = 1;
      timeStart_b1 = millis();
      startScroll(DISPLAY_B1,trainInR_b1,trainInL_b1);
    }
  } else {

    // Is the train exiting?
    if ((trainInR_b1 && (stateL_b1 == kSet)) ||
        (trainInL_b1 && (stateR_b1 == kSet))) {
        timeDelta_b1 = millis() - timeStart_b1;
        speedWrite(DISPLAY_B1,speedConv/timeDelta_b1,trainInR_b1,trainInL_b1);
      }
    if (trainInR_b1) trainInR_b1 = !(stateR_b1 == kOff && stateL_b1 == kClear);
    if (trainInL_b1) trainInL_b1 = !(stateR_b1 == kClear && stateL_b1 == kOff);
    if (!(trainInL_b1 || trainInR_b1))
      fade_b1 = -1;
  }

  if (trainInR_b1)
    digitalWrite(tInR_b1, HIGH);
  else
    digitalWrite(tInR_b1, LOW);

  if (trainInL_b1)
    digitalWrite(tInL_b1, HIGH);
  else
    digitalWrite(tInL_b1, LOW);

  if (fade_b1 != 0) {

    if (millis() - fademils_b1 > 50) {
      fademils_b1 = millis();

      // change the brightness for next time through the loop:
      brightness_b1 = brightness_b1 + fade_b1 * fadeAmount;

      // set the brightness bridge lights
      analogWrite(lights_b1, max(0, 255 - brightness_b1));

      // stop fading if we have reached max or min
      if (brightness_b1 <= 0) {
        brightness_b1 = 0;
        fade_b1 = 0;
      } else if (brightness_b1 >= 255) {
        brightness_b1 = 255;
        fade_b1 = 0;
      }
    }
  }
  if (brightness_b1 > 0) {
    if (millis() - blinkred_b1 > 500) {
      blinkred_b1 = millis();
      highlow_b1 = 1 - highlow_b1;
      digitalWrite(redR_b1, highlow_b1);
      digitalWrite(redL_b1, 1 - highlow_b1);
    }
  } else {
    digitalWrite(redR_b1, HIGH);
    analogWrite(lights_b1, 255);
    digitalWrite(redL_b1, HIGH);
    stopDisplay(DISPLAY_B1);
  }
}
