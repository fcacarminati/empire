//
// Sketch for two bridges
//
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "DFRobotDFPlayerMini.h"
#include <SoftwareSerial.h>
// Pin configuration
//
// Bridge 1
// D2  driver right red lights
// D9  driver bridge lights
// D3  driver left red lights
// D4  digital input for sensor 1R
// D5  digital input for sensor 1L
// A0 led for train in from right
// A1  led for train in from left
//
// Bridge 2
// D8  driver right red lights
// D10  driver bridge lights
// D11  driver left red lights
// D12 digital input for sensor 2R
// A2 digital input dor sensor 2L
// A3  led for train in from right
// A6  let for train in from left
//
// A4 SDA for OLED
// A5 SCL for OLED
//
// D6 RX channel for DFPlayer
// D7 TX channel for DFPlayer
//

// Bridge 1
const int redR_b1 = 2;   // pin for right red lights
const int lights_b1 = 9; // pin white lights
const int redL_b1 = 3;   // pin for left red lights
const int sensR_b1 = 4;  // right sensor
const int sensL_b1 = 5;  // left sensor
const int tInR_b1 = A0;  // led for train in from right
const int tInL_b1 = A1;  // led for train in from left
int brightness_b1 = 0;
unsigned long fademils_b1 = 0;
unsigned long blinkred_b1 = 0;
int highlow_b1 = 0;
short curR_b1 = 0;    // current state of right sensor
short curL_b1 = 0;    // current state of left sensor
short preR_b1 = 1;    // previous state of right sensor
short preL_b1 = 1;    // previous state of left sensor
short stateR_b1 = 0;  // logical state of right side
// 0 off; 1 cleared; 2 set; 3 on
short stateL_b1 = 0;  // logical state of left side
short fade_b1 = 0;    // fading direction
bool trainInL_b1 = false;
bool trainInR_b1 = false;
float timeStart_b1 = 0;
float timeDelta_b1 = 0;

// Bridge 2
int redR_b2 = 8;
int lights_b2 = 10;
int redL_b2 = 11;
int sensR_b2 = 12;
int sensL_b2 = A2;
int tInR_b2 = A3;  // led for train in from right
int tInL_b2 = A6;  // led for train in from left
int brightness_b2 = 0;
unsigned long fademils_b2 = 0;
unsigned long blinkred_b2 = 0;
int highlow_b2 = 0;
short curR_b2 = 0;
short curL_b2 = 0;
short preR_b2 = 1;
short preL_b2 = 1;
short stateR_b2 = 0;  // logical state of right side
// 0 off; 1 cleared; 2 set; 3 on
short stateL_b2 = 0;  // logical state of left side
short fade_b2 = 0;
bool trainInL_b2 = false;
bool trainInR_b2 = false;
float timeStart_b2 = 0;
float timeDelta_b2 = 0;

const int fadeAmount = 5;    // how many points to fade the LED by
const double bridgeLength = 20; // bridge length in cm
const double speedConv = bridgeLength * 87 * 1e-5 * (3600 * 1e3); 

enum State {kOn, kClear, kSet, kOff};
const char *State_name[] = {"On","Clear","Set","Off"};

#define DEBUG 1

//------------------------------------------- OLED Stuff -----------------------------------------
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for SSD1306 display connected using I2C
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define DISPLAY_B1 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define DISPLAY_B2 0x3D
//Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
unsigned char oldaddr = 0XFF;

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

SoftwareSerial softSerial(/*RX=*/ 6, /*TX=*/ 7);
DFRobotDFPlayerMini myDFPlayer;

void startScroll(unsigned char addr, bool left, bool right) {
 if(addr != oldaddr) {
#ifdef DEBUG
    Serial.println(F("StartScroll: Changing address"));
#endif
    display.begin(SSD1306_SWITCHCAPVCC, addr); // Default OLED address, usually  
    delay(500);
    display.clearDisplay(); 
    oldaddr = addr;
  }
  if(left) {
    display.drawBitmap(0, 0, TrainLogoR, 40, 40, SSD1306_WHITE);
    delay(500);
    display.startscrollleft(0x00,0x04);
  } else {
    display.drawBitmap(0, 0, TrainLogoL, 40, 40, SSD1306_WHITE);
    delay(500);
    display.startscrollright(0x00,0x04);
  }
  display.display();
}

void speedWrite(unsigned char addr, float speed, bool left, bool right) {
  char cstring[16];
  char cspeed[8];

 if(addr != oldaddr) {
#ifdef DEBUG
    Serial.println(F("SpeedWrite: Changing address"));
#endif
    display.begin(SSD1306_SWITCHCAPVCC, addr); // Default OLED address, usually  
    delay(500);
    display.clearDisplay(); 
    startScroll(addr, left, right);
    oldaddr = addr;
  }
  
  display.setTextSize(2);
  display.setTextColor(WHITE);
  dtostrf(speed, 4, 1, cspeed);
  cspeed[6] = '\0';
  snprintf(cstring,11,"%s km/h",cspeed);
  cstring[11]='\0';
  display.setCursor(1,51);
  display.print(cstring);
  display.display();
}

void stopDisplay(unsigned char addr) {
if(addr != oldaddr) {
#ifdef DEBUG
    Serial.println(F("stopDisplay: Changing address"));
#endif
    display.begin(SSD1306_SWITCHCAPVCC, addr); // Default OLED address, usually  
    delay(500);
    display.clearDisplay();
    oldaddr = addr;
  }
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
  pinMode(sensR_b1, INPUT_PULLUP);
  pinMode(sensL_b1, INPUT_PULLUP);
  pinMode(tInR_b1, OUTPUT);
  pinMode(tInL_b1, OUTPUT);

  digitalWrite(redR_b1, HIGH);
  digitalWrite(lights_b1, HIGH);
  digitalWrite(redL_b1, HIGH);
  digitalWrite(tInR_b1, LOW);
  digitalWrite(tInL_b1, LOW);

  fademils_b1 = millis();
  blinkred_b1 = millis();

  // Bridge 2
  pinMode(redR_b2, OUTPUT);
  pinMode(lights_b2, OUTPUT);
  pinMode(redL_b2, OUTPUT);
  pinMode(sensR_b2, INPUT_PULLUP);
  pinMode(sensL_b2, INPUT_PULLUP);
  pinMode(tInR_b2, OUTPUT);
  pinMode(tInL_b2, OUTPUT);

  digitalWrite(redR_b2, HIGH);
  digitalWrite(lights_b2, HIGH);
  digitalWrite(redL_b2, HIGH);
  digitalWrite(tInR_b2, LOW);
  digitalWrite(tInL_b2, LOW);

  fademils_b2 = millis();
  blinkred_b2 = millis();

  oldaddr = 0x00;
  
  Serial.begin(57600);

  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  softSerial.begin(9600);
  
  if (!myDFPlayer.begin(softSerial, /*isACK = */true, /*doReset = */true)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));
  
  myDFPlayer.volume(15);  //Set volume value. From 0 to 30
  myDFPlayer.play(2);  //Play the first mp3


}

// the loop routine runs over and over again forever:
void loop() {
  const int nrep = 5;
  const double hnorm = 1. / nrep;

  static int playercount = 0;

  static bool active_b1 = false;

#ifdef DEBUG
  static short statePreR_b1 = 99;
  static short statePreL_b1 = 99;
#endif

// Debouncing the sensors
  curR_b1 = 0;
  for (int i = 0; i < nrep; ++i) {
    curR_b1 += digitalRead(sensR_b1);
    delay(10);
  }
  curR_b1 = curR_b1 * hnorm + 0.5;
  stateR_b1 = curR_b1 + 2 * preR_b1;
#ifdef DEBUG
  if(curR_b1 != preR_b1) {
      Serial.print(F("B1: SensorR = "));
      Serial.println(curR_b1);
  } 
  if(stateR_b1 != statePreR_b1) {
    Serial.print(F("B1: State R "));
    Serial.println(State_name[stateR_b1]);
    statePreR_b1 = stateR_b1;
  }
#endif
  preR_b1 = curR_b1;

  curL_b1 = 0;
  for (int i = 0; i < nrep; ++i) {
    curL_b1 += digitalRead(sensL_b1);
    delay(10);
  }
  curL_b1 = curL_b1 * hnorm + 0.5;
  stateL_b1 = curL_b1 + 2 * preL_b1;
#ifdef DEBUG
  if(curL_b1 != preL_b1) {
      Serial.print(F("B1: SensorL = "));
      Serial.println(curL_b1);
  } 
  if(stateL_b1 != statePreL_b1) {
    Serial.print(F("B1: State L "));
    Serial.println(State_name[stateL_b1]);
    statePreL_b1 = stateL_b1;
  }
#endif 
  preL_b1 = curL_b1;

  if (!(trainInL_b1 || trainInR_b1)) {

    // Is the train entering?
    trainInR_b1 = (stateR_b1 == kSet) && (stateL_b1 == kOff);
    trainInL_b1 = (stateR_b1 == kOff) && (stateL_b1 == kSet);

#ifdef DEBUG
    if(trainInR_b1) Serial.println(F("B1: Train in Right"));
    if(trainInL_b1) Serial.println(F("B1: Train in Left"));
#endif
    
    if (trainInR_b1 || trainInL_b1) {
      active_b1 = true;
      fade_b1 = 1;
      timeStart_b1 = millis();
      startScroll(DISPLAY_B1,trainInR_b1,trainInL_b1);
      if(!playercount) 
        myDFPlayer.play(1);  //Play the first mp3
      ++playercount;

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
  } else if(active_b1) {
    digitalWrite(redR_b1, HIGH);
    analogWrite(lights_b1, 255);
    digitalWrite(redL_b1, HIGH);
    stopDisplay(DISPLAY_B1);
    if(!--playercount) myDFPlayer.stop();
    active_b1 = false;
  }


// Bridge 2

  static bool active_b2 = false;

#ifdef DEBUG
  static short statePreR_b2 = 99;
  static short statePreL_b2 = 99;
#endif

// Debouncing the sensors
  curR_b2 = 0;
  for (int i = 0; i < nrep; ++i) {
    curR_b2 += digitalRead(sensR_b2);
    delay(10);
  }
  curR_b2 = curR_b2 * hnorm + 0.5;
  stateR_b2 = curR_b2 + 2 * preR_b2;
#ifdef DEBUG
  if(curR_b2 != preR_b2) {
      Serial.print(F("B2: SensorR = "));
      Serial.println(curR_b2);
  } 
  if(stateR_b2 != statePreR_b2) {
    Serial.print(F("B2: State R "));
    Serial.println(State_name[stateR_b2]);
    statePreR_b2 = stateR_b2;
  }
#endif
  preR_b2 = curR_b2;

  curL_b2 = 0;
  for (int i = 0; i < nrep; ++i) {
    curL_b2 += digitalRead(sensL_b2);
    delay(10);
  }
  curL_b2 = curL_b2 * hnorm + 0.5;
  stateL_b2 = curL_b2 + 2 * preL_b2;
#ifdef DEBUG
  if(curL_b2 != preL_b2) {
      Serial.print(F("B2: SensorL = "));
      Serial.println(curL_b2);
  } 
  if(stateL_b2 != statePreL_b2) {
    Serial.print(F("B2: State L "));
    Serial.println(State_name[stateL_b2]);
    statePreL_b2 = stateL_b2;
  }
#endif 
  preL_b2 = curL_b2;

  if (!(trainInL_b2 || trainInR_b2)) {

    // Is the train entering?
    trainInR_b2 = (stateR_b2 == kSet) && (stateL_b2 == kOff);
    trainInL_b2 = (stateR_b2 == kOff) && (stateL_b2 == kSet);

#ifdef DEBUG
    if(trainInR_b2) Serial.println(F("B2: Train in Right"));
    if(trainInL_b2) Serial.println(F("B2: Train in Left"));
#endif
    
    if (trainInR_b2 || trainInL_b2) {
      active_b2 = true;
      fade_b2 = 1;
      timeStart_b2 = millis();
      startScroll(DISPLAY_B2,trainInR_b2,trainInL_b2);
      if(!playercount)
        myDFPlayer.play(1);  //Play the first mp3
      ++playercount;
    }
  } else {

    // Is the train exiting?
    if ((trainInR_b2 && (stateL_b2 == kSet)) ||
        (trainInL_b2 && (stateR_b2 == kSet))) {
        timeDelta_b2 = millis() - timeStart_b2;
        speedWrite(DISPLAY_B2,speedConv/timeDelta_b2,trainInR_b2,trainInL_b2);
      }
    if (trainInR_b2) trainInR_b2 = !(stateR_b2 == kOff && stateL_b2 == kClear);
    if (trainInL_b2) trainInL_b2 = !(stateR_b2 == kClear && stateL_b2 == kOff);
    if (!(trainInL_b2 || trainInR_b2))
      fade_b2 = -1;
  }

  if (trainInR_b2)
    digitalWrite(tInR_b2, HIGH);
  else
    digitalWrite(tInR_b2, LOW);

  if (trainInL_b2)
    digitalWrite(tInL_b2, HIGH);
  else
    digitalWrite(tInL_b2, LOW);

  if (fade_b2 != 0) {

    if (millis() - fademils_b2 > 50) {
      fademils_b2 = millis();

      // change the brightness for next time through the loop:
      brightness_b2 = brightness_b2 + fade_b2 * fadeAmount;

      // set the brightness bridge lights
      analogWrite(lights_b2, max(0, 255 - brightness_b2));

      // stop fading if we have reached max or min
      if (brightness_b2 <= 0) {
        brightness_b2 = 0;
        fade_b2 = 0;
      } else if (brightness_b2 >= 255) {
        brightness_b2 = 255;
        fade_b2 = 0;
      }
    }
  }
  if (brightness_b2 > 0) {
    if (millis() - blinkred_b2 > 500) {
      blinkred_b2 = millis();
      highlow_b2 = 1 - highlow_b2;
      digitalWrite(redR_b2, highlow_b2);
      digitalWrite(redL_b2, 1 - highlow_b2);
    }
  } else if (active_b2) {
    digitalWrite(redR_b2, HIGH);
    analogWrite(lights_b2, 255);
    digitalWrite(redL_b2, HIGH);
    stopDisplay(DISPLAY_B2);
    if(!--playercount) myDFPlayer.stop();
    active_b2 = false;
  }
}
