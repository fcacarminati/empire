/*

                                       https://www.youtube.com/channel/UCvFLDXbmsceBIvblMLXwsWA

The MIT License (MIT)

Copyright (c) 2017 Sipos Péter

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
#include <Adafruit_GFX.h> // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <TouchScreen.h> //touch library

#define LCD_CS A3    // Chip Select goes to Analog 3
#define LCD_CD A2    // Command/Data goes to Analog 2
#define LCD_WR A1    // LCD Write goes to Analog 1
#define LCD_RD A0    // LCD Read goes to Analog 0
#define LCD_RESET A4 // you can also just connect RESET to the arduino RESET pin. *if so just //comment line out

#define YP A3  // must be an analog pin, use "An" notation!  A1 for shield
#define XM A2  // must be an analog pin, use "An" notation!  A2 for shield
#define YM 9   // can be a digital pin-----uno=9 mega=23      7 for shield
#define XP 8   // can be a digital pin-----uno=8 mega=22      6 for shield

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);   

float V1 = {0.00};
float volt1;
// Color definitions - in 5:6:5
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0 
#define WHITE           0xFFFF
//No Majf
#define MINPRESSURE 10
#define MAXPRESSURE 1000

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

int high_time;
int low_time;
float time_period;
float frequency;

const uint16_t LCD_WIDTH = tft.width();
const uint16_t LCD_HEIGHT = tft.height();
#define SAMPLES 270
#define DOTS_DIV 30

#define ad_ch0 4                   // Analog 4 pin for channel 0
#define ad_ch1 5                   // Analog 5 pin for channel 1
const uint16_t VREF[] = {150, 300, 750, 1500, 3000};

const byte MILLIVOL_per_dot[] = {33, 17, 6, 3, 2}; // mV/dot
#define MODE_ON 0
#define MODE_INV 1
#define MODE_OFF 2
const char *Modes[] = {"NORM", "INV", "OFF"};
#define TRIG_AUTO 0
#define TRIG_NORM 1
#define TRIG_SCAN 2
#define TRIG_ONE  3
const char *TRIG_Modes[] = {"Auto", "Norm", "Scan", "One"};
#define TRIG_E_UP 0
#define TRIG_E_DN 1
#define RATE_MIN 0
#define RATE_MAX 13
const char *Rates[] = {"F1-1", "F1-2 ", "F2  ", "5ms", "10ms", "20ms", "50ms", "0.1s", "0.2s", "0.5s", "1s", "2s", "5s", "10s"};
#define RANGE_MIN 0
#define RANGE_MAX 4
const char *Ranges[] = {" 1V ", "0.5V", "0.2V", "0.1V", "50mV"};
byte data[4][SAMPLES];                   
byte sample=0;                           

///////////////////////////////////////////////////////////////////////////////////////////////
// Define colors here
#define BGCOLOR  BLACK
#define GRIDCOLOR WHITE
#define CH1COLOR  BLUE
#define CH2COLOR  YELLOW

#define VRF 5

// #define CALIBRATE


byte range0 = RANGE_MIN, ch0_mode = MODE_ON;  // CH0
short ch0_off = 204;
byte range1 = RANGE_MIN, ch1_mode = MODE_ON;  // CH1
short ch1_off = 204;
byte rate = 1;                                // sampling rate
byte trig_mode = TRIG_AUTO, trig_lv = 30, trig_edge = TRIG_E_UP, trig_ch = 1; // trigger settings
byte Start = 1;  // Start sampling
byte menu = 0;  // Default menu

// Calibration stuff
#ifdef CALIBRATE
#define text_x_center (LCD_HEIGHT / 2) - 4
#define text_y_center (LCD_WIDTH / 2) - 6
uint16_t rx[8], ry[8];
#endif
TSPoint tp;
#ifdef CALIBRATE
int16_t clx=853;
int16_t crx=113;
int16_t cty=174;
int16_t cby=862;
float px=-0.40431265;
float py=0.32163741;
#else
#define clx 853
#define crx 113
#define cty 174
#define cby 862
#define px -0.40431265
#define py 0.32163741
#endif

#ifdef CALIBRATE
void Calibrate() {
    tft.fillScreen(BLACK);

    drawCrossHair(LCD_HEIGHT - 11, 10,0x528A);
    drawCrossHair(LCD_HEIGHT / 2, 10,0x528A);
    drawCrossHair(10, 10,0x528A);
    drawCrossHair(LCD_HEIGHT - 11, LCD_WIDTH / 2,0x528A);
    drawCrossHair(10, LCD_WIDTH / 2,0x528A);
    drawCrossHair(LCD_HEIGHT - 11, LCD_WIDTH - 11,0x528A);
    drawCrossHair(LCD_HEIGHT / 2, LCD_WIDTH - 11,0x528A);
    drawCrossHair(10, LCD_WIDTH - 11,0x528A);

    calibratePoint(10, 10, 0);
    calibratePoint(10, LCD_WIDTH / 2, 1);
    calibratePoint(10, LCD_WIDTH - 11, 2);
    calibratePoint(LCD_HEIGHT / 2, 10, 3);
    calibratePoint(LCD_HEIGHT / 2, LCD_WIDTH - 11, 4);
    calibratePoint(LCD_HEIGHT - 11, 10, 5);
    calibratePoint(LCD_HEIGHT - 11, LCD_WIDTH / 2, 6);
    calibratePoint(LCD_HEIGHT - 11, LCD_WIDTH - 11, 7);

    clx = (ry[0] + ry[1] + ry[2]) / 3;
    crx = (ry[5] + ry[6] + ry[7]) / 3;
    cty = (rx[0] + rx[3] + rx[5]) / 3;
    cby = (rx[2] + rx[4] + rx[7]) / 3;
    
    px =  (LCD_HEIGHT - 20) / float(crx - clx);
    py =  (LCD_WIDTH - 20) / float(cby - cty) ;
/*
    Serial.println("New Calibration values:");
    Serial.print("clx=\t");
    Serial.print(clx);
    Serial.print("\tcrx=\t");
    Serial.println(crx);
    Serial.print("cty=\t");
    Serial.print(cty);
    Serial.print("\tcby=\t");
    Serial.println(cby);
    Serial.print("px=\t");
    Serial.print(px,6);
    Serial.print("\tpy=\t");
    Serial.println(py,6);
    */

#ifdef NEVER
    int16_t x, y;
    Serial.println(px);
    Serial.println(py);
    while(1) {
      if(isPressed()){
      readResistive();
      mapCoord(&x, &y);
      Serial.print("(");
      Serial.print(tp.x);
      Serial.print(",");
      Serial.print(tp.y);
      Serial.print(")->(");
      Serial.print(x);
      Serial.print(",");
      Serial.print(y);
      Serial.println(")");
      }
    }
#endif
}

void writeCenterText(const char *text, uint16_t fg, uint16_t bg, uint8_t textsize) {
  tft.setTextColor(fg,bg);
  tft.setTextSize(textsize);
  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds(text,0,0,&x1,&y1,&tw,&th);
  tft.fillRect(text_x_center-tw/2, text_y_center-th/2, tw,th,BLACK);
  tft.setCursor(text_x_center-tw/2, text_y_center-th/2);
  tft.print(text);
}

void calibratePoint(int x, int y, byte i)
{
  drawCrossHair(x, y, WHITE);
  readCoordinates(i);
  writeCenterText("* RELEASE *", BLUE, BLACK, 2);
  while (isPressed()) {}
  drawCrossHair(x, y,0x528A);
}

void drawCrossHair(int x, int y, unsigned int color)
{
   tft.drawFastVLine(x-10, y-10, 20, color);
   tft.drawFastVLine(x+10, y-10, 20, color);
   tft.drawFastHLine(x-10, y-10, 20, color);
   tft.drawFastHLine(x-10, y+10, 20, color);
   tft.drawFastVLine(x, y-5, 11, color);
   tft.drawFastHLine(x-5, y, 11, color);
}

void readCoordinates(byte i)
{
#define iter 500
  const float norm = 1./iter;
  uint16_t failcount = 0;
  uint16_t cnt = 0;
  uint32_t tx = 0;
  uint32_t ty = 0;
    
  writeCenterText("*  PRESS  *", GREEN, BLACK, 2);
  while (!isPressed()) {}
  writeCenterText("*  HOLD!  *", RED, BLACK, 2);
  while (cnt<iter) {
    readResistive();
    if (tp.z > 20 && tp.z < 1000) {
      tx += tp.x;
      ty += tp.y;
      cnt++;
    } else {
      failcount++;
      if (failcount >= 10000)
        exit;
    } 
  }
  rx[i] = tx * norm;
  ry[i] = ty * norm;
}

#endif

bool isPressed(void)
{
    float avp = 0;
    const int niter=10;
    const float norm = 1./niter;
    for(uint8_t i=0; i<niter; ++i) {
      readResistive();
      avp += tp.z;
    }
    avp *= norm;
    return (avp > MINPRESSURE && avp < MAXPRESSURE);
}

void readResistive(void)
{
    tp = ts.getPoint();
    if(tp.z<MINPRESSURE) {
      tp.x=-0;
      tp.y=-0;
    }
    pinMode(YP, OUTPUT);      //restore shared pins
    pinMode(XM, OUTPUT);
    digitalWrite(YP, HIGH);   //because TFT control pins
    digitalWrite(XM, HIGH);
}

void mapCoord(int16_t *x, int16_t *y) {
  *x = (tp.y-clx) * px + 10;
  *y = (tp.x-cty) * py + 10;
}

void setup(){
Serial.begin(9600);
  
  tft.reset();
  
  uint16_t identifier = tft.readID();
  uint16_t default_identifier = 0x9341;
//  default_identifier = 0x9486;
  Serial.println(identifier);

/* 
    if(identifier == 0x9325) {
    Serial.println(F("ILI9325 LCD driver"));
  } else if(identifier == 0x9327) {
    Serial.println(F("ILI9327 LCD driver"));
  } else if(identifier == 0x9328) {
    Serial.println(F("ILI9328 LCD driver"));
  } else if(identifier == 0x7575) {
    Serial.println(F("HX8347G LCD driver"));
  } else if(identifier == 0x9341) {
    Serial.println(F("ILI9341 LCD driver"));
  } else if(identifier == 0x8357) {
    Serial.println(F("HX8357D LCD driver"));
  } else if(identifier == 0x0154) {
    Serial.println(F("S6D0154 LCD driver"));
  } else {
    Serial.print(F("Unknown LCD driver ic: "));
    Serial.println(identifier, HEX);
    Serial.println("Defaulting to ILI9341 LCD driver");
 */
    identifier = default_identifier;
//  }
  Serial.println(tft.height());
  Serial.println(tft.width());

  tft.begin(identifier);

  tft.setRotation(1);

  tft.fillScreen(BLACK);

  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.setCursor(40, 30);
  tft.print("Arduino Oscilloscope");
  tft.setTextColor(WHITE);
  tft.setTextColor(GREEN);
  tft.setTextSize(2);
  tft.setCursor(1, 60);
  tft.print("Original code: Sipos Peter");
#ifdef CALIBRATE
  tft.setCursor(30,160);
  tft.print("Calibrate?");
  tft.fillRect(170,145,40,40,GREEN);
  tft.setCursor(173,157);
  tft.setTextSize(2);
  tft.setTextColor(BLACK);
  tft.print("Yes");

  tft.fillRect(210,145,40,40,RED);
  tft.setCursor(220,157);
  tft.setTextSize(2);
  tft.setTextColor(BLACK);
  tft.print("No");

  unsigned long startTime = millis();

  bool calibrate = false;

  while(millis()-startTime < 10000) {
    if(isPressed()) {
      int16_t x, y;
      readResistive();
      mapCoord(&x, &y);
      calibrate = (x>170 && x<210 && y>145 && y<185);
      break;
    }
  }

  if(calibrate) {
    delay(500);
    Calibrate();
  }
#else
  delay(5000);
#endif

  tft.fillScreen(BGCOLOR);
   
  Serial.begin(9600);

#ifndef CALIBRATE
  DrawGrid();
  DrawText();
#endif
}

/*
void SendData() {
  Serial.print(Rates[rate]);
  Serial.println("/div (30 samples)");
  for (int i=0; i<SAMPLES; i ++) {
      Serial.print(data[sample + 0][i]*MILLIVOL_per_dot[range0]);
      Serial.print(" ");
      Serial.println(data[sample + 1][i]*MILLIVOL_per_dot[range1]);
   } 
}
*/

void DrawGrid() {
    for (int x=0; x<=SAMPLES; x += 2) { // Horizontal Line
      for (int y=0; y<=LCD_HEIGHT; y += DOTS_DIV) {
        tft.drawPixel(x, y, GRIDCOLOR);

      }

        tft.drawPixel(x, LCD_HEIGHT-1, GRIDCOLOR);
    }
    for (int x=0; x<=SAMPLES; x += DOTS_DIV ) { // Vertical Line
      for (int y=0; y<=LCD_HEIGHT; y += 2) {
        tft.drawPixel(x, y, GRIDCOLOR);

      }
    }
}

void DrawText() {
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.setCursor(SAMPLES+3, 20);
  tft.print(Ranges[range1]);
  tft.println("/DIV");
  tft.setCursor(SAMPLES+3, 30);
  tft.print(Rates[rate]);
  tft.println("/DIV");
  tft.setCursor(SAMPLES+3, 40);
  tft.println(TRIG_Modes[trig_mode]);
  tft.setCursor(SAMPLES+3, 50);
  tft.println(trig_edge == TRIG_E_UP ? "UP" : "DN"); 
  tft.setCursor(SAMPLES+3, 60);
  tft.println(Modes[ch1_mode]);
}

void DrawGrid(int x) {
    if ((x % 2) == 0)
      for (int y=0; y<=LCD_HEIGHT; y += DOTS_DIV)
        tft.drawPixel(x, y, GRIDCOLOR);
    if ((x % DOTS_DIV) == 0)
      for (int y=0; y<=LCD_HEIGHT; y += 2)
        tft.drawPixel(x, y, GRIDCOLOR);
}

void ClearAndDrawGraph() {
  int clear = 0;
  
  if (sample == 0)
    clear = 2;
   for (int x=0; x<(SAMPLES-1); x++) {
     tft.drawLine(x, LCD_HEIGHT-data[clear+0][x], x+1, LCD_HEIGHT-data[clear+0][x+1], BGCOLOR);
     tft.drawLine(x, LCD_HEIGHT-data[clear+1][x], x+1, LCD_HEIGHT-data[clear+1][x+1], BGCOLOR);
     if (ch0_mode != MODE_OFF)
       tft.drawLine(x, LCD_HEIGHT-data[sample+0][x], x+1, LCD_HEIGHT-data[sample+0][x+1], CH1COLOR);
     if (ch1_mode != MODE_OFF)
       tft.drawLine(x, LCD_HEIGHT-data[sample+1][x], x+1, LCD_HEIGHT-data[sample+1][x+1], CH2COLOR);

  } 
}

void ClearAndDrawDot(int i) {
  int clear = 0;

  if (i <= 1)
    return;
  if (sample == 0)
    clear = 2;
  tft.drawLine(i-1, LCD_HEIGHT-data[clear+0][i-1], i, LCD_HEIGHT-data[clear+0][i], BGCOLOR);
  tft.drawLine(i-1, LCD_HEIGHT-data[clear+1][i-1], i, LCD_HEIGHT-data[clear+1][i], BGCOLOR);
  if (ch0_mode != MODE_OFF)
    tft.drawLine(i-1, LCD_HEIGHT-data[sample+0][i-1], i, LCD_HEIGHT-data[sample+0][i], CH1COLOR);
  if (ch1_mode != MODE_OFF)
    tft.drawLine(i-1, LCD_HEIGHT-data[sample+1][i-1], i, LCD_HEIGHT-data[sample+1][i], CH2COLOR);
  DrawGrid(i);
}

void DrawGraph() {
   for (int x=0; x<SAMPLES; x++) {
     tft.drawPixel(x, LCD_HEIGHT-data[sample+0][x], CH1COLOR);
     tft.drawPixel(x, LCD_HEIGHT-data[sample+1][x], CH2COLOR);
  }
}

void ClearGraph() {
  int clear = 0;
  
  if (sample == 0)
    clear = 2;
  for (int x=0; x<SAMPLES; x++) {
     tft.drawPixel(x, LCD_HEIGHT-data[clear+0][x], BGCOLOR);
     tft.drawPixel(x, LCD_HEIGHT-data[clear+1][x], BGCOLOR);
  }
}

inline unsigned long adRead(byte ch, byte mode, int off)
{
  unsigned long a = analogRead(ch);
  a = ((a+off)*VREF[ch == ad_ch0 ? range0 : range1]+512) >> 10;
  a = a>=(LCD_HEIGHT+1) ? LCD_HEIGHT : a;
  if (mode == MODE_INV)
    return LCD_HEIGHT - a;
  return a;
}


void  loop() {
  if (trig_mode != TRIG_SCAN) {
      unsigned long st = millis();
      byte oad;
      if (trig_ch == 0)
        oad = adRead(ad_ch0, ch0_mode, ch0_off);
      else
        oad = adRead(ad_ch1, ch1_mode, ch1_off);
      for (;;) {
        byte ad;
        if (trig_ch == 0)
          ad = adRead(ad_ch0, ch0_mode, ch0_off);
        else
          ad = adRead(ad_ch1, ch1_mode, ch1_off);

        if (trig_edge == TRIG_E_UP) {
           if (ad >= trig_lv && ad > oad)
            break; 
        } else {
           if (ad <= trig_lv && ad < oad)
            break; 
        }
        oad = ad;


        if (trig_mode == TRIG_SCAN)
          break;
        if (trig_mode == TRIG_AUTO && (millis() - st) > 100)
          break; 
      }
  }
  

    if (rate <= 5 && Start) {
    if (sample == 0)
      sample = 2;
    else
      sample = 0;

    if (rate == 0) { 
      unsigned long st = millis();
      for (int i=0; i<SAMPLES; i ++) {
        data[sample+0][i] = adRead(ad_ch0, ch0_mode, ch0_off);
      }
      for (int i=0; i<SAMPLES; i ++)
        data[sample+1][i] = 0;
    } else if (rate == 1) { 
      unsigned long st = millis();
      for (int i=0; i<SAMPLES; i ++) {
        data[sample+1][i] = adRead(ad_ch1, ch1_mode, ch1_off);
      }
      for (int i=0; i<SAMPLES; i ++)
        data[sample+0][i] = 0;
    } else if (rate == 2) { 
      unsigned long st = millis();
      for (int i=0; i<SAMPLES; i ++) {
        data[sample+0][i] = adRead(ad_ch0, ch0_mode, ch0_off);
        data[sample+1][i] = adRead(ad_ch1, ch1_mode, ch1_off);
      }

    } else if (rate >= 3 && rate <= 5) {
      const unsigned long r_[] = {5000/DOTS_DIV, 10000/DOTS_DIV, 20000/DOTS_DIV};
      unsigned long st0 = millis();
      unsigned long st = micros();
      unsigned long r = r_[rate - 3];
      for (int i=0; i<SAMPLES; i ++) {
        while((st - micros())<r) ;
        st += r;
        data[sample+0][i] = adRead(ad_ch0, ch0_mode, ch0_off);
        data[sample+1][i] = adRead(ad_ch1, ch1_mode, ch1_off);
      }

    }
    ClearAndDrawGraph();

    DrawGrid();
    DrawText();
  } else if (Start) {

    if (sample == 0) {
      for (int i=0; i<SAMPLES; i ++) {
        data[2][i] = data[0][i];
        data[3][i] = data[1][i];
      }
    } else {
      for (int i=0; i<SAMPLES; i ++) {
        data[0][i] = data[2][i];
        data[1][i] = data[3][i];
      }      
    }

    const unsigned long r_[] = {50000/DOTS_DIV, 100000/DOTS_DIV, 200000/DOTS_DIV,
                      500000/DOTS_DIV, 1000000/DOTS_DIV, 2000000/DOTS_DIV, 
                      5000000/DOTS_DIV, 10000000/DOTS_DIV};
    unsigned long st0 = millis();
    unsigned long st = micros();
    for (int i=0; i<SAMPLES; i ++) {
      while((st - micros())<r_[rate-6]) {

        if (rate<6)
          break;
      }
      if (rate<6) {
        tft.fillScreen(BGCOLOR);
        break;
      }
      st += r_[rate-6];
      if (st - micros()>r_[rate-6])
          st = micros();
      if (!Start) {
         i --;
         continue;
      }
      data[sample+0][i] = adRead(ad_ch0, ch0_mode, ch0_off);
      data[sample+1][i] = adRead(ad_ch1, ch1_mode, ch1_off);
      ClearAndDrawDot(i);     
    }
    // Serial.println(millis()-st0);
    DrawGrid();
    DrawText();
  } else {

  }
  if (trig_mode == TRIG_ONE)
    Start = 0;

  int16_t x, y;

  readResistive();
  mapCoord(&x, &y);
   
     // s/div buttin
  const char* SDIV="S/DIV";
  tft.fillRect(275, 75, 40, 20, RED);
  tft.setTextSize(1);
  tft.setTextColor(WHITE);
  tft.setCursor(280, 80);
  tft.println(SDIV);
  if (x > 275 && x < 305 && y > 75 && y < 95) {     
    tft.fillRect(275, 75, 40, 20, GREEN);
    tft.setTextSize(1);
    tft.setTextColor(WHITE);
    tft.setCursor(280, 80);
    tft.println(SDIV);
    delay(100);
    tft.fillRect(275, 75, 40, 20, RED);
    tft.setTextColor(GREEN); 
    tft.setCursor(280, 80);  
    tft.println(SDIV);     
    tft.fillScreen(BLACK);
    if (rate < RATE_MAX) {
      rate ++;
    } else if (rate = RATE_MAX) {
      rate = RATE_MIN;
    } 
  }
//s/div end

//v/div button
   
  tft.fillRect(275, 100, 40, 20, RED);
  tft.setTextSize(1);
  tft.setTextColor(WHITE);
  tft.setCursor(280, 105);
  tft.println("V/DIV");
  if (x > 275 && x < 315 && y > 100 && y < 120) {
    tft.fillRect(275, 100, 40, 20, GREEN);
    tft.setTextSize(1);
    tft.setTextColor(WHITE);
    tft.setCursor(280, 105);
    tft.println("V/DIV");
    delay(100);
    tft.fillRect(275, 100, 40, 20, RED);
    tft.setTextColor(GREEN);
    tft.setCursor(280, 105);
    tft.println("V/DIV");
    tft.fillScreen(BLACK);
    if (range1 < RANGE_MAX) {
      range1 ++;
    } else if (range1 = RANGE_MAX) {
      range1 = RANGE_MIN;
    } 
  }
//v/div end

//TRIG BUTTON
  tft.fillRect(275, 125, 40, 20, RED);
  tft.setTextSize(1);
  tft.setTextColor(WHITE);
  tft.setCursor(280, 130);
  tft.println("TRIG");
  if (x > 275 && x < 315 && y > 125 && y < 145) {
    tft.fillRect(275, 125, 40, 20, GREEN);
    tft.setTextSize(1);
    tft.setTextColor(WHITE);
    tft.setCursor(280, 130);
    tft.println("TRIG");
    delay(100);
    tft.fillRect(275, 130, 40, 20, RED);
    tft.setTextColor(GREEN); 
    tft.setCursor(280, 130);  
    tft.println("TRIG");  
    tft.fillScreen(BLACK);
    if (trig_mode < TRIG_SCAN) {
      trig_mode ++;
    } else if (trig_mode = TRIG_SCAN) {
      trig_mode = TRIG_AUTO;
    } 
  }
//TRIG END

//START
  tft.fillRect(275, 150, 40, 20, RED);
  tft.setTextSize(1);
  tft.setTextColor(WHITE);
  tft.setCursor(280, 155);
  tft.println("START");
  if (x > 275 && x < 315 && y > 150 && y < 170) {
    tft.fillRect(275, 150, 40, 20, GREEN);
    tft.setTextSize(1);
    tft.setTextColor(WHITE);
    tft.setCursor(280, 155);
    tft.println("START");
    delay(100);
    tft.fillRect(275, 150, 40, 20, RED);
    tft.setTextColor(GREEN);
    tft.setCursor(280, 155); 
    tft.println("START");  
    Start = 1;
  }
//START END

//STOP
  tft.fillRect(275, 175, 40, 20, RED);
  tft.setTextSize(1); 
  tft.setTextColor(WHITE);
  tft.setCursor(280, 180);
  tft.println("STOP");
  if (x > 275 && x < 315 && y > 175 && y < 195) {
    tft.fillRect(275, 175, 40, 20, GREEN);
    tft.setTextSize(1);
    tft.setTextColor(WHITE);
    tft.setCursor(280, 180);
    tft.println("STOP");
    delay(100);
    tft.fillRect(275, 175, 40, 20, RED);
    tft.setTextColor(GREEN); 
    tft.setCursor(280, 180);
    tft.println("STOP");
    Start = 0;
  }
//STOP END

//FEL
  tft.fillRect(275, 200, 40, 20, RED);
  tft.setTextSize(1);
  tft.setTextColor(WHITE);
  tft.setCursor(280, 205);
  tft.println("FEL");
  if (x > 275 && x < 315 && y > 200 && y < 220) {
    tft.fillRect(275, 200, 40, 20, GREEN);
    tft.setTextSize(1);
    tft.setTextColor(WHITE);
    tft.setCursor(280, 205);
    tft.println("FEL");
    delay(100);
    tft.fillRect(275, 200, 40, 20, RED);
    tft.setTextColor(GREEN);
    tft.setCursor(280, 205);
    tft.println("FEL");
    ch1_off += 1024/VREF[range1];
  }
//FEL END

//LE
  tft.fillRect(275, 225, 40, 20, RED);
  tft.setTextSize(1);
  tft.setTextColor(WHITE);
  tft.setCursor(280, 230);
  tft.println("LE");
  if (x > 275 && x < 315 && y > 225 && y < 245) {
    tft.fillRect(275, 225, 40, 20, GREEN);
    tft.setTextSize(1);
    tft.setTextColor(WHITE);
    tft.setCursor(280, 230);
    tft.println("LE");
    delay(100);
    tft.fillRect(275, 225, 40, 20, RED);
    tft.setTextColor(GREEN);  
    tft.setCursor(280, 230);  
    tft.println("LE");
    ch1_off -= 1024/VREF[range1];
}
//LE END
//FREKVENCIA
  high_time=pulseIn(A5,HIGH);
  low_time=pulseIn(A5,LOW);
 
  time_period=high_time+low_time;
  time_period=time_period/1000;
  frequency=1000/time_period;
  tft.fillRect(5, 230, 120, 40, BLACK);
  tft.setCursor(8, 230);
  tft.println("f="); 
  tft.setCursor(20, 230);
  tft.println(frequency);
//FREKVENCIA END

//VOLT
  V1= analogRead(5);
  volt1 = (((V1*VRF) / 1024));
  tft.setCursor(75, 230);
  tft.println("U=");
  tft.setCursor(90, 230);
  tft.println(volt1);
//VOLT END
}
