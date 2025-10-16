/*
   This is the code for the Bismark Tunnel. 
   There are three switches two sensors and the illumination of the tunnel
   Here are the assignments:
   D2  => Digital control servo 1
   D3  => DCC relay servo 1
   D4  => Button servo 1
   D5  => Digital control servo 2
   D6  => Digital relay servo 2
   D7  => Button servo 2
   D8  => Digital control servo 3
   D9  => Digital relay servo 3
   D10  => Button servo 3 
*/
#define DEBUG
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Switch.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint16_t dw = -1;
uint16_t dh = -1;
const double degrad = acos(-1.)/180;
const uint16_t yoff = 6;
uint8_t boldw = 3;
 
void drawArc(int xc, int yc, int rad, int deg0, int deg1, bool bold) {
  int16_t nstep = 2*(deg1-deg0);
  int16_t beg = 0;
  int16_t end = 1;
  if(bold) {
    beg = -boldw;
    end =  boldw;
  }
  for(int j=beg; j<end; ++j) {
    for(int16_t i=0; i<= nstep; ++i) {
      double ang = (deg0 + 0.5*i)*degrad;
      uint8_t xp = xc + (rad+j) * cos(ang) + 0.5;
      uint8_t yp = yc + (rad+j) * sin(ang) + 0.5;
      display.drawPixel(xp,yp,SSD1306_WHITE);
    }
  }
}

void drawSlip(int state) {
  // start of diagonal branch
  uint16_t x0 = 0.5*dw;
  uint16_t y0 = 0.5938*dh;
  // end of diagonal branch
  uint16_t x1 = 0.8516*dw;
  uint16_t y1 = 0.9219*dh;
  // bold indicators for straight lines
  int8_t beg = 0;
  int8_t end = 1;
 
  state >>= 1;

  /*
  * Diagonal line
  */
  if(state == 3) {
    beg = -boldw;
    end =  boldw;
  }
  for(int8_t i=beg; i<end; ++i){
    display.drawLine(x0,y0+i,x1,y1+i,SSD1306_WHITE);
  }

  /*
  * lower arch
  */
  double rad = 1.8750*dh;
  int deg = 26;
  uint16_t xc1 = 0.4375*dw;
  uint16_t yc1 = 0.75*dh+rad;
  
  bool arcbold = false;
  if(state == 1) arcbold = true;
  drawArc(xc1,yc1,rad,270,270+deg,arcbold);

  /*
  * Upper arch
  */
  int16_t xc2 = 0.8750*dw;
  int16_t yc2 = 0.75*dh-rad;

  arcbold = false;
  if(state == 2) arcbold = true;
  drawArc(xc2,yc2,rad,90,90+deg,arcbold);

  /*
  * horizontal branch
  */
  beg = 0;
  end = 1;
  if (state == 0) {
    beg = -boldw;
    end =  boldw;
  }

  yc1 = 0.7604*dh;
  for(int8_t i=beg; i<end; ++i) {
    display.drawLine(xc1,yc1+i,xc2,yc1+i,SSD1306_WHITE);
  }
}

void drawTurnout(int16_t status) {
  double rad = 1.8750*dh;
  double xc = 0.1094*dw;
  double yc = 0.4219*dh+rad;

  int deg = 26;

  int8_t beg = -boldw;
  int8_t end =  boldw;
  bool arcbold = false;
  if( status == -1) {
    beg = 0;
    end = 1;
  } else if( 1 & status) {
    beg = 0;
    end = 1;
    arcbold = true;
  }

  uint16_t y1 = 0.4271*dh;
  for(int8_t i = beg; i<end; ++i) {
    display.drawLine(xc,y1+i,0.5*dw,y1+i,SSD1306_WHITE);
  }
  drawArc(xc,yc,rad,270,270+deg,arcbold);
}

void writeRoute(const char* mess){
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  int16_t x, y;
  uint16_t w, h;
  display.getTextBounds(mess, 0, 0, &x, &y, &w, &h); // compute size
  int16_t xpos = (display.width() - w) / 2;
  int16_t ypos = 3; // top line
  display.setCursor(xpos, ypos);
  display.print(mess);
}

/*
 * 
 */
int half1 = 1200;
int right1 = -180;
int curve1 = -200;
Switch Switch1(2,3,4,half1+right1,half1-curve1);
int half2 = 1600;
int right2 = 65;
int curve2 = 100;
Switch Switch2(5,6,7,half2+right2,half2-curve2);
int half3 = 1450;
int right3 = -80;//-100;
int curve3 = -95;//-80;
Switch Switch3(8,9,10,half3+right3,half3-curve3);

// The setup function runs once when you press reset or power the board
void setup() {
  
  Serial.begin(9600);

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  dw = display.width();
  dh = display.height();

  // Clear the buffer
  display.clearDisplay();
  uint16_t y0 = 0.4271*dh;
  display.drawLine(0,y0,dw,y0,SSD1306_WHITE);
  y0 = 0.7604*dh;
  display.drawLine(0,y0,dw,y0,SSD1306_WHITE);

  drawTurnout(-1);
  drawSlip(-1);
  writeRoute("no route");
  display.display();

  Switch1.Init();
  Switch2.Init();
  Switch3.Init(); 
}

// The loop function runs over and over again forever
void loop() {
 
  const char* routeName[] = {"route A","no route","route B","no route","no route","route C","no route","route D"};
  const int8_t drawState[] = {0,1,2,3,4,5,6,7};

  static uint16_t oldstate = -1;
  bool s1 = Switch1.readButton() == HIGH;
  bool s2 = Switch2.readButton() == HIGH;
  bool s3 = Switch3.readButton() == HIGH;
  uint8_t state = (s1 ? 0 : 1) | (s2 ? 0 : 2) | (s3 ? 0 : 4);
  if(state != oldstate) {
#ifdef DEBUG
    Serial.print("New state ");
    Serial.print(state);
    Serial.print(" s1 ");
    Serial.print(s1);
    Serial.print(" s2 ");
    Serial.print(s2);
    Serial.print(" s3 ");
    Serial.println(s3);
#endif
    display.clearDisplay();
    uint16_t y0 = 0.4271*dh;
    display.drawLine(0,y0,dw,y0,SSD1306_WHITE);
    y0 = 0.7604*dh;
    display.drawLine(0,y0,dw,y0,SSD1306_WHITE);
    drawTurnout(drawState[state]);
    drawSlip(drawState[state]);
    writeRoute(routeName[state]);
    display.display();
    oldstate = state;
  }
  Switch1.Change(s1);
  Switch2.Change(s2);
  Switch3.Change(s3);
}
