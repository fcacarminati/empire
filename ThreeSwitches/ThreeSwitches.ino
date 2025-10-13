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

uint8_t dw = display.width();
uint8_t dh = display.height();
const double degrad = acos(-1.)/180;
const uint16_t yoff = 6;
 
void drawArc(int xc, int yc, int rad, int deg0, int deg1, bool bold) {
  int nstep = 4*(deg1-deg0);
  int beg = 0;
  int end = 1;
  if(bold) {
    beg = -2;
    end = 2;
  }
  for(int j=beg; j<end; ++j) {
    for(int i=0; i<= nstep; ++i) {
      double ang = (deg0 + 0.25*i)*degrad;
      uint8_t xp = xc + (rad+j) * cos(ang);
      uint8_t yp = yc + (rad+j) * sin(ang);
      display.drawPixel(xp,yp,SSD1306_WHITE);
    }
  }
}

void drawSlip(int state) {
  uint16_t x0 = dw/2;
  uint16_t y0 = dh/2+yoff;
  uint16_t x1 = 109;
  uint16_t y1 = 53+yoff;

  int8_t beg = 0;
  int8_t end = 1;

  /*
  * Diagonal line
  */
 
  if(state == 1) {
    beg = -3;
    end = 3;
  }
  for(int8_t i=beg; i<end; ++i){
    display.drawLine(x0,y0+i,x1,y1+i,SSD1306_WHITE);
  }

  double rad = 120;
  int deg = 26;
  int xc1 = 14*dw/32;
  int yc1 = 2*21+rad+yoff;
  
  bool arcbold = false;
  if(state == 6 || state == 7) arcbold = true;
  drawArc(xc1,yc1,rad,270,270+deg,arcbold);

  int xc2 = 7*dw/8;
  int yc2 = 2*21-rad+yoff;
  
  arcbold = false;
  if(state == 7) arcbold = true;
  drawArc(xc2,yc2,rad,90,90+deg,arcbold);

  beg = 0;
  end = 1;
  if (state == 0 || state == 1) {
    beg = -3;
    end = +3;
  }
  for(int8_t i=beg; i<end; ++i) {
    display.drawLine(xc1,2*dh/3+yoff+i,xc2,2*dh/3+yoff+i,SSD1306_WHITE);
  }
}

void drawTurnout(int status) {
  double rad = 120;
  double xc = 7.*dw/64;
  double yc = 21+rad+yoff;
  int deg = 26;

  uint8_t beg = -3;
  uint8_t end = 3;
  bool arcbold = false;
  if( status == -1) {
    beg = 0;
    end = 1;
  } else if( 1 & status) {
    beg = 0;
    end = 1;
    arcbold = true;
  }
  for(int8_t i = beg; i<end; ++i) {
    display.drawLine(xc,dh/3+yoff+i,dw/2,dh/3+yoff+i,SSD1306_WHITE);
  }
  drawArc(xc,yc,rad,270,270+deg,arcbold);
}

void writeRoute(const char* mess){
  display.setTextSize(2);
  display.setTextColor(WHITE);
  uint8_t xpos = 0.5*dw*(1.-0.1*strlen(mess))+1.5;
  display.setCursor(xpos,1);
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
  Switch1.Init();
  Switch2.Init();
  Switch3.Init();
  delay(1000);
  Serial.begin(9600);

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Clear the buffer
  display.clearDisplay();
  drawTurnout(-1);
  drawSlip(-1);
  writeRoute("no route");
  display.display();
}

// The loop function runs over and over again forever
void loop() {
 
  const char* route[] = {"route A","route B","no route","no route","no route","no route","route C","route D"};

  static uint8_t oldstate = -1;
  bool s1 = Switch1.readButton() == HIGH;
  bool s2 = Switch2.readButton() == HIGH;
  bool s3 = Switch3.readButton() == HIGH;
/*
  Serial.print(" s1 ");
  Serial.print(s1);
  Serial.print(" s2 ");
  Serial.print(s2);
  Serial.print(" s3 ");
  Serial.println(s3);
  delay(100);
*/
  uint8_t state = (s3 ? 0 : 4) + (s2 ? 0 : 2) + (s1 ? 0 : 1);
  if(state != oldstate) {
#ifdef DEBUG
    Serial.print("New state ");
    Serial.println(state);
    Serial.print(" s1 ");
    Serial.print(s1);
    Serial.print(" s2 ");
    Serial.print(s2);
    Serial.print(" s3 ");
    Serial.println(s3);
#endif
    display.clearDisplay();
    if(state == 0 || state == 1 || state == 6 || state == 7){
      drawTurnout(state);
      drawSlip(state);
    } else {
      drawTurnout(-1);
      drawSlip(-1);
    }
    writeRoute(route[state]);
    display.display();
    oldstate = state;
  }
  Switch1.Change(s1);
  Switch2.Change(s2);
  Switch3.Change(s3);

}
