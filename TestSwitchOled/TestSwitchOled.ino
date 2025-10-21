/**************************************************************************
 This is an example for our Monochrome OLEDs based on SSD1306 drivers

 Pick one up today in the adafruit shop!
 ------> http://www.adafruit.com/category/63_98

 This example is for a 128x64 pixel display using I2C to communicate
 3 pins are required to interface (two I2C and one reset).

 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source
 hardware by purchasing products from Adafruit!

 Written by Limor Fried/Ladyada for Adafruit Industries,
 with contributions from the open source community.
 BSD license, check license.txt for more information
 All text above, and the splash screen below must be
 included in any redistribution.
 **************************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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

void drawSlip(int status) {
  uint16_t x0 = dw/2;
  uint16_t y0 = dh/2;
  uint16_t x1 = 109;
  uint16_t y1 = 53;
  display.drawLine(x0,y0,x1,y1,SSD1306_WHITE);

 double rad = 120;
 int deg = 26;
 int xc1 = 14*dw/32;
 int yc1 = 2*21+rad;
  
 drawArc(xc1,yc1,rad,270,270+deg,false);

  int xc2 = 7*dw/8;
  int yc2 = 2*21-rad;
  
 drawArc(xc2,yc2,rad,90,90+deg,false);

 display.drawLine(xc1,2*dh/3,xc2,2*dh/3,SSD1306_WHITE);
 display.display();
}

void drawTurnout(int status) {
  double rad = 120;
  double xc = 7.*dw/64;
  double yc = 21+rad;
  int deg = 26;
  display.drawLine(xc,dh/3,dw/2,dh/3,SSD1306_WHITE);

  drawArc(xc,yc,rad,270,270+deg,false);

}

void setup() {
  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  // This wi
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Clear the buffer
  display.clearDisplay();
 // display.drawLine(0,21,dw,21,SSD1306_WHITE);
 // display.drawLine(0,42,dw,42,SSD1306_WHITE);

 


 drawTurnout(0);
 drawSlip(0);
 display.display();

  for(;;);

}

void loop() {
}


