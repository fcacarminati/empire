/*

// the regular Adafruit "TouchScreen.h" library only works on AVRs

// different mcufriend shields have Touchscreen on different pins
// and rotation.
// Run the TouchScreen_Calibr_native sketch for calibration of your shield

#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;       // hard-wired for UNO shields anyway.
#include <TouchScreen.h>

char *name = "Please Calibrate.";  //edit name of shield
const int XP = 8, YP = A3, XM = A2, YM = 9;  //next common configuration
//const int TS_LEFT=907,TS_RT=136,TS_TOP=942,TS_BOT=139;
const int TS_LEFT=178,TS_RT=860,TS_TOP=947,TS_BOT=127;

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
TSPoint tp;

#define MINPRESSURE 200
#define MAXPRESSURE 1000

int16_t BOXSIZE;
int16_t PENRADIUS = 1;
uint16_t ID, oldcolor, currentcolor;
uint8_t Orientation = 0;    //PORTRAIT

// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

void show_Serial(void)
{
    Serial.println(F("Most Touch Screens use pins 6, 7, A1, A2"));
    Serial.println(F("But they can be in ANY order"));
    Serial.println(F("e.g. right to left or bottom to top"));
    Serial.println(F("or wrong direction"));
    Serial.println(F("Edit name and calibration statements\n"));
    Serial.println(name);
    Serial.print(F("ID=0x"));
    Serial.println(ID, HEX);
    Serial.println("Screen is " + String(tft.width()) + "x" + String(tft.height()));
    Serial.println("Calibration is: ");
    Serial.println("LEFT = " + String(TS_LEFT) + " RT  = " + String(TS_RT));
    Serial.println("TOP  = " + String(TS_TOP)  + " BOT = " + String(TS_BOT));
    Serial.println("Wiring is always PORTRAIT");
    Serial.println("YP=" + String(YP)  + " XM=" + String(XM));
    Serial.println("YM=" + String(YM)  + " XP=" + String(XP));
}

void show_tft(void)
{
    tft.setCursor(0, 0);
    tft.setTextSize(1);
    tft.print(F("ID=0x"));
    tft.println(ID, HEX);
    tft.println("Screen is " + String(tft.width()) + "x" + String(tft.height()));
    tft.println("");
    tft.setTextSize(2);
    tft.println(name);
    tft.setTextSize(1);
    tft.println("PORTRAIT Values:");
    tft.println("LEFT = " + String(TS_LEFT) + " RT  = " + String(TS_RT));
    tft.println("TOP  = " + String(TS_TOP)  + " BOT = " + String(TS_BOT));
    tft.println("\nWiring is: ");
    tft.println("YP=" + String(YP)  + " XM=" + String(XM));
    tft.println("YM=" + String(YM)  + " XP=" + String(XP));
    tft.setTextSize(2);
    tft.setTextColor(RED);
    tft.setCursor((tft.width() - 48) / 2, (tft.height() * 2) / 4);
    tft.print("EXIT");
    tft.setTextColor(YELLOW, BLACK);
    tft.setCursor(0, (tft.height() * 6) / 8);
    tft.print("Touch screen for loc");
    while (1) {
        tp = ts.getPoint();
        pinMode(XM, OUTPUT);
        pinMode(YP, OUTPUT);
        if (tp.z < MINPRESSURE || tp.z > MAXPRESSURE) continue;
        if (tp.x > 450 && tp.x < 570  && tp.y > 450 && tp.y < 570) break;
        tft.setCursor(0, (tft.height() * 3) / 4);
        tft.print("tp.x=" + String(tp.x) + " tp.y=" + String(tp.y) + "   ");
    }
}


void setup(void)
{
    uint16_t tmp;

    tft.reset();
    ID = tft.readID();
    tft.begin(ID);
    Serial.begin(9600);
    show_Serial();
    tft.setRotation(Orientation);
    tft.fillScreen(BLACK);
    show_tft();

    BOXSIZE = tft.width() / 6;
    tft.fillScreen(BLACK);

    tft.fillRect(0, 0, BOXSIZE, BOXSIZE, RED);
    tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, YELLOW);
    tft.fillRect(BOXSIZE * 2, 0, BOXSIZE, BOXSIZE, GREEN);
    tft.fillRect(BOXSIZE * 3, 0, BOXSIZE, BOXSIZE, CYAN);
    tft.fillRect(BOXSIZE * 4, 0, BOXSIZE, BOXSIZE, BLUE);
    tft.fillRect(BOXSIZE * 5, 0, BOXSIZE, BOXSIZE, MAGENTA);

    tft.drawRect(0, 0, BOXSIZE, BOXSIZE, WHITE);
    currentcolor = RED;
    delay(1000);
}

void loop()
{
    uint16_t xpos, ypos;  //screen coordinates
    tp = ts.getPoint();   //tp.x, tp.y are ADC values

    // if sharing pins, you'll need to fix the directions of the touchscreen pins
    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    // we have some minimum pressure we consider 'valid'
    // pressure of 0 means no pressing!

    if (tp.z > MINPRESSURE && tp.z < MAXPRESSURE) {
        // most mcufriend have touch (with icons) that extends below the TFT
        // screens without icons need to reserve a space for "erase"
        // scale the ADC values from ts.getPoint() to screen values e.g. 0-239
        //
        // Calibration is true for PORTRAIT. tp.y is always long dimension 
        // map to your current pixel orientation
        switch (Orientation) {
            case 0:
                xpos = map(tp.x, TS_LEFT, TS_RT, 0, tft.width());
                ypos = map(tp.y, TS_TOP, TS_BOT, 0, tft.height());
                break;
            case 1:
                xpos = map(tp.y, TS_TOP, TS_BOT, 0, tft.width());
                ypos = map(tp.x, TS_RT, TS_LEFT, 0, tft.height());
                break;
            case 2:
                xpos = map(tp.x, TS_RT, TS_LEFT, 0, tft.width());
                ypos = map(tp.y, TS_BOT, TS_TOP, 0, tft.height());
                break;
            case 3:
                xpos = map(tp.y, TS_BOT, TS_TOP, 0, tft.width());
                ypos = map(tp.x, TS_LEFT, TS_RT, 0, tft.height());
                break;
        }

        // are we in top color box area ?
        if (ypos < BOXSIZE) {               //draw white border on selected color box
            oldcolor = currentcolor;

            if (xpos < BOXSIZE) {
                currentcolor = RED;
                tft.drawRect(0, 0, BOXSIZE, BOXSIZE, WHITE);
            } else if (xpos < BOXSIZE * 2) {
                currentcolor = YELLOW;
                tft.drawRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, WHITE);
            } else if (xpos < BOXSIZE * 3) {
                currentcolor = GREEN;
                tft.drawRect(BOXSIZE * 2, 0, BOXSIZE, BOXSIZE, WHITE);
            } else if (xpos < BOXSIZE * 4) {
                currentcolor = CYAN;
                tft.drawRect(BOXSIZE * 3, 0, BOXSIZE, BOXSIZE, WHITE);
            } else if (xpos < BOXSIZE * 5) {
                currentcolor = BLUE;
                tft.drawRect(BOXSIZE * 4, 0, BOXSIZE, BOXSIZE, WHITE);
            } else if (xpos < BOXSIZE * 6) {
                currentcolor = MAGENTA;
                tft.drawRect(BOXSIZE * 5, 0, BOXSIZE, BOXSIZE, WHITE);
            }

            if (oldcolor != currentcolor) { //rub out the previous white border
                if (oldcolor == RED) tft.fillRect(0, 0, BOXSIZE, BOXSIZE, RED);
                if (oldcolor == YELLOW) tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, YELLOW);
                if (oldcolor == GREEN) tft.fillRect(BOXSIZE * 2, 0, BOXSIZE, BOXSIZE, GREEN);
                if (oldcolor == CYAN) tft.fillRect(BOXSIZE * 3, 0, BOXSIZE, BOXSIZE, CYAN);
                if (oldcolor == BLUE) tft.fillRect(BOXSIZE * 4, 0, BOXSIZE, BOXSIZE, BLUE);
                if (oldcolor == MAGENTA) tft.fillRect(BOXSIZE * 5, 0, BOXSIZE, BOXSIZE, MAGENTA);
            }
        }
        // are we in drawing area ?
        if (((ypos - PENRADIUS) > BOXSIZE) && ((ypos + PENRADIUS) < tft.height())) {
            tft.fillCircle(xpos, ypos, PENRADIUS, currentcolor);
        }
        // are we in erase area ?
        // Plain Touch panels use bottom 10 pixels e.g. > h - 10
        // Touch panels with icon area e.g. > h - 0
        if (ypos > tft.height() - 10) {
            // press the bottom of the screen to erase
            tft.fillRect(0, BOXSIZE, tft.width(), tft.height() - BOXSIZE, BLACK);
        }
    }
}

*/

/*
* Driver program for the the big switcyard
* Simulated with an ILI9341, will switch to ILI9486
*/

#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define BLACK       0x0000  ///<   0,   0,   0
#define NAVY        0x000F  ///<   0,   0, 123
#define DARKGREEN   0x03E0  ///<   0, 125,   0
#define DARKCYAN    0x03EF  ///<   0, 125, 123
#define MAROON      0x7800  ///< 123,   0,   0
#define PURPLE      0x780F  ///< 123,   0, 123
#define OLIVE       0x7BE0  ///< 123, 125,   0
#define LIGHTGREY   0xC618  ///< 198, 195, 198
#define DARKGREY    0x7BEF  ///< 123, 125, 123
#define BLUE        0x001F  ///<   0,   0, 255
#define GREEN       0x07E0  ///<   0, 255,   0
#define CYAN        0x07FF  ///<   0, 255, 255
#define RED         0xF800  ///< 255,   0,   0
#define MAGENTA     0xF81F  ///< 255,   0, 255
#define YELLOW      0xFFE0  ///< 255, 255,   0
#define WHITE       0xFFFF  ///< 255, 255, 255
#define ORANGE      0xFD20  ///< 255, 165,   0
#define GREENYELLOW 0xAFE5  ///< 173, 255,  41
#define PINK        0xFC18  ///< 255, 130, 198

// For the Adafruit shield, these are the default.
#define TFT_CLK 13
#define TFT_MISO 12
#define TFT_MOSI 11
#define TFT_DC 9
#define TFT_CS 10
#define TFT_RST 8
// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

const double degrad = acos(-1.)/180.;
double scale = 0;
int dw = 0;
int dh = 0;

enum kside {kright,kleft};

//================================= track ===================================
class track {
public:
  track(kside side=kleft):
    m_side(side)
  {};

  static void setScale(double scale) {
    m_scale=scale;
    m_npbed=3*scale+0.5;
    m_npbed += 1-m_npbed%2;
    m_nptrk=1.65*scale+0.5;
    m_nptrk += 1-m_nptrk%2;
  }

protected:  
  inline uint16_t rgb888_to_rgb565_round(uint8_t r, uint8_t g, uint8_t b) {
    uint16_t R5 = (r * 31 + 127) / 255;  // ≈ round(r/255*31)
    uint16_t G6 = (g * 63 + 127) / 255;  // ≈ round(g/255*63)
    uint16_t B5 = (b * 31 + 127) / 255;
    return (R5 << 11) | (G6 << 5) | B5;
  }
  
  inline const void matrot(const double x0, const double y0, uint16_t &x1, uint16_t &y1) {
      x1 = x0*m_rotmat[0]+y0*m_rotmat[1]+m_rotmat[2]+0.5;
      y1 = x0*m_rotmat[3]+y0*m_rotmat[4]+m_rotmat[5]+0.5;
  }

  inline const void matset(double angle, double x, double y) {
    m_rotmat[0] = cos(angle);
    m_rotmat[1] = -sin(angle);
    m_rotmat[2] = x;
    m_rotmat[3] = -m_rotmat[1];
    m_rotmat[4] = m_rotmat[0];
    m_rotmat[5] = y;
  }

  void drawArc(double rad, double ddeg0, double ddeg1, int16_t color, int16_t yshift=0) {
      
    double deg0 = ddeg0<ddeg1 ? ddeg0 : ddeg1;
    double deg1 = ddeg0<ddeg1 ? ddeg1 : ddeg0;
    /*
    * > 2 pix per step
    */
    int16_t npix = rad*degrad+0.5;
    double hnorm = 0.5/npix;
    uint16_t nstep = 2*npix*(deg1-deg0)+0.5;
    int8_t rsign = m_side == kright ? 1 : -1;
    int8_t beg = -m_nptrk/2;
    int8_t end = -beg;
    uint16_t xp = 0;
    uint16_t yp = 0;
    for(int8_t j=beg; j<=end; ++j) {
      for(uint16_t i=0; i<= nstep; ++i) {
        double ang = (deg0+hnorm*i)*degrad;
        double xp0 = (rad+j) * cos(ang);
        double yp0 = (rad+j) * sin(ang) + rsign*(rad+yshift);
        matrot(xp0,yp0,xp,yp);
        tft.drawPixel(xp,yp,color);
      }
    }
  }

  static uint8_t m_npbed; // number of pixels for track bed
  static uint8_t m_nptrk; // number of pixels for track gauge
  kside m_side;    // side of the turnout (right, left)
  double m_rotmat[6];

  static double m_scale; // scale for drawing
};
//================================= track ===================================

uint8_t track::m_npbed=0;
uint8_t track::m_nptrk=0;
double  track::m_scale = 0;

//================================= slip ====================================
class slip: public track {
public:
   slip(double len, uint16_t ang, double rad): 
   track(),
   m_len(len),
   m_ang(ang),
   m_rad(rad)
   {};

  void draw(uint16_t x, uint16_t y, double rot) {
/*
* x      -- position of the start of the switch
* y      -- position of the start of the switch
* rot    -- rotation angle
*/
    double angle = rot*degrad;
    matset(angle,x,y);
  
    uint16_t irad = m_rad*m_scale+0.5;
    m_side = kleft;
    int16_t mshift = 0.33*m_npbed + 0.5;
    drawArc(irad,90-0.5*m_ang,90+0.5*m_ang,WHITE, mshift);
    m_side = kright;
    drawArc(irad,270-0.5*m_ang,270+0.5*m_ang,WHITE, mshift);
  
    angle = (rot+0.5*m_ang)*degrad;
    matset(angle,x,y);
    int8_t beg = -m_nptrk/2;
    int8_t end =  -beg;
    uint16_t x0 = 0;
    uint16_t y0 = 0;
    uint16_t x1 = 0;
    uint16_t y1 = 0;
    int16_t len = 0.5*(m_rad*m_scale)*sin(m_ang*degrad)+0.5;
    for(int8_t j=beg; j<=end; ++j) {
      matrot(-len,j,x0,y0);
      matrot( len,j,x1,y1);
      tft.drawLine(x0,y0,x1,y1,CYAN);  
    }

    angle = (rot-0.5*m_ang)*degrad;
    matset(angle,x,y);
    for(int8_t j=beg; j<=end; ++j) {
      matrot(-len,j,x0,y0);
      matrot( len,j,x1,y1);
      tft.drawLine(x0,y0,x1,y1,CYAN);  
    }
  }

private:
  double m_len;    // length of the straight part of the switch
  uint16_t m_ang;  // angle of the curved part of the switch
  double m_rad;    // radius of the curved part of the switch
};
//================================= slip ===================================

//================================= turnout ================================
class turnout: public track {
public:
  turnout(double len, uint16_t ang, double rad, kside side):
    track(side),
    m_len(len),
    m_ang(ang),
    m_rad(rad)
    {};

  void draw(uint16_t x, uint16_t y, double rot) {
/*
* x      -- position of the start of the switch
* y      -- position of the start of the switch
* rot    -- rotation angle
*/
    double angle = rot*degrad;
    matset(angle,x,y);
    uint16_t irad = m_rad*m_scale+0.5;
    if(m_side == kleft) {
      drawArc(irad,90-m_ang,90,GREEN);
    } else {
      drawArc(irad,270,270+m_ang,GREEN);
    }
  
    int8_t beg = -m_nptrk/2;
    int8_t end = -beg;
    uint16_t len = m_len*m_scale+0.5;
    uint16_t x0 = 0;
    uint16_t y0 = 0;
    uint16_t x1 = 0;
    uint16_t y1 = 0;
    for(int8_t j=beg; j<=end; ++j) {
      matrot(0,j,x0,y0);
      matrot(len,j,x1,y1);
      tft.drawLine(x0,y0,x1,y1,WHITE);  
    }
  }

private:
  double m_len;    // length of the straight part of the switch
  uint16_t m_ang;  // angle of the curved part of the switch
  double m_rad;    // radius of the curved part of the switch
};
//================================= turnout ================================

//================================= straight ===============================
class straight: public track {
public:
  straight(double len):
    track(),
    m_len(len)
    {};

  void draw(uint16_t x, uint16_t y, double rot) {
/*
* x      -- position of the start of the switch
* y      -- position of the start of the switch
* rot    -- rotation angle
*/
    double angle = rot*degrad;
    angle = rot*degrad;
    matset(angle,x,y);

    uint16_t x0 = 0;
    uint16_t y0 = 0;
    uint16_t x1 = 0;
    uint16_t y1 = 0;

    int16_t len = m_len*m_scale+0.5;
    int8_t beg = -m_nptrk/2;
    int8_t end = -beg;
    for(int8_t j=beg; j<=end; ++j) {
      matrot(-0.5*len,j,x0,y0);
      matrot( 0.5*len,j,x1,y1);
      tft.drawLine(x0,y0,x1,y1,CYAN);  
    }
  }

private:
  double m_len;    // length of the straight part of the switch
};
//================================= straight ===============================


turnout turn1(23,15,87.35,kright);
turnout turn2(23,15,87.35,kleft);
turnout turn3(23,15,87.35,kright);
turnout turn4(23,15,87.35,kleft);
slip slip1(23,15,105);
slip slip2(23,15,105);
slip slip3(23,15,105);
slip slip4(23,15,105);

// 6 Aand 8.6
void setup() {
  Serial.begin(9600);
  Serial.println("ILI9341 Test!"); 
 
  tft.begin();
  tft.setRotation(1);
  //tft.fillScreen(WHITE);
  yield();

  // read diagnostics (optional but can help debug problems)
  uint8_t x = tft.readcommand8(ILI9341_RDMODE);
  Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDMADCTL);
  Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDPIXFMT);
  Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDIMGFMT);
  Serial.print("Image Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDSELFDIAG);
  Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX); 
  Serial.print("TFT width:  "); Serial.println(dw=tft.width());
  Serial.print("TFT height: "); Serial.println(dh=tft.height());

/*
* Define scale as pixels per cm
*/  

  scale = dw/102.;
  track::setScale(scale);
  double dist1 = 6;
  double dist2 = 8.6;
  uint16_t heig1 = dh/2-(dist2/2+dist1)*scale+0.5;
  uint16_t heig2 = heig1+dist1*scale+0.5;
  uint16_t heig3 = heig2+dist2*scale+0.5;
  uint16_t heig4 = heig3+dist1*scale+0.5;

  /*
  tft.drawLine(0,heig1,dw,heig1,CYAN);
  tft.drawLine(0,heig2,dw,heig2,CYAN);
  tft.drawLine(0,heig3,dw,heig3,CYAN);
  tft.drawLine(0,heig4,dw,heig4,CYAN);
  */

  straight str1(5.);
  str1.draw(dw/2,dh/2,15);
  str1.draw(dw/2,dh/2,-15);

  straight str2(4.);
  str2.draw(dw/2,heig2,0);
  str2.draw(dw/2,heig3,0);

  straight str3(56.);
  str3.draw(dw/2,heig1,0);
  str3.draw(dw/2,heig4,0);

  double len = 22;
  straight str4(len);
  str4.draw(scale*len/2,heig2,0);
  str4.draw(dw-scale*len/2,heig2,0);
  str4.draw(scale*len/2,heig3,0);
  str4.draw(dw-scale*len/2.,heig3,0);

  turn1.draw(0,heig1,0.);
  turn2.draw(dw,heig1,180.);
  turn3.draw(dw,heig4,180.);
  turn4.draw(0,heig4,0.);

  //Double slip (L= 230mm 15° R=1050mm)

  int16_t xshift = 35.2*scale+0.5;
  slip1.draw(xshift,heig2,7.5);
  slip2.draw(xshift,heig3,-7.5);
  slip3.draw(dw-xshift,heig2,-7.5);
  slip4.draw(dw-xshift,heig3,7.5);

  yield();
  
//#define TEST
#ifdef TEST
  Serial.println(F("Benchmark                Time (microseconds)"));
  delay(10);
  Serial.print(F("Screen fill              "));
  Serial.println(testFillScreen());
  delay(500);

  Serial.print(F("Text                     "));
  Serial.println(testText());
  delay(3000);

  Serial.print(F("Lines                    "));
  Serial.println(testLines(CYAN));
  delay(500);

  Serial.print(F("Horiz/Vert Lines         "));
  Serial.println(testFastLines(RED, BLUE));
  delay(500);

  Serial.print(F("Rectangles (outline)     "));
  Serial.println(testRects(GREEN));
  delay(500);

  Serial.print(F("Rectangles (filled)      "));
  Serial.println(testFilledRects(YELLOW, MAGENTA));
  delay(500);

  Serial.print(F("Circles (filled)         "));
  Serial.println(testFilledCircles(10, MAGENTA));

  Serial.print(F("Circles (outline)        "));
  Serial.println(testCircles(10, WHITE));
  delay(500);

  Serial.print(F("Triangles (outline)      "));
  Serial.println(testTriangles());
  delay(500);

  Serial.print(F("Triangles (filled)       "));
  Serial.println(testFilledTriangles());
  delay(500);

  Serial.print(F("Rounded rects (outline)  "));
  Serial.println(testRoundRects());
  delay(500);

  Serial.print(F("Rounded rects (filled)   "));
  Serial.println(testFilledRoundRects());
  delay(500);

  Serial.println(F("Done!"));
#endif

}


void loop(void) {
  return;
  tft.fillScreen(NAVY);
  tft.setCursor(0, 0);
  tft.setTextColor(WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(YELLOW); tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(RED);    tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");
  delay(5000);

}


unsigned long testFillScreen() {
  unsigned long start = micros();
  tft.fillScreen(BLACK);
  yield();
  tft.fillScreen(RED);
  yield();
  tft.fillScreen(GREEN);
  yield();
  tft.fillScreen(BLUE);
  yield();
  tft.fillScreen(BLACK);
  yield();
  return micros() - start;
}

unsigned long testText() {
  tft.fillScreen(BLACK);
  unsigned long start = micros();
  tft.setCursor(0, 0);
  tft.setTextColor(WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(YELLOW); tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(RED);    tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");
  return micros() - start;
}

unsigned long testLines(uint16_t color) {
  unsigned long start, t;
  int           x1, y1, x2, y2,
                w = tft.width(),
                h = tft.height();

  tft.fillScreen(BLACK);
  yield();
  
  x1 = y1 = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t     = micros() - start; // fillScreen doesn't count against timing

  yield();
  tft.fillScreen(BLACK);
  yield();

  x1    = w - 1;
  y1    = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  yield();
  tft.fillScreen(BLACK);
  yield();

  x1    = 0;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  yield();
  tft.fillScreen(BLACK);
  yield();

  x1    = w - 1;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);

  yield();
  return micros() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2) {
  unsigned long start;
  int           x, y, w = tft.width(), h = tft.height();

  tft.fillScreen(BLACK);
  start = micros();
  for(y=0; y<h; y+=5) tft.drawFastHLine(0, y, w, color1);
  for(x=0; x<w; x+=5) tft.drawFastVLine(x, 0, h, color2);

  return micros() - start;
}

unsigned long testRects(uint16_t color) {
  unsigned long start;
  int           n, i, i2,
                cx = tft.width()  / 2,
                cy = tft.height() / 2;

  tft.fillScreen(BLACK);
  n     = min(tft.width(), tft.height());
  start = micros();
  for(i=2; i<n; i+=6) {
    i2 = i / 2;
    tft.drawRect(cx-i2, cy-i2, i, i, color);
  }

  return micros() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
  unsigned long start, t = 0;
  int           n, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(BLACK);
  n = min(tft.width(), tft.height());
  for(i=n; i>0; i-=6) {
    i2    = i / 2;
    start = micros();
    tft.fillRect(cx-i2, cy-i2, i, i, color1);
    t    += micros() - start;
    // Outlines are not included in timing results
    tft.drawRect(cx-i2, cy-i2, i, i, color2);
    yield();
  }

  return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;

  tft.fillScreen(BLACK);
  start = micros();
  for(x=radius; x<w; x+=r2) {
    for(y=radius; y<h; y+=r2) {
      tft.fillCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int           x, y, r2 = radius * 2,
                w = tft.width()  + radius,
                h = tft.height() + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = micros();
  for(x=0; x<w; x+=r2) {
    for(y=0; y<h; y+=r2) {
      tft.drawCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testTriangles() {
  unsigned long start;
  int           n, i, cx = tft.width()  / 2 - 1,
                      cy = tft.height() / 2 - 1;

  tft.fillScreen(BLACK);
  n     = min(cx, cy);
  start = micros();
  for(i=0; i<n; i+=5) {
    tft.drawTriangle(
      cx    , cy - i, // peak
      cx - i, cy + i, // bottom left
      cx + i, cy + i, // bottom right
      tft.color565(i, i, i));
  }

  return micros() - start;
}

unsigned long testFilledTriangles() {
  unsigned long start, t = 0;
  int           i, cx = tft.width()  / 2 - 1,
                   cy = tft.height() / 2 - 1;

  tft.fillScreen(BLACK);
  start = micros();
  for(i=min(cx,cy); i>10; i-=5) {
    start = micros();
    tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(0, i*10, i*10));
    t += micros() - start;
    tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(i*10, i*10, 0));
    yield();
  }

  return t;
}

unsigned long testRoundRects() {
  unsigned long start;
  int           w, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(BLACK);
  w     = min(tft.width(), tft.height());
  start = micros();
  for(i=0; i<w; i+=6) {
    i2 = i / 2;
    tft.drawRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(i, 0, 0));
  }

  return micros() - start;
}

unsigned long testFilledRoundRects() {
  unsigned long start;
  int           i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(BLACK);
  start = micros();
  for(i=min(tft.width(), tft.height()); i>20; i-=6) {
    i2 = i / 2;
    tft.fillRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(0, i, 0));
    yield();
  }

  return micros() - start;
}