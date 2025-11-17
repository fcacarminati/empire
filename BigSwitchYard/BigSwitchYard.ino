/*
* Driver program for the the big switcyard
* Simulated with an ILI9341, will switch to ILI9486
*/

#include <math.h>
#include <string.h>

//#define TFT25257
#define ILI9341

#include "Adafruit_GFX.h"
#ifdef ILI9341
#include "SPI.h"
#include "Adafruit_ILI9341.h"
#include <Adafruit_FT6206.h>
#endif
#ifdef TFT25257
#include "MCUFRIEND_kbv.h"
#include "TouchScreen.h"
const int XP = 8, YP = A3, XM = A2, YM = 9;
//const int TS_LEFT=175,TS_RT=869,TS_TOP=940,TS_BOT=133;
const int TS_LEFT=127,TS_RT=909,TS_TOP=951,TS_BOT=98;
#define MINPRESSURE 200
#define MAXPRESSURE 1000
#endif

const uint8_t Orientation = 1;    //Landscape

#include "Switch.h"

static float lut[91];
const float degrad = acosf(-1.f)/180.f;
const float raddeg = 1.f/degrad;

void preplut() {
  for(uint16_t i=0; i<91; ++i) {
    float ang = i*degrad;
    lut[i] = sinf(ang);
  }
}

float lutsin(float ang) {
  ang = fmodf(ang,360.f);
  ang = ang < 0 ? ang+360.f : ang;
  int8_t isign = 1;
  if(ang > 270.f) {
    ang = 360.f-ang;
    isign =-1;
  } else if(ang > 180.f) {
    ang = ang - 180.f;
    isign = -1;
  } else if(ang > 90.f) {
    ang = 180.f-ang;
  }
  if(ang >= 90.f) return isign;
  else if(ang <=0.f) return 0.f;
  int iang = ang;
  float frac = ang - iang;
  float lin = (1.f-frac)*lut[iang]+frac*lut[iang+1];
  if(iang==0) return isign*lin;
  float curv = lut[iang-1] - 2.f*lut[iang] + lut[iang+1];
  return isign*(lin - 0.5f*frac*(1.f-frac)*curv);
}

float lutcos(float ang) {
  return lutsin(90.f-ang);
}

inline uint16_t rgb888_to_rgb565_round(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t R5 = (r * 31 + 127) / 255;  // ≈ round(r/255*31)
  uint16_t G6 = (g * 63 + 127) / 255;  // ≈ round(g/255*63)
  uint16_t B5 = (b * 31 + 127) / 255;
  return (R5 << 11) | (G6 << 5) | B5;
}

inline void rgb565_to_rgb888(uint16_t c,
                                    uint8_t *r, uint8_t *g, uint8_t *b)
{
  uint8_t R5 = (c >> 11) & 0x1F;   // 5 bits
  uint8_t G6 = (c >> 5)  & 0x3F;   // 6 bits
  uint8_t B5 =  c        & 0x1F;   // 5 bits
// Bit replication to fill 8 bits (keeps brightness scale reasonable)
  *r = (R5 << 3) | (R5 >> 2);      // 5 -> 8
  *g = (G6 << 2) | (G6 >> 4);      // 6 -> 8
  *b = (B5 << 3) | (B5 >> 2);      // 5 -> 8
}


#define BLACK       0x0000  ///<   0,   0,   0
#define NAVY        0x000F  ///<   0,   0, 123
#define DARKGREEN   0x03E0  ///<   0, 125,   0
#define DARKCYAN    0x03EF  ///<   0, 125, 123
#define MAROON      0x7800  ///< 123,   0,   0
#define PURPLE      0x780F  ///< 123,   0, 123
#define OLIVE       0x7BE0  ///< 123, 125,   0
#define LIGHTGREY   0xC618  ///< 198, 195, 198
#define MEDIUMGREY  0x94D2  ///< 150, 152, 150
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
#define SPINK       0xFC18  ///< 255, 130, 198

uint16_t dw = 0;
uint16_t dh = 0;

#ifdef ILI9341
// For the Adafruit shield, these are the default.
//#define TFT_CLK 13
#define TFT_CLK 52
//#define TFT_MISO 12
#define TFT_MISO 50
//#define TFT_MOSI 11
#define TFT_MOSI 51
#define TFT_DC 9
#define TFT_CS 10
#define TFT_RST 8
// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

//Creating an object called "ts". Because the Capacitive Touch is I2C, nothing is defined here, but 
//SDA and SCL must be properly connected or it will not work at all. 
Adafruit_FT6206 ts = Adafruit_FT6206();
TS_Point p;
#endif

#ifdef TFT25257
MCUFRIEND_kbv tft;
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
TSPoint p;
static inline void mapP(uint16_t &xpos, uint16_t &ypos) {
  switch (Orientation) {
  case 0: //portrait
    xpos = map(p.x, TS_LEFT, TS_RT, 0, tft.width());
    ypos = map(p.y, TS_TOP, TS_BOT, 0, tft.height());
    break;
  case 1:
    xpos = map(p.y, TS_TOP, TS_BOT, 0, tft.width());
    ypos = map(p.x, TS_RT, TS_LEFT, 0, tft.height());
    break;
  case 2:
    xpos = map(p.x, TS_RT, TS_LEFT, 0, tft.width());
    ypos = map(p.y, TS_BOT, TS_TOP, 0, tft.height());
    break;
  case 3:
    xpos = map(p.y, TS_BOT, TS_TOP, 0, tft.width());
    ypos = map(p.x, TS_LEFT, TS_RT, 0, tft.height());
    break;
  };
  xpos = constrain(xpos, 0, dw-1);
  ypos = constrain(ypos, 0, dh-1);
}
#endif
#ifdef ILI9341
static inline void mapP(uint16_t &xpos, uint16_t &ypos) {
  // FT6206 returns ~0..239/0..319 in its own axes.
  // Adjust for your display Orientation = 0..3
  switch (Orientation) {
    case 0: // portrait
      xpos = p.x;
      ypos = p.y;
      break;
    case 1: // landscape (your default)
      xpos = p.y;
      ypos = dh - 1 - p.x;
      break;
    case 2: // portrait flipped
      xpos = dw - 1 - p.x;
      ypos = dh - 1 - p.y;
      break;
    case 3: // landscape flipped
      xpos = dw - 1 - p.y;
      ypos = p.x;
      break;
  }
  xpos = constrain(xpos, 0, dw - 1);
  ypos = constrain(ypos, 0, dh - 1);
}
#endif


float scale = 0;
uint16_t xlowt=0;
uint16_t ylowt=0;
uint16_t xhigt=0;
uint16_t yhigt=0;
 

enum kside:uint8_t {kright,kleft};

//================================= track ===================================
class track {
public:
  track(uint16_t coloff,uint16_t colon,kside side=kleft):
    m_coloff(coloff),
    m_colon(colon),
    m_drfirst(true), 
    m_side(side),
    m_xpos(0),
    m_ypos(0),
    m_rotation(0)
  { 
    memset(m_win, 0, sizeof(m_win));
  }

  track(kside side=kleft):
    m_coloff(LIGHTGREY),
    m_colon(GREEN),
    m_drfirst(true), 
    m_side(side),
    m_xpos(0),
    m_ypos(0),
    m_rotation(0)
  { 
    memset(m_win, 0, sizeof(m_win));
  }

  static void setScale(float scale) {
    m_scale=scale;
    m_npbed=3*scale+0.5f;
    m_npbed += 1-m_npbed%2;
    m_nptrk=1.65*scale+0.5f;
    m_nptrk += 1-m_nptrk%2;
  }

  void setPosition(float ang, float x, float y) {
    m_xpos = x+0.5;
    m_ypos = y+0.5;
    m_rotation = 100*fmodf(ang,360);
    matset(ang, m_xpos, m_ypos);
  }

  const bool inPoint(uint16_t x, uint16_t y) {
    uint16_t z = upack(0);
    if(x<z) return false;
    z += upack(2);
    if(x>z) return false;
    z = upack(1);
    if(y<z) return false;
    z += upack(3);
    if(y>z) return false;
    return true;
  }

  virtual void draw(uint8_t ) = 0;

  virtual void setState(uint8_t , uint8_t ) {};

  virtual void setState(uint8_t ) {};

  protected:    
  void matrot(const float x0, const float y0, uint16_t &x1, uint16_t &y1) const {
      x1 = x0*m_rotmat[0]+y0*m_rotmat[1]+m_rotmat[2]+0.5f;
      y1 = x0*m_rotmat[3]+y0*m_rotmat[4]+m_rotmat[5]+0.5f;
  }

  void matset(float angle, uint16_t x, uint16_t y) {
    /*
    *  LUT tables are in degrees, so we use degrees
    */
    //float angle = angle * degrad;
    m_rotmat[0] = lutcos(angle);
    m_rotmat[1] = -lutsin(angle);
    m_rotmat[2] = x;
    m_rotmat[3] = -m_rotmat[1];
    m_rotmat[4] = m_rotmat[0];
    m_rotmat[5] = y;
  }

  inline void pack(uint16_t arg, uint8_t index) {
  const uint8_t bit = index * 10;
  const uint8_t byte = bit>>3;
  const uint8_t off = bit&7;
  const uint8_t mask = uint8_t(0XFF) >> off;
  arg &= uint16_t(0X03FF);
  m_win[byte]   = (m_win[byte]&~mask) | (arg >> (2+off));
  m_win[byte+1] = (m_win[byte+1]&(mask>>2)) | (arg << (6-off));
}

inline uint16_t upack(uint8_t index) {
  const uint8_t bit = index * 10;
  const uint8_t byte = bit>>3;
  const uint8_t off = bit&7;
  const uint8_t mask = uint8_t(0XFF) >> off;
  uint16_t arg=0;
  arg = (m_win[byte] & mask) << (off+2);
  arg |= m_win[byte+1] >> (6-off);
  return arg;
}

  void drawInit() {
    xlowt=dw;
    ylowt=dh;
    xhigt=0;
    yhigt=0;
    pack(0,0);
    pack(0,1);
    pack(dw,2);
    pack(dh,3);
  }

void drawEnd() {
  pack(xlowt,0);
  pack(ylowt,1);
  pack(xhigt-xlowt,2);
  pack(yhigt-ylowt,3);
  m_drfirst = false;
//  tft.drawRect(xlowt,ylowt,xhigt-xlowt,yhigt-ylowt,RED);
//  tft.drawRect(xlowt-1,ylowt-1,xhigt-xlowt+2,yhigt-ylowt+2,RED);
}

  void updWin (uint16_t x0, uint16_t y0) {
    xlowt = x0 < xlowt ? x0 : xlowt;
    ylowt = y0 < ylowt ? y0 : ylowt;
    xhigt = x0 > xhigt ? x0 : xhigt;
    yhigt = y0 > yhigt ? y0 : yhigt;
  }

  void drawArc(float rad, float ddeg0, float ddeg1, int16_t color, int16_t yshift=0) {
      
    const float deg0 = ddeg0<ddeg1 ? ddeg0 : ddeg1;
    const float deg1 = ddeg0<ddeg1 ? ddeg1 : ddeg0;
    /*
    * > 2 pix per step
    */
   //pixels per degree
    uint8_t npix = (rad+yshift)*degrad+0.5f;
    npix = npix > 0 ? npix : 1;
    const float hnorm = 1/(1.2f*npix);
    // 1.2 pixels per degree
    const uint16_t nstep = 1.2f*npix*(deg1-deg0)+0.5f;
    const int8_t end = m_nptrk/2;
    uint16_t x0 = 0;
    uint16_t y0 = 0;
     for(int8_t j=-end; j<=end; ++j) {
       for(uint16_t i=0; i<= nstep; ++i) {
        const float ang = (deg0+hnorm*i);
        const float xp0 = (rad+j) * lutcos(ang);
        const float yp0 = (rad+j) * lutsin(ang) + (m_side == kright ? 1 : -1)*(rad+yshift);
        matrot(xp0,yp0,x0,y0);
        if(m_drfirst) updWin(x0,y0);
        tft.writePixel(x0,y0,end-abs(j) == 1 ? MEDIUMGREY : color);
      }
    }
  }

  void drawLine(int16_t xx0, int16_t xx1, uint16_t color) {
    const int8_t end = m_nptrk/2;

    uint16_t x0 = 0;
    uint16_t y0 = 0;
    uint16_t x1 = 0;
    uint16_t y1 = 0;

    for(int8_t j=-end; j<=end; ++j) {
      matrot(xx0,j,x0,y0);
      matrot(xx1,j,x1,y1);
      if(m_drfirst) {
        updWin(x0,y0);
        updWin(x1,y1);
      }
      tft.writeLine(x0,y0,x1,y1,end-abs(j) == 1 ? MEDIUMGREY : color);  
   }
  }

  static uint8_t m_npbed; // number of pixels for track bed
  static uint8_t m_nptrk; // number of pixels for track gauge
  uint16_t m_coloff;      // color for inactive branch
  uint16_t m_colon;       // color cor active branch
  uint8_t  m_win[5];      // packed window limits
  bool  m_drfirst;        // first pass throug "draw"
  kside m_side;           // side of the turnout (right, left)
  uint16_t  m_xpos;       // x position for the track
  uint16_t  m_ypos;       // y position for the track
  int16_t  m_rotation;    // rotation for the track in 100th of degree
  float m_rotmat[6];      // rotation matrix

  static float m_scale; // scale for drawing
};
//================================= track ===================================

uint8_t track::m_npbed=0;
uint8_t track::m_nptrk=0;
float   track::m_scale=0.f;

//================================= slip ====================================
class slip: public track {
public:
   slip(float len, uint8_t ang, float rad): 
   track(),
   m_len(len*100+0.5f),
   m_ang(ang),
   m_rad(rad*100+0.5f),
   m_switch1(nullptr),
   m_switch2(nullptr)
   {};

  void draw(uint8_t state) {
/*
* x      -- position of the start of the switch
* y      -- position of the start of the switch
* rot    -- rotation angle
*/
//    static uint16_t xlow=0,ylow=0,xlen=dw,ylen=dh;
    const uint16_t len = 0.5f*(0.01f*m_rad*m_scale)*lutsin(m_ang)+1.5;
    const uint16_t irad = 0.01f*m_rad*m_scale+0.5f;
    const uint16_t mshift = 0.33*m_npbed + 0.5f;
    float angle = 0;
   
    int8_t jorder = -2-state;

    if(m_drfirst) drawInit();
    setWin(upack(0),upack(1),upack(2),upack(3));     

    tft.startWrite();
    while(1) {
      if(jorder > 0) {
        matset(0.01f*m_rotation, m_xpos, m_ypos);
        m_side = kleft;
        drawArc(irad,90-0.5f*m_ang,90+0.5f*m_ang,jorder == 4 ? m_colon : m_coloff, mshift);
        if(jorder == 4) break;
      }
      ++jorder;

      if(jorder > 0) {
        matset(0.01f*m_rotation, m_xpos, m_ypos);
        m_side = kright;
        drawArc(irad,270-0.5f*m_ang,270+0.5f*m_ang,jorder == 4 ? m_colon : m_coloff, mshift);
        if(jorder == 4) break;
      }
      ++jorder;

      if(jorder > 0) {
        angle = 0.01f*m_rotation+0.5f*m_ang;
        matset(angle,m_xpos,m_ypos);
        drawLine(-len,len,jorder == 4 ? m_colon : m_coloff);
        if(jorder == 4) break;
      }
      ++jorder;

      if(jorder > 0) {  
        angle = 0.01f*m_rotation-0.5f*m_ang;
        matset(angle,m_xpos,m_ypos);
        drawLine(-len,len,jorder == 4 ? m_colon : m_coloff);
        if(jorder == 4) break;
     }
      ++jorder;
    }
    tft.endWrite();
    if(m_drfirst) drawEnd();
  }

  void setSwitch(Switch *sw1, Switch* sw2) {
    m_switch1 = sw1;
    m_switch2 = sw2;
    m_switch1->Init();
    m_switch2->Init();
  }

  void setState(uint8_t state) {
    if(m_switch1) m_switch1->Change(state & 1);
    if(m_switch2) m_switch2->Change((state>>1) & 1);
  }

private:
  uint16_t m_len;    // length of the straight part of the switch
  uint8_t m_ang;  // angle of the curved part of the switch
  uint16_t m_rad;    // radius of the curved part of the switch
  Switch* m_switch1; // Switch servo & frog juicer
  Switch* m_switch2; // Switch servo & frog juicer
};
//================================= slip ===================================

//================================= turnout ================================
class turnout: public track {
public:
  turnout(float len, uint8_t ang, float rad, kside side):
    track(side),
    m_len(len*100+0.5f),
    m_ang(ang),
    m_rad(rad*100+0.5f),
    m_switch(nullptr)
    {};

  void draw(uint8_t state) {
/*
* x      -- position of the start of the switch
* y      -- position of the start of the switch
* rot    -- rotation angle
*/
    int8_t jorder = 1-state;

    const uint16_t len = 0.01f*m_len*m_scale+0.5f;
    const uint16_t irad = 0.01f*m_rad*m_scale+0.5f;
    const float ang1 = m_side == kleft ? 90.f-m_ang : 270.f;
     
    if(m_drfirst) drawInit();
    setWin(upack(0),upack(1),upack(2),upack(3));

    tft.startWrite();
    while(1) {
      if(jorder > 0) {
        drawArc(irad,ang1,ang1+m_ang,jorder == 2 ? m_colon : m_coloff);
        if(jorder == 2) break;
      }
      ++jorder;

      if(jorder > 0) {
        drawLine(0,len,jorder == 2 ? m_colon : m_coloff);
        if(jorder == 2) break;
      }
      ++jorder;
   }
   tft.endWrite();
   if(m_drfirst) drawEnd();
  }

  void setSwitch(Switch *sw) {
    m_switch = sw;
    m_switch->Init();
  }

  void setState(uint8_t state) {
    if(m_switch) m_switch->Change(state & 1);
  }

private:
  uint16_t m_len;    // length of the straight part of the switch
  uint8_t m_ang;  // angle of the curved part of the switch
  uint16_t m_rad;    // radius of the curved part of the switch
  Switch* m_switch; // Switch servo & frog juicer
};
//================================= turnout ================================

//================================= straight ===============================
class straight: public track {
public:
  straight(float len):
    track(),
    m_len(len*100+0.5f)
    {};

  const float getLen() const {return 0.01f*m_len;}

  void draw(uint8_t state) override {
/*
* x      -- position of the start of the switch
* y      -- position of the start of the switch
* rot    -- rotation angle
*/
    const int16_t xlen = 0.5f*0.01f*m_len*m_scale+0.5f; 

    if(m_drfirst) drawInit();
    setWin(upack(0),upack(1),upack(2),upack(3));

    tft.startWrite();
    drawLine(-xlen,xlen,state == 0 ? m_coloff : m_colon);
    tft.endWrite();
    if(m_drfirst) drawEnd();
  }

private:
  uint16_t m_len;    // length of the straight part of the switch
};
//================================= straight ===============================

//================================= trackset ===============================
class trackset {
#define MAXTRACK 3
public:
  trackset():
  m_ntrack(0)
  {} 

  track* addTrack(track *newt) {
    if(m_ntrack == MAXTRACK) {
      Serial.println(F("addTrack cannot add another track"));
      return nullptr;
    } else {
      m_cont[m_ntrack] = newt;
      return m_cont[m_ntrack++];
    }
  }

  void draw(uint8_t state) {
    for(uint8_t i = 0; i<m_ntrack; ++i) 
       m_cont[i]->draw(state);
  }

private:
  track     *m_cont[MAXTRACK];
  uint8_t   m_ntrack;
};

bool checkRoute(const uint8_t state[8]) {
  if((state[0] == 0 && state[4] == 0) || (state[7] == 0 && state[2] == 0) || (state[7] == 2 && state[4] == 1) || 
     (state[7] == 3 && state[2] == 0) || (state[6] == 1 && state[5] == 0) || (state[6] == 2 && state[1] == 0) || 
     (state[6] == 3 && state[5] == 0) || (state[6] == 0 && state[5] == 1) || (state[6] == 1 && state[1] == 0) || 
     (state[3] == 0 && state[5] == 1) || (state[3] == 0 && state[5] == 3) || (state[6] == 0 && state[5] == 2) ||
     (state[0] == 0 && state[4] == 2) || (state[7] == 1 && state[4] == 3) || (state[3] == 1 && state[5] == 0) || 
     (state[7] == 1 && state[2] == 1) || (state[6] == 3 && state[5] == 3) || (state[5] == 3 && state[7] == 2) || 
     (state[3] == 1 && state[5] == 2) || (state[4] == 1 && state[7] == 0) || (state[7] == 2 && state[2] == 1) ||
     (state[5] == 3 && state[7] == 0) || (state[4] == 3 && state[7] == 3) || (state[1] == 1 && state[6] == 0) ||
     (state[5] == 2 && state[6] == 2) || (state[6] == 2 && state[4] == 3) || (state[6] == 3 && state[1] == 1) ||
     (state[5] == 1 && state[6] == 2) || (state[6] == 1 && state[5] == 3) || (state[4] == 0 && state[7] == 1) ||
     (state[0] == 1 && state[4] == 1) || (state[4] == 2 && state[7] == 2) || (state[4] == 2 && state[6] == 3) ||
     (state[0] == 1 && state[4] == 3) || (state[7] == 3 && state[4] == 0) || (state[4] == 0 && state[6] == 2) ||
     (state[4] == 2 && state[6] == 1) || (state[5] == 2 && state[7] == 3) || (state[4] == 2 && state[7] == 0) ||
      0) 
      return false;
    else 
      return true;
}
//================================= trackset ===============================

#ifdef NEVER
size_t freeHeap() {
  #if defined(ESP32)
    return ESP.getFreeHeap();

  #elif defined(ESP8266)
    return ESP.getFreeHeap();

  #elif defined(__AVR__)
    extern char __heap_start;
    extern void* __brkval;
    char top;
    return (size_t) (&top - (char*)(__brkval ? __brkval : &__heap_start));

  #elif defined(ARDUINO_ARCH_RP2040)
    // Earle Philhower core
    return rp2040.getFreeHeap();

  #elif defined(TEENSYDUINO) || defined(ARDUINO_ARCH_SAMD) || \
        defined(ARDUINO_ARCH_MEGAAVR) || defined(ARDUINO_ARCH_RENESAS)
    // Uses newlib; mallinfo() is available
#ifdef NEVER
    #include <malloc.h>
    struct mallinfo mi = mallinfo();
    return (size_t)mi.fordblks;   // free bytes in the heap
#endif
  #else
    return 0; // unknown target
  #endif
}
#endif

turnout turn1(23,15,87.35f,kright);
Switch switch1(2,3,4,1200,1800);
turnout turn2(23,15,87.35f,kleft);
Switch switch2(5,6,7,1200,1800);
turnout turn3(23,15,87.35f,kright);
Switch switch3(11,12,13,1200,1800);
turnout turn4(23,15,87.35f,kleft);
Switch switch4(22,23,24,1200,1800);
slip slip1(23,15,105);
Switch switch5(25,26,27,1200,1800);
Switch switch6(28,29,30,1200,1800);
slip slip2(23,15,105);
Switch switch7(31,32,33,1200,1800);
Switch switch8(34,35,36,1200,1800); 
slip slip3(23,15,105);
Switch switch9(37,38,39,1200,1800);
Switch switch10(40,41,42,1200,1800);
slip slip4(23,15,105);
Switch switch11(43,44,45,1200,1800);
Switch switch12(46,47,48,1200,1800);
straight tr1(5.f);
straight tr2(5.f);
straight tr3(4.f);
straight tr4(22.f);
straight tr5(22.f);
straight tr6(4.f);
straight tr7(22.f);
straight tr8(22.f);
straight str3(56.f);
straight str4(56.f);
trackset ts1;
trackset ts2;
trackset ts3;
track *tvect[8];
uint8_t ostate[8];
uint8_t state[8];
uint16_t xxpos[8];
uint16_t yypos[8];
uint16_t xpos = 0;
uint16_t ypos = 0;
uint16_t tw = 0;
uint16_t th = 0;
const int8_t xtshift = -5;
const int8_t ytshift = -4;
char buf[6];                        // enough for "0xFFF\0"
uint16_t coltext;
uint16_t colback;
struct {
      uint16_t x = 0;
      uint16_t y = 0;
      uint8_t  w = 0;
      uint8_t  h = 0;
} rb;

void initState() {
  state[0]=0;
  state[1]=0;
  state[2]=0;
  state[3]=0;
  state[4]=1;
  state[5]=0;
  state[6]=0;
  state[7]=1;
  tft.fillRect(15,0,dw-30,0.167*dh+0.5f,colback);
  for(uint8_t j = 0; j<8; ++j) {
    if(ostate[j] != state[j]) {
      tvect[j]->setState(state[j]);
      xpos = xxpos[j];
      ypos = yypos[j];
      tft.setCursor(xpos-2.5*tw,ypos);
      tft.print(j+1);
      tft.print(F("-"));
      tft.setCursor(xpos,ypos);
      xpos += xtshift;
      ypos += ytshift;
      tft.fillRect(xpos,ypos,20,22,colback);
      tft.drawRect(xpos,ypos,20,22,GREEN);
      tvect[j]->draw(state[j]);
      tft.print(state[j]);
      ostate[j] = state[j];
    }
  }
}

inline void setWin(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
#ifdef ILI9341
  // Adafruit wants x1,y1 inclusive
  tft.setAddrWindow(x, y, x + (w ? w - 1 : 0), y + (h ? h - 1 : 0));
#elif defined(TFT25257)
  tft.setAddrWindow(x, y, w, h);
#endif
}

bool getPoint(uint16_t &x, uint16_t &y) {
  static bool wasTouch = false;
  bool retval = false;
  x = 0;
  y = 0;
#ifdef ILI9341
  bool touching = ts.touched();
  if (touching && !wasTouch) {
    p = ts.getPoint();
#endif
#ifdef TFT25257
  p = ts.getPoint();
// if sharing pins, you'll need to fix the directions of the touchscreen pins
  pinMode(XM, OUTPUT);digitalWrite(XM, HIGH);
  pinMode(YP, OUTPUT);digitalWrite(YP, HIGH);

  bool touching = (p.z > MINPRESSURE && p.z < MAXPRESSURE);
  if (touching && !wasTouch) {
#endif
    mapP(x, y);
    retval = true;
  }
  wasTouch = touching;
  return retval;
}
    
 

// 6 Aand 8.6
void setup() {
  Serial.begin(9600);
  uint16_t begarg = 0;
#ifdef ILI9341
  if (!ts.begin(40)) { Serial.println(F("FT6206 not found")); }
#endif
#ifdef TFT25257
  begarg = tft.readID();
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);
 
#endif
 
  Serial.println(begarg);
  tft.begin(begarg);
  tft.setRotation(Orientation);
  
  const uint8_t palette = 2;
  
  if(palette == 1) {
    coltext = WHITE;
    colback = BLACK;
  } else {
    coltext = BLACK;
    colback = WHITE;
  }
  tft.fillScreen(colback);
  yield();

  preplut();

#ifdef NEVER
  const float hnorm = 1.f/RAND_MAX;
  const uint32_t nrep = 10;
  const float hnorm1 = 1.f/nrep;
  float sumdiff1 = 0;
  float sumdiff2 = 0;
  float sumdiff12 = 0;
  float sumdiff22 = 0;
  for(uint32_t i=0; i<nrep; ++i) {
    if(i%1000 == 0) Serial.println("Working");
    float ang = 1000.f*(1.f-2.f*rand()*hnorm);
//    ang = 90.*rand()*hnorm;
    float diff1 = cos(ang*degrad) - lutcos(ang/**degrad*/);
    sumdiff1 += diff1;
    sumdiff12 += diff1*diff1;
    if(fabs(diff1) > 5e-7f) {
      Serial.print(F("cos diff "));
      Serial.print(diff1,7);
      Serial.print(F(", "));
      Serial.println(ang);
    }
    float diff2 = sin(ang*degrad) - lutsin(ang/**degrad*/);
    sumdiff2 += diff2;
    sumdiff22 += diff2*diff2;
    if(fabs(diff2) > 5e-7f) {
      Serial.print(F("sin diff "));
      Serial.print(diff2,7);
      Serial.print(F(", "));
      Serial.println(ang);
    }
  }
  Serial.print(F("Average error cos "));Serial.print(sumdiff1*hnorm1,7);
  Serial.print(F(" +- "));Serial.println(sqrt(sumdiff12*hnorm1 - sumdiff1*hnorm1*sumdiff1*hnorm1));
  Serial.print(F("              sin "));Serial.print(sumdiff2*hnorm1,7);
  Serial.print(F(" +- "));Serial.println(sqrt(sumdiff22*hnorm1 - sumdiff2*hnorm1*sumdiff2*hnorm1));
#endif

  // read diagnostics (optional but can help debug problems)
#ifdef ILI9341
  uint8_t x = tft.readcommand8(ILI9341_RDMODE);
  Serial.print(F("Display Power Mode: 0x")); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDMADCTL);
  Serial.print(F("MADCTL Mode: 0x")); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDPIXFMT);
  Serial.print(F("Pixel Format: 0x")); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDIMGFMT);
  Serial.print(F("Image Format: 0x")); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDSELFDIAG);
  Serial.print(F("Self Diagnostic: 0x")); Serial.println(x, HEX); 
#endif
  Serial.print(F("TFT width:  ")); Serial.println(dw=tft.width());
  Serial.print(F("TFT height: ")); Serial.println(dh=tft.height());

/*
* Define scale as pixels per cm
*/  
//  Serial.print(F("Free RAM before: ")); Serial.println(freeHeap());
 
  scale = dw/102.f;
  track::setScale(scale);
  const float dist1 = 6.4f;
  const float dist2 = 8.6f;
  const uint16_t heig1 = dh/2-(dist2/2+dist1)*scale+0.5f;
  const uint16_t heig2 = heig1+dist1*scale+0.5f;
  const uint16_t heig3 = heig2+dist2*scale+0.5f;
  const uint16_t heig4 = heig3+dist1*scale+0.5f;

  ts1.addTrack(&tr1)->setPosition( 15,dw/2,dh/2);
  ts1.addTrack(&tr2)->setPosition(-15,dw/2,dh/2);;
  ts1.draw(0);

  ts2.addTrack(&tr3)->setPosition(0,dw/2,heig2);
  float hl = 0.5f*tr4.getLen()*scale;
  ts2.addTrack(&tr4)->setPosition(0,hl,heig2);
  hl = 0.5f*tr7.getLen()*scale;
  ts2.addTrack(&tr5)->setPosition(0,dw-hl,heig2);
  ts2.draw(0);

  ts3.addTrack(&tr6)->setPosition(0,dw/2,heig3);
  hl = 0.5f*tr7.getLen()*scale;
  ts3.addTrack(&tr7)->setPosition(0,hl,heig3);
  hl = 0.5f*tr7.getLen()*scale;
  ts3.addTrack(&tr8)->setPosition(0,dw-hl,heig3);
  ts3.draw(0);

  str3.setPosition(0,dw/2,heig1);
  str3.draw(0);

  str4.setPosition(0,dw/2,heig4); 
  str4.draw(0);

//  Serial.print(F("Free RAM after: "));  Serial.println(freeHeap());
  yield();
  
  //Serial.println(rgb888_to_rgb565_round(150,152,150));


  turn1.setPosition(0,0,heig1);
  //turn1.setSwitch(&switch1);
  turn2.setPosition(180,dw,heig1);
  //turn2.setSwitch(&switch2);
  turn3.setPosition(180,dw,heig4);
  //turn3.setSwitch(&switch3);
  turn4.setPosition(0,0,heig4);
  //turn4.setSwitch(&switch4);

  //float slip (L= 230mm 15° R=1050mm)

  int16_t xshift = 35.*scale+0.5f;
  slip1.setPosition(7.5,xshift,heig2);
  //slip1.setSwitch(&switch5, &switch6);
  slip2.setPosition(-7.5,xshift,heig3);
  //slip2.setSwitch(&switch7, &switch8);
  slip3.setPosition(-7.5,dw-xshift,heig2);
  //slip3.setSwitch(&switch9, &switch10);
  slip4.setPosition(7.5,dw-xshift,heig3);
  //slip4.setSwitch(&switch11, &switch12);
  
  tft.setTextSize(2);
  for(uint8_t i=0; i<8; ++i) {
    ostate[i] = 0XF;
    state[i] = 0XF;
  }

  xxpos[0] = uint16_t(0.12*dw+0.5f);
  xxpos[1] = uint16_t(0.92*dw+0.5f);
  xxpos[2] = uint16_t(0.92*dw+0.5f);
  xxpos[3] = uint16_t(0.12*dw+0.5f);
  xxpos[4] = uint16_t(0.36*dw+0.5f);
  xxpos[5] = uint16_t(0.36*dw+0.5f);
  xxpos[6] = uint16_t(dw*(1-0.373)+0.5f);
  xxpos[7] = uint16_t(dw*(1-0.373)+0.5f);
  yypos[0] = uint16_t(heig1-0.125*dh+0.5f);
  yypos[1] = uint16_t(heig1-0.125*dh);
  yypos[2] = uint16_t(heig4+0.08*dh+0.5f);
  yypos[3] = uint16_t(heig4+0.08*dh+0.5f);
  yypos[4] = uint16_t(heig1-0.125*dh+0.5f);
  yypos[5] = uint16_t(heig4+0.08*dh+0.5f);
  yypos[6] = uint16_t(heig1-0.125*dh+0.5f);
  yypos[7] = uint16_t(heig4+0.08*dh+0.5f);

  tvect[0] = &turn1;
  tvect[1] = &turn2;
  tvect[2] = &turn3;
  tvect[3] = &turn4;
  tvect[4] = &slip1;
  tvect[5] = &slip2;
  tvect[6] = &slip3;
  tvect[7] = &slip4;
  
  // For classic font (setFont(NULL)) with scaling:
  int16_t x1;
  int16_t y1; 
  tft.setFont(NULL);
  tft.setTextSize(2);              // example
  tft.getTextBounds("A", 0, 0, &x1, &y1, &tw, &th); // bounding box of 'A'

  uint16_t istart = 0X0FBB + 1;
  istart = 0;

  uint8_t nvalid = 0;

  tft.setTextColor(coltext);
  
if(0) {
  for(uint16_t i=istart; i<0XFFF+1; ++i) {
//    tft.print(i,HEX);
// T1 | T2 | T3 | T4 | S1 | S2 | S3 | S4
//  1    1    1    1    2    2    2    2
// 11   10    9    8  7-6  5-4  3-2  1-0
//800  400  200  100    C0  30    C    3
//

    state[0] = i>>11 & 1;
    state[1] = i>>10 & 1;
    state[2] = i>>9 & 1;
    state[3] = i>>8 & 1;
    state[4] = i>>6 & 3;
    state[5] = i>>4 & 3;
    state[6] = i>>2 & 3;
    state[7] = i & 3;

  
//    bool skip = false;

    uint16_t ctext = coltext;

{
  if(!checkRoute(state)) {
//    skip = true;
    ctext = RED;
    continue;
  }
 
    nvalid++;
    tft.fillRect(0,0,dw,0.167*dh+0.5f,colback);
    tft.setCursor(dw/2-9*tw,0.083*dh+0.5f);
    tft.setTextColor(ctext);
    tft.print(F("Route 0X"));
    tft.print(itoa(i&0XFFF,buf,16));
    /* if(skip) {
      continue;
    }*/
    tft.setTextColor(coltext);

    for(uint8_t j = 0; j<8; ++j) {
      if(state[j] != ostate[j]) {
        xpos = xxpos[j];
        ypos = yypos[j];
        tft.setCursor(xpos-2.5*tw,ypos);
        tft.print(j+1);
        tft.print(F("-"));
        tft.setCursor(xpos,ypos);
        xpos += xtshift;
        ypos += ytshift;
        tft.fillRect(xpos,ypos,20,22,colback);
        tft.drawRect(xpos,ypos,20,22,GREEN);
        tvect[j]->draw(state[j]);
        tft.print(state[j]);
        ostate[j] = state[j];
      }
    }
  
    tft.setTextColor(coltext);
    tft.setCursor(dw/2+2*tw,0.083f*dh+0.5f);
    tft.print(F("...done"));
    delay(100);
  }
}
  Serial.print("Numnber of valid configurations ");Serial.println(nvalid);
}
  initState();

    tft.fillCircle(10,10,3,RED);
    tft.fillCircle(dw-10,10,3,RED);
    tft.fillCircle(10,dh-10,3,RED);
    tft.fillCircle(dw-10,dh-10,3,RED);

    rb.x = dw/2-2.6f*tw+0.5f;
    rb.y = 0.85f*dh+0.5f;
    rb.w = 5.2f*tw+0.5f;
    rb.h = 1.2f*th+0.5f;
    
    tft.drawRect(rb.x,rb.y,rb.w,rb.h,BLACK);
    tft.setCursor(rb.x+0.2f*tw, rb.y+0.1f*th);
    tft.print("reset");
}


void loop(void) {
  if(checkRoute(state))
  for(uint8_t j = 0; j<8; ++j) {
      tvect[j]->setState(state[j]);
  }

  uint16_t xx = 0;
  uint16_t yy = 0;

  if(getPoint(xx,yy)) {
    if(xx>rb.x && xx<rb.x+rb.w && yy>rb.y && yy<rb.y+rb.h) {
      const char* reset = "reset";
      tft.setCursor(rb.x+0.2f*tw, rb.y+0.1f*th);
      tft.setTextColor(RED);
      tft.print(reset);
      tft.setTextColor(BLACK);
      initState();
      tft.setCursor(rb.x+0.2f*tw, rb.y+0.1f*th);
      tft.print(reset);
    }
    for(uint8_t j = 0; j<8; ++j) {
      if(tvect[j]->inPoint(xx,yy)) {
        tft.fillRect(15,0,dw-30,0.167*dh+0.5f,colback);
        if(j<4) 
          state[j] = (state[j]+1) & 1;
        else
          state[j] = (state[j]+1) & 3;
        xpos = xxpos[j];
        ypos = yypos[j];
        tft.setCursor(xpos,ypos);
        xpos += xtshift;
        ypos += ytshift;
        tft.fillRect(xpos,ypos,20,22,colback);
        tft.drawRect(xpos,ypos,20,22,GREEN);
        tvect[j]->draw(state[j]);
        tft.print(state[j]);
        if(!checkRoute(state)) {
          tft.setCursor(dw/2-4*tw,0.083*dh+0.5f);
          tft.setTextColor(RED);
          tft.print(F("no route!"));
          tft.setTextColor(coltext);
        }
      }
      ostate[j] = state[j];
    }
  }
}