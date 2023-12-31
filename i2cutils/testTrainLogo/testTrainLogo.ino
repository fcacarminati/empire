#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(128, 64, &Wire, -1);

// The Arduino Logo in a bitmap format
static const unsigned char PROGMEM arduinoLogo[] =  
{
 0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x03,0x03,0x00,0x03,0x08,0x04,0x01,0x00,0x00,0x00,
0x01,0x00,0x68,0x3e,0x00,0x46,0x9e,0xbd,0xc9,0xc6,0x6c,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x02,0x02,0x00,0x04,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x05,0x07,0x06,0x00,0x00,0x01,0x00,0x00,0x00,0x0f,0xdd,0xff,0xff,0xc8,0xff,0xff,0xff,0xff,
0xff,0xa3,0x00,0x01,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x07,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x0b,0x06,0x00,0x01,0x00,0x00,0x01,0x00,
0xe3,0xfe,0xfd,0xff,0xff,0xff,0xf8,0xf8,0xf9,0xf4,0xfc,0xfe,0x8a,0x00,0x00,0x00,0x01,0x00,0x00,
0x00,0x00,0x01,0x00,0x00,0x01,0x04,0x0b,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,
0x09,0x01,0x00,0x03,0x02,0x00,0x00,0x00,0x11,0xd3,0xfe,0xf8,0xf8,0xf8,0xf8,0xf8,0xf8,0xf9,0xf7,
0xf7,0xff,0xff,0x15,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x04,0x09,0x02,0x00,
0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x01,0x07,0x09,0x01,0x00,0x00,0x00,0x02,0x01,0x01,0x47,0xee,
0xf8,0xf9,0xf8,0xfa,0xf8,0xf8,0xf9,0xf8,0xf9,0xfa,0xf8,0xff,0xff,0x2a,0x01,0x00,0x01,0x00,0x00,
0x00,0x00,0x01,0x01,0x07,0x0a,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x08,
0x00,0x00,0x00,0x00,0x01,0x00,0x4f,0xff,0xff,0xf8,0xfa,0xf8,0xf8,0xf8,0xf9,0xf7,0xf9,0xf8,0xf8,
0xf9,0xff,0xb2,0x01,0x01,0x00,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x06,0x06,0x02,0x01,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x04,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x34,0xff,0xfe,0xf8,
0xfa,0xf9,0xf8,0xf8,0xf8,0xf8,0xf9,0xf9,0xf8,0xf8,0xfb,0xff,0x66,0x01,0x00,0x03,0x02,0x00,0x00,
0x00,0x01,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x00,0x02,0x02,0x00,0x00,0x00,0x02,0x03,0x00,0x00,
0x00,0x00,0x00,0x01,0x00,0x85,0xff,0xfa,0xf8,0xf9,0xf8,0xf9,0xf8,0xf8,0xf9,0xf9,0xf9,0xf8,0xf8,
0xf8,0xff,0xe5,0x00,0x00,0x01,0x03,0x02,0x00,0x00,0x00,0x03,0x02,0x00,0x00,0x00,0x00,0x01,0x00,
0x00,0x02,0x01,0x00,0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x8f,0xff,0xf8,0xf8,
0xf9,0xf8,0xf9,0xf8,0xf8,0xf8,0xfa,0xf8,0xf9,0xf9,0xf8,0xff,0xea,0x00,0x00,0x00,0x00,0x03,0x02,
0x02,0x01,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x04,0x05,0x02,0x00,0x01,0x00,0x00,
0x00,0x00,0x00,0x02,0x86,0xef,0xfc,0xf9,0xf8,0xf9,0xf5,0xf6,0xf8,0xf8,0xf7,0xf9,0xf9,0xff,0xfe,
0xff,0xfa,0x49,0x00,0x00,0x00,0x00,0x01,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x01,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x16,0xdf,0xff,0xfe,0xf8,0xf9,0xf8,
0xfe,0xff,0xff,0xff,0xff,0xff,0xff,0xf7,0xff,0xff,0xec,0x40,0x00,0x00,0x00,0x01,0x00,0x01,0x04,
0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x00,0x00,0x02,0x03,0x01,0x00,0x00,
0x00,0x17,0xe7,0xfe,0xf9,0xf8,0xf9,0xfa,0xff,0xff,0xed,0xaf,0xe6,0xec,0xe8,0xdd,0x99,0x27,0x52,
0x0a,0x00,0x01,0x00,0x00,0x00,0x03,0x02,0x00,0x00,0x02,0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x00,
0x02,0x01,0x00,0x00,0x01,0x02,0x02,0x01,0x01,0x00,0x00,0xae,0xfe,0xf9,0xf8,0xff,0xff,0xff,0xff,
0x2a,0x00,0x00,0x0a,0x05,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,
0x00,0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x03,0x01,
0x00,0xd5,0xff,0xf8,0xff,0xe2,0x92,0x6c,0x4c,0x01,0x00,0x01,0x00,0x01,0x00,0x00,0x01,0x00,0x00,
0x00,0x00,0x00,0x02,0x03,0x03,0x00,0x01,0x01,0x00,0x00,0x00,0x01,0x02,0x01,0x00,0x00,0x00,0x00,
0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x03,0x00,0x00,0xd8,0xff,0xff,0xbf,0x00,0x00,0x00,0x00,
0x00,0x00,0x02,0x00,0x00,0x01,0x02,0x02,0x01,0x00,0x00,0x00,0x04,0x09,0x04,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x02,0x02,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xd0,0xff,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x01,
0x00,0x03,0x09,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x03,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x5f,0xff,0xdd,0x06,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x06,0x07,0x00,0x00,0x00,0x01,0x01,0x00,0x01,
0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x03,0x01,0x03,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,
0x00,0xa7,0xff,0xfe,0xf5,0x4f,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
0x08,0x01,0x1e,0x4f,0x4e,0x4b,0x4d,0x4c,0x4b,0x4f,0x50,0x51,0x51,0x19,0x01,0x00,0x00,0x05,0x05,
0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0xc5,0xff,0xfe,0xff,0x4e,0x00,0x00,0x01,
0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x01,0x09,0x07,0x03,0x00,0xae,0xff,0xfe,0xff,0xfe,0xff,0xff,
0xff,0xfe,0xff,0xa3,0x00,0x01,0x03,0x00,0x02,0x02,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x02,
0x00,0x7b,0xff,0xff,0xee,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x06,0x07,
0x00,0x00,0xc2,0xff,0xfe,0xff,0xff,0xff,0xf1,0x7e,0x7b,0x7d,0x82,0x43,0x01,0x03,0x01,0x00,0x00,
0x01,0x00,0x01,0x01,0x00,0x01,0x00,0x01,0x03,0x02,0x00,0x88,0xff,0xff,0xed,0x12,0x00,0x00,0x00,
0x82,0x8a,0x5b,0x01,0x00,0x04,0x0a,0x05,0x00,0x00,0x00,0x00,0xcb,0xff,0xfa,0xf8,0xf9,0xff,0xd3,
0x00,0x00,0x00,0x01,0x03,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x02,0x00,
0x01,0x84,0xff,0xfe,0xee,0x07,0x00,0x00,0x3f,0xff,0xff,0xff,0xff,0x99,0x00,0x03,0x04,0x00,0x00,
0x00,0x00,0xd7,0xfe,0xf8,0xf8,0xf8,0xff,0xcf,0x00,0x01,0x00,0x02,0x02,0x01,0x00,0x00,0x00,0x00,
0x03,0x00,0x00,0x00,0x00,0x02,0x01,0x01,0x00,0x00,0x00,0x81,0xff,0xff,0xf0,0x01,0x00,0x01,0x00,
0xfe,0xfe,0xff,0x5e,0x00,0x00,0x00,0x00,0x01,0x00,0x01,0x00,0xe0,0xff,0xf8,0xf8,0xf9,0xff,0xc5,
0x00,0x02,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x02,0x00,0x00,0x02,0x02,0x00,0x25,0x7b,
0x7c,0xe2,0xfc,0xfa,0xfb,0xb8,0x91,0x92,0x92,0xf0,0xfe,0xff,0xfc,0xa9,0x92,0x95,0x95,0x95,0x95,
0x91,0x95,0xf2,0xfc,0xf8,0xf9,0xf8,0xfe,0xb9,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x01,0x05,0x06,0x02,0x00,0x00,0xe1,0xff,0xff,0xff,0xff,0xf8,0xf8,0xf9,0xff,0xff,0xff,0xff,
0xf5,0xf4,0xf8,0xff,0xff,0xff,0xff,0xfe,0xfe,0xff,0xff,0xff,0xfe,0xf6,0xf9,0xf9,0xf8,0xff,0x9a,
0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x07,0x08,0x01,0x00,0x34,0xff,0xff,
0xf9,0xf9,0xf9,0xf9,0xf8,0xf9,0xf9,0xf8,0xf9,0xfa,0xf3,0xf3,0xf8,0xf8,0xf8,0xf8,0xf8,0xf8,0xf8,
0xf8,0xf7,0xf7,0xf9,0xf9,0xfa,0xf9,0xfc,0xc7,0x5d,0x5e,0x5f,0x5f,0x65,0x32,0x00,0x00,0x00,0x00,
0x02,0x03,0x00,0x00,0x03,0x00,0x62,0xff,0xfd,0xf9,0xf8,0xf7,0xf9,0xf8,0xf9,0xf8,0xf8,0xf9,0xf7,
0xf9,0xf8,0xf8,0xf8,0xf8,0xf8,0xf8,0xf9,0xf9,0xf8,0xf8,0xf8,0xf8,0xf9,0xf9,0xf8,0xf8,0xf8,0xff,
0xff,0xff,0xff,0xff,0xa3,0x01,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x02,0x00,0x6d,0xff,0xfa,
0xf8,0xf8,0xf7,0xf8,0xf8,0xf8,0xf8,0xf9,0xf8,0xf8,0xf9,0xf8,0xf9,0xf8,0xf9,0xf9,0xf8,0xf8,0xf8,
0xf9,0xf9,0xf8,0xf7,0xf8,0xf9,0xf9,0xf9,0xf8,0xfc,0xfc,0xfc,0xfd,0xff,0x95,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x68,0xff,0xfc,0xf9,0xf8,0xf9,0xf8,0xf8,0xf9,0xf8,0xf7,0xf8,0xf8,
0xf9,0xf8,0xf8,0xf9,0xf8,0xf9,0xf8,0xf8,0xf9,0xf9,0xf9,0xf8,0xf7,0xf9,0xf8,0xf8,0xf8,0xf9,0xf7,
0xf8,0xf9,0xf7,0xff,0x91,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x3f,0xfe,0xfe,
0xf8,0xf8,0xf9,0xf8,0xf9,0xf8,0xfa,0xf7,0xf8,0xf8,0xf7,0xf9,0xf9,0xf8,0xf8,0xf8,0xf9,0xf8,0xf9,
0xf7,0xf8,0xf9,0xf7,0xf8,0xf9,0xf9,0xf8,0xf8,0xf9,0xf8,0xf7,0xf9,0xff,0x93,0x00,0x00,0x01,0x00,
0x00,0x00,0x00,0x00,0x00,0x01,0x00,0xd1,0xfe,0xf8,0xf8,0xf7,0xf9,0xf9,0xf8,0xf8,0xf8,0xfa,0xf9,
0xf8,0xf8,0xf7,0xf9,0xf9,0xf9,0xf9,0xf8,0xf7,0xf8,0xf8,0xf8,0xf7,0xf9,0xf8,0xf8,0xf8,0xf8,0xf9,
0xf8,0xf9,0xfa,0xfe,0x94,0x01,0x01,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x04,0x71,0xc6,0xfb,
0xf8,0xf9,0xf7,0xf9,0xf7,0xf9,0xf8,0xf9,0xf8,0xf8,0xf8,0xf9,0xf8,0xf7,0xf9,0xf8,0xf9,0xf8,0xf9,
0xfa,0xf8,0xf9,0xf8,0xf8,0xf9,0xf8,0xf8,0xf7,0xf8,0xf8,0xf8,0xf9,0xff,0x90,0x00,0x02,0x00,0x04,
0x00,0x00,0x00,0x00,0x00,0x0f,0xff,0xff,0xff,0xf8,0xf9,0xf8,0xf8,0xf7,0xf8,0xfe,0xff,0xff,0xf7,
0xf8,0xf8,0xf7,0xf7,0xfe,0xff,0xff,0xf9,0xf8,0xf5,0xf4,0xf9,0xf7,0xff,0xff,0xff,0xfa,0xf8,0xf7,
0xf8,0xf8,0xff,0xfe,0xa2,0x00,0x00,0x06,0x07,0x03,0x01,0x00,0x00,0x00,0x01,0x09,0xd1,0xe3,0xe8,
0xf9,0xf8,0xf7,0xfa,0xf7,0xdd,0xdd,0xe0,0xf8,0xfa,0xf8,0xf8,0xfa,0xf9,0xe0,0xdb,0xdc,0xf8,0xfb,
0xf5,0xf9,0xf8,0xe3,0xdc,0xd8,0xf5,0xf9,0xf8,0xf7,0xf9,0xfa,0xe6,0xe2,0x7c,0x00,0x06,0x09,0x04,
0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0xc3,0xff,0xf8,0xf8,0xf9,0xfd,0xfe,0x48,0x00,0x88,0xff,
0xf8,0xf7,0xfa,0xff,0x7a,0x01,0x54,0xff,0xfe,0xfa,0xfa,0xf9,0xff,0xaa,0x00,0x2a,0xf7,0xfe,0xf8,
0xf9,0xfe,0xbc,0x01,0x00,0x05,0x07,0x01,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xe5,
0xfa,0xf8,0xf9,0xf9,0xff,0x7f,0x00,0xc2,0xff,0xf8,0xf7,0xf8,0xf8,0xff,0xb8,0x00,0x96,0xff,0xf7,
0xf8,0xf8,0xff,0xde,0x00,0x6b,0xff,0xfa,0xf8,0xf8,0xf7,0xff,0xee,0x00,0x00,0x09,0x01,0x00,0x00,
0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xd5,0xff,0xf8,0xf8,0xf8,0xfd,0xff,0x62,0x00,0xb5,0xff,
0xf9,0xf8,0xf8,0xff,0xa8,0x00,0x82,0xff,0xfa,0xfa,0xf8,0xf7,0xff,0xd2,0x00,0x62,0xff,0xfc,0xf7,
0xf7,0xff,0xea,0x02,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x01,0x3e,
0xff,0xf8,0xf9,0xfe,0xcb,0x00,0x00,0x18,0xf4,0xff,0xf8,0xf8,0xff,0xf1,0x0c,0x00,0x00,0xe3,0xff,
0xf8,0xfe,0xfa,0x3b,0x01,0x00,0xdd,0xff,0xf9,0xf8,0xfb,0xff,0x70,0x00,0x00,0x00,0x00,0x00,0x00,
0x36,0x3a,0x36,0x38,0x36,0x39,0x39,0x35,0x21,0xaf,0xf9,0xf9,0xfa,0xeb,0x4f,0x2e,0x36,0x23,0x9d,
0xfa,0xfb,0xf7,0x92,0x24,0x36,0x28,0x71,0xf4,0xfc,0xf9,0xfc,0xb6,0x23,0x37,0x2a,0x6e,0xf5,0xfa,
0xfc,0xd8,0x2e,0x37,0x36,0x36,0x36,0x37,0x36,0x37,0xf9,0xff,0xff,0xff,0xff,0xff,0xfe,0xff,0xff,
0xf6,0xf8,0xf7,0xf0,0xf1,0xff,0xff,0xff,0xec,0xf7,0xf8,0xf8,0xf7,0xec,0xff,0xff,0xff,0xee,0xf4,
0xf8,0xf7,0xec,0xff,0xff,0xff,0xf0,0xf4,0xf7,0xf7,0xf7,0xed,0xf8,0xfe,0xff,0xff,0xff,0xff,0xff
};

void setup() {
  Serial.begin(9600);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 not found"));
    while(1){}
  }

  display.display();
  delay(1000);
  display.clearDisplay();
  
  display.drawBitmap(39, 0, arduinoLogo, 50, 40, SSD1306_WHITE);
  display.display();

}

void loop() {
}