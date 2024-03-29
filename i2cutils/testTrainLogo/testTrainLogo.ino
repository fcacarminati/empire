#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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


void setup() {
  Serial.begin(9600);
}

void loop() {
  static int il = 0;
  Serial.println(++il);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Default OLED address, usually  
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(2,51);
  display.print("Speed:");
  delay(100);
  display.drawBitmap(0, 0, TrainLogoR, 40, 40, SSD1306_WHITE);
  display.startscrollleft(0x00,0x04);
  delay(100);
  display.display();

  delay(100);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D); // Default OLED address, usually
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,51);
  display.print("Speed:");
  delay(200);
  display.print("40 kmh");
  display.drawBitmap(0, 0, TrainLogoR, 40, 40, SSD1306_WHITE);
  display.startscrollleft(0x00,0x04);
  delay(100);
  display.display();

  delay(5000);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Default OLED address, usually  
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(2,51);
  display.print("Speed:");
  delay(100);
  display.drawBitmap(0, 0, TrainLogoL, 40, 40, SSD1306_WHITE);
//  display.startscrollright(0x00,0x0c);
  display.startscrollright(0x00,0x04);
  delay(100);
  display.display();

 
  delay(1500);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D); // Default OLED address, usually
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(2,51);
  display.print("Speed:");
  delay(100);
  display.drawBitmap(0, 0, TrainLogoL, 40, 40, SSD1306_WHITE);
  display.startscrollright(0x00,0x04);
  delay(100);
  display.display();

  delay(5000);


}
