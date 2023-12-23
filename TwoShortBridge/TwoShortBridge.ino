//
// Sketch for two bridges
//
// Pin configuration
//
// Bridge 1
// A0 analog input for sensor 1R
// A1 analog input for sensor 1L
// D2 front red lights
// D3 bridge lights
// D4 read red lights
// D5 control for sensor 1R
// D6 control for sensor 1L
//
// Bridge 2
// A2 analog input for sensor 2R
// A3 analog input for sensor 2L
// D7 front red lights
// D9 bridge lights
// D8 rear red lights
// D9 control for sensor 2R
// D10 control for sensor 2L
//
 
// Bridge 1
int red_front_b1 = 2;
int lights_b1 = 3;
int red_back_b1 = 4;
int sensR_b1 = 5;
int sensL_b1 = 6;
int brightness_b1 = 0;
unsigned long fademils_b1 = millis();
unsigned long blinkred_b1 = millis();
int highlow_b1 = 0;
int befR_b1 = 0;
int befL_b1 = 0;
int aftR_b1 = 0;
int aftL_b1 = 0;
int fade_b1 = 1;

// Bridge 2
int red_front_b2 = 7;
int lights_b2 = 9;
int red_back_b2 = 8;
int sensR_b2 = 9;
int sensL_b2 = 10;
int brightness_b2 = 0;
unsigned long fademils_b2 = millis();
unsigned long blinkred_b2 = millis();
int highlow_b2 = 0;
int befR_b2 = 0;
int befL_b2 = 0;
int aftR_b2 = 0;
int aftL_b2 = 0;
int fade_b2 = 1;

const int fadeAmount = 5;    // how many points to fade the LED by

bool ss1 = false;
bool ss2 = false;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize pins
  // Bridge 1
  pinMode(red_front_b1, OUTPUT);
  pinMode(lights_b1, OUTPUT);
  pinMode(red_back_b1, OUTPUT);
  pinMode(sensR_b1, OUTPUT);
  pinMode(sensL_b1, OUTPUT);

  // Bridge 2
  pinMode(red_front_b2, OUTPUT);
  pinMode(lights_b2, OUTPUT);
  pinMode(red_back_b2, OUTPUT);
  pinMode(sensR_b2, OUTPUT);
  pinMode(sensL_b2, OUTPUT);

  pinMode(13, OUTPUT); //LED on Model A  or Pro
  Serial.begin(57600); 
}

// the loop routine runs over and over again forever:
void loop() {
  const short nrep = 5;
  befR_b1 = 0;
  for(short i=0; i<nrep; ++i) 
    befR_b1 += analogRead(A0);
  befR_b1 /= nrep; 
  delay(50);
  aftR_b1 = 0;
  for(short i=0; i<nrep; ++i) 
    aftR_b1 += analogRead(A0);
  aftR_b1 /= nrep; 
  Serial.println(befR_b1-aftR_b1);
  if(aftR_b1-befR_b1 < 100)
    digitalWrite(13,HIGH);
   else 
    digitalWrite(13,LOW);   

  digitalWrite(sensL_b1,LOW);
  befL_b1 = analogRead(A1);
  digitalWrite(sensL_b1,HIGH);
  delay(10);
  aftL_b1 = analogRead(A1);
//  Serial.println(aftL_b1-befL_b1);
  if(aftL_b1-befL_b1 < 100)
    digitalWrite(13,HIGH);
   else 
    digitalWrite(13,LOW);   

  if(millis()-fademils_b1 > 50) {
    fademils_b1 = millis();
    // set the brightness bridge lights
    analogWrite(lights_b1, brightness_b1);

    // change the brightness for next time through the loop:
    brightness_b1 = brightness_b1 + fade_b1 * fadeAmount;

    // reverse the direction of the fading at the ends of the fade:
    if (brightness_b1 <= 0 || brightness_b1 >= 255) {
    fade_b1 = -fade_b1;
    }
    if(millis() - blinkred_b1 > 500) {
      blinkred_b1 = millis();
      highlow_b1 = 1-highlow_b1;
      digitalWrite(red_front_b1,highlow_b1);
      digitalWrite(red_back_b1,1-highlow_b1);
    }
  }
}
