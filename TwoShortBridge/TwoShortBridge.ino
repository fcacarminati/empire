//
// Sketch for two bridges
//
// Pin configuration
//
// Bridge 1
// D2  driver right red lights
// D3  driver bridge lights
// D4  driver read red lights
// D5  digital input for sensor 1R
// D6  digital input for sensor 1L
//
// Bridge 2
// D7  driver right red lights
// D9  driver bridge lights
// D8  driver left red lights
// D10  digital input for sensor 2R
// D11 digital input dor sensor 2L
//
 
// Bridge 1
int redR_b1 = 2;   // pin for right red lights
int lights_b1 = 3; // pin white lights
int redL_b1 = 4;   // pin for left red lights
int sensR_b1 = 5;  // right sensor
int sensL_b1 = 6;  // left sensor
int brightness_b1 = 0;
unsigned long fademils_b1 = millis();
unsigned long blinkred_b1 = millis();
int highlow_b1 = 0;
short curR_b1 = 0;    // current state of right sensor 
short curL_b1 = 0;    // current state of left sensor
short preR_b1 = 0;    // previous state of right sensor
short preL_b1 = 0;    // previous state of left sensor
short stateR_b1 = 0;  // logical state of right side
                      // 0 off; 1 set; 2 cleared; 3 on
short stateL_b1 = 0;  // logical state of left side
short fade_b1 = 0;    // fading direction

// Bridge 2
int redR_b2 = 7;
int lights_b2 = 9;
int redL_b2 = 8;
int sensR_b2 = 10;
int sensL_b2 = 11;
int brightness_b2 = 0;
unsigned long fademils_b2 = millis();
unsigned long blinkred_b2 = millis();
int highlow_b2 = 0;
short curR_b2 = 0;
short curL_b2 = 0;
short preR_b2 = 0;
short preL_b2 = 0;
short stateR_b2 = 0;  // logical state of right side
                      // 0 off; 1 set; 2 cleared; 3 on
short stateL_b2 = 0;  // logical state of left side
short fade_b2 = 0;

const int fadeAmount = 5;    // how many points to fade the LED by

enum state {kOn, kSet, kClear, kOff};

// the setup routine runs once when you press reset:
void setup() {                
  // initialize pins
  // Bridge 1
  pinMode(redR_b1, OUTPUT);
  pinMode(lights_b1, OUTPUT);
  pinMode(redL_b1, OUTPUT);
  pinMode(sensR_b1, INPUT);
  pinMode(sensL_b1, INPUT);
  
  digitalWrite(redR_b1, HIGH);
  digitalWrite(lights_b1, HIGH);
  digitalWrite(redL_b1, HIGH);
  
  // Bridge 2
  pinMode(redR_b2, OUTPUT);
  pinMode(lights_b2, OUTPUT);
  pinMode(redL_b2, OUTPUT);
  pinMode(sensR_b2, INPUT);
  pinMode(sensL_b2, INPUT);
  
  digitalWrite(redR_b2, HIGH);
  digitalWrite(lights_b2, HIGH);
  digitalWrite(redL_b2, HIGH);
  
  Serial.begin(57600); 
}

// the loop routine runs over and over again forever:
void loop() {
  curR_b1 = digitalRead(sensR_b1);
  stateR_b1 = curR_b1 + 2*preR_b1;
  preR_b1 = curR_b1;
  
  curL_b1 = digitalRead(sensL_b1);
  stateL_b1 = curL_b1 + 2*preL_b1;
  preL_b1 = curL_b1;
  
// Is the train entering?
  if((stateR_b1 == kSet && stateL_b1 == kOff) ||
     (stateR_b1 == kOff && stateL_b1 == kSet))
    fade_b1 = 1;
  
// Is the train exiting?
  else
    if((stateR_b1 == kClear && stateL_b1 == kOff) ||
       (stateR_b1 == kOff   && stateL_b1 == kClear))
      fade_b1 = -1;
    
  if(fade_b1 != 0) {
    
    if(millis()-fademils_b1 > 50) {
      fademils_b1 = millis();
      
      // change the brightness for next time through the loop:
      brightness_b1 = brightness_b1 + fade_b1 * fadeAmount;
      
      // set the brightness bridge lights
      analogWrite(lights_b1, brightness_b1);
      
      // stop fading if we have reached max or min
      if(brightness_b1 <= 0) {
	brightness_b1 = 0;
	fade_b1 = 0;
      } else if(brightness_b1 >= 255) {
	brightness_b1 = 255;
	fade_b1 = 0;
      }
    }
  }
  if(brightness_b1 > 0) {
    if(millis() - blinkred_b1 > 500) {
      blinkred_b1 = millis();
      highlow_b1 = 1-highlow_b1;
      digitalWrite(redR_b1,highlow_b1);
      digitalWrite(redL_b1,1-highlow_b1);
    }
  } else {
    digitalWrite(redR_b1,HIGH);
    analogWrite(lights_b1, 255);
    digitalWrite(redL_b1,HIGH);
  }
}
