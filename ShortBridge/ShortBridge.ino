
int led = 9;           // the PWM pin the LED is attached to
int frontred = 8;
int rearred = 10;
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by
int sens1 = 6;
int sens2 = 7;
int vccs1 = LOW;
int vccs2 = LOW;
unsigned long fademils = millis();
unsigned long blinkred = millis();
int highlow = 0;
int bef1 = 0;
int aft1 = 0;
bool ss1 = FALSE:
bool ss2 = FALSE;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT); //LED on Model B
  pinMode(frontred, OUTPUT); // front red lights
  pinMode(rearred, OUTPUT); // front red lights
  pinMode(sens1, OUTPUT); // front red lights
  pinMode(sens2, OUTPUT); // front red lights
  pinMode(13, OUTPUT); //LED on Model A  or Pro
  Serial.begin(9600); 
}

// the loop routine runs over and over again forever:
void loop() {
  digitalWrite(sens1,LOW);
  bef1 = analogRead(A0);
  digitalWrite(sens1,HIGH);
  delay(10);
  aft1 = analogRead(A0);
  Serial.println(aft1-bef1);
  if(aft1-bef1 < 100)
    digitalWrite(13,HIGH);
   else 
    digitalWrite(13,LOW);   

  digitalWrite(sens2,LOW);
  bef2 = analogRead(A1);
  digitalWrite(sens1,HIGH);
  delay(10);
  aft2 = analogRead(A1);
  Serial.println(aft1-bef1);
  if(aft2-bef2 < 100)
    digitalWrite(13,HIGH);
   else 
    digitalWrite(13,LOW);   

  if(ss1 || ss2
  if(millis()-fademils > 50) {
    fademils = millis();
    // set the brightness of pin 9:
    analogWrite(led, brightness);

    // change the brightness for next time through the loop:
    brightness = brightness + fadeAmount;

    // reverse the direction of the fading at the ends of the fade:
    if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
    }
    if(millis() - blinkred > 500) {
      blinkred = millis();
      highlow = 1-highlow;
      digitalWrite(frontred,highlow);
      digitalWrite(rearred,1-highlow);
    }
  }
}
