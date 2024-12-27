int sensorPin1 = 0;    // The potentiometer is connected to
                      // analog pin 0
int sensorPin2 = 1;    // The potentiometer is connected to
                      // analog pin 1
                      
int ledPin = 13;      // The LED is connected to digital pin 13

// One more thing. If you declare variables outside of a function,
// as we have here, they are called "global variables" and can be
// seen by all the functions. If you declare variables within a 
// function, they can only be seen within that function. It's good
// practice to "limit the scope" of a variable whenever possible,
// but as we're getting started, global variables are just fine.

unsigned long threshold1 = 0;
unsigned long threshold2 = 0;

void setup() // this function runs once when the sketch starts up
{
  // We'll be using pin 13 to light a LED, so we must configure it
  // as an output.
 
  // Because we already created a variable called ledPin, and
  // set it equal to 13, we can use "ledPin" in place of "13".
  // This makes the sketch easier to follow.

  static int sensorValue1;
  static int sensorValue2;
  float avecount1 = 0;
  float avecount2 = 0;
  unsigned long numcount = 1000;
  float sigcount1 = 0;
  float sigcount2 = 0;

  
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);

  
  int i=0;
  for(i=0; i<numcount; ++i) {
     if((i+1)%50 == 1) digitalWrite(ledPin,HIGH);
     if((i+1)%50 == 5) digitalWrite(ledPin,LOW);   
     sensorValue1 = analogRead(sensorPin1);
     sensorValue2 = analogRead(sensorPin2);
     avecount1 += sensorValue1;
     sigcount1 += sensorValue1*sensorValue1;
     avecount2 += sensorValue2;
     sigcount2 += sensorValue2*sensorValue2;
  }
  avecount1 /=numcount;
  avecount2 /=numcount;
//         Serial.println(sigcount/numcount);
//         Serial.println(avecount);
  sigcount1 = sigcount1/numcount - avecount1*avecount1;
  if(sigcount1 <= 0) sigcount1 = avecount1/30.;
  else sigcount1 = sqrt(sigcount1);
  sigcount2 = sigcount2/numcount - avecount2*avecount2;
  if(sigcount2 <= 0) sigcount2 = avecount2/30.;
  else sigcount2 = sqrt(sigcount2);
  Serial.print("count1 ");Serial.print(avecount1);Serial.print(" +- ");Serial.println(sigcount1);
  Serial.print("count2 ");Serial.print(avecount2);Serial.print(" +- ");Serial.println(sigcount2);
  threshold1 = avecount1 - 3*sigcount1;
  threshold2 = avecount2 - 3*sigcount2;
  Serial.print("Setting new thresholds at ");Serial.print(threshold1);Serial.print(", ");Serial.println(threshold2);

  	
  // The above line is the same as "pinMode(13, OUTPUT);"

  // You might be wondering why we're not also configuring
  // sensorPin as an input. The reason is that this is an
  // "analog in" pin. These pins have the special ability to
  // read varying voltages from sensors like the potentiometer.
  // Since they're always used as inputs, there is no need to
  // specifically configure them.
}


void loop() // this function runs repeatedly after setup() finishes
{
  // First we'll declare another integer variable
  // to store the value of the potentiometer:

  static int sensorValue1;
  static int sensorValue2;
  static int oldsensorValue1;
  static int oldsensorValue2;
  const int dark = 0;
  const int light = 1;
  static int oldstate1 = 1;
  static int oldstate2 = 1;
  static int state1 = 1;
  static int state2 = 1;
  bool count1 = false;
  bool count2 = false;
  static unsigned long microsec = 0;
  float speed;
  float mics;
  const float distance = 14.92;
  unsigned long itimeout = 200000;
  const float mile = 1.60934;
  float avecount1 = 0;
  float avecount2 = 0;
  unsigned long numcount = 0;
  float sigcount1 = 0;
  float sigcount2 = 0;
  const unsigned long maxcount = 50000;

  unsigned long icount = 0;
  while(1) {
    ++icount;
    sensorValue1 = analogRead(sensorPin1);
//    Serial.print("S1:"); Serial.println(sensorValue1);
    oldstate1 = state1;
    if(sensorValue1 < threshold1) state1 = dark;
    else state1 = light;

    sensorValue2 = analogRead(sensorPin2);
    oldstate2 = state2;
    if(sensorValue2 < threshold2) state2 = dark;
    else state2 = light;
    bool t1 = state1 == dark && oldstate1 == light;
    bool t2 = state2 == dark && oldstate2 == light;
    if((!count1 && !count2) && (t1 || t2)) {
        count1 = t1;
        count2 = t2;
        microsec = micros();
        digitalWrite(13,HIGH);
        icount = 0;
    } else if((count1 && t2) || (count2 && t1)) {
//         Serial.print("D2:"); Serial.println(oldsensorValue2-sensorValue2);
         mics = micros()-microsec;
         Serial.println("===============================================================");
         Serial.print("Time (microsecs) = ");
         Serial.println(mics);    
         speed = distance*1000000./mics;
         Serial.print("Raw speed = ");
         Serial.print(speed);
         Serial.println(" cm/s ");
         speed = 3600.*speed*87./(1000.*100.);
         Serial.print("Scale speed = ");
         Serial.print(speed);
         Serial.print(" km/h / ");
         Serial.print(speed/mile);
         Serial.println(" mph");
         Serial.println("===============================================================");
         count1 = false;
         count2 = false;
         digitalWrite(13,LOW);
         delay(min(max(2*mics/1000,1000),15000));
         icount = 0;
     } else if((count1 || count2) && icount>itimeout) {
      // Adjust timeout for 1 km/h
         mics = micros()-microsec;
         speed = 3600.*87.*distance*10./mics;
         itimeout = itimeout * speed/3.;
         Serial.print("Speed lower than ");Serial.print(speed);Serial.println(" km/h");
         digitalWrite(13,LOW);
         count1 = false;
         count2 = false;
         icount = 0;
    }
    if((!count1 && !count2)) {
       sensorValue1 = analogRead(sensorPin1);
       sensorValue2 = analogRead(sensorPin2);
       avecount1 += sensorValue1;
       sigcount1 += sensorValue1*sensorValue1;
       avecount2 += sensorValue2;
       sigcount2 += sensorValue2*sensorValue2;
       ++numcount;
       if(numcount == maxcount) {
         avecount1 /=numcount;
         avecount2 /=numcount;
//       Serial.println(sigcount/numcount);
//       Serial.println(avecount);
         sigcount1 = sigcount1/numcount - avecount1*avecount1;
         if(sigcount1 <= 0) sigcount1 = avecount1/30.;
         else sigcount1 = sqrt(sigcount1);
         sigcount2 = sigcount2/numcount - avecount2*avecount2;
         if(sigcount2 <= 0) sigcount2 = avecount2/30.;
         else sigcount2 = sqrt(sigcount2);
         Serial.print("count1 ");Serial.print(avecount1);Serial.print(" +- ");Serial.println(sigcount1);
         Serial.print("count2 ");Serial.print(avecount2);Serial.print(" +- ");Serial.println(sigcount2);
         threshold1 = avecount1 - 3*sigcount1;
         threshold2 = avecount2 - 3*sigcount2;
         Serial.print("Setting new thresholds at ");Serial.print(threshold1);Serial.print(", ");Serial.println(threshold2);
         numcount = 0;
         sigcount2 = 0;
         sigcount2 = 0;
         avecount1 = 0;
         avecount2 = 0;
       }
    }
  }
} 

