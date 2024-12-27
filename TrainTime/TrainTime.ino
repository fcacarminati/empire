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


void setup() // this function runs once when the sketch starts up
{
  // We'll be using pin 13 to light a LED, so we must configure it
  // as an output.
 
  // Because we already created a variable called ledPin, and
  // set it equal to 13, we can use "ledPin" in place of "13".
  // This makes the sketch easier to follow.
  
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
	
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
  const int threshold = 150;
  static int oldstate1 = 1;
  static int oldstate2 = 1;
  static int state1 = 1;
  static int state2 = 1;
  bool count = false;
  static unsigned long microsec = 0;
  float speed;
  float mics;

  oldsensorValue1 = sensorValue1;  
  sensorValue1 = analogRead(sensorPin1);
  Serial.print("S1:"); Serial.println(sensorValue1);
  oldstate1 = state1;
  if(sensorValue1 < threshold) state1 = dark;
  else state1 = light;

  if(state1 == dark && oldstate1 == light && state2 == light) {
     microsec = micros();
     count = true;
     digitalWrite(13,HIGH);
     Serial.print("D1:"); Serial.println(oldsensorValue1-sensorValue1);
  }
  
  oldsensorValue2 = sensorValue2;
  sensorValue2 = analogRead(sensorPin2);
  oldstate2 = state2;
  if(sensorValue2 < threshold) state2 = dark;
  else state2 = light;

  long int icount = 0;
  while(state2==light && count && ++icount < 100000) {
    oldsensorValue2 = sensorValue2;
    sensorValue2 = analogRead(sensorPin2);
    Serial.print("S2:"); Serial.println(sensorValue2);

//     if((icount+1)%1000 == 1) Serial.println(icount);    
    if(icount == 99999) Serial.println("Timeout!");    
         
    oldstate2 = state2;
    if(sensorValue2 < threshold) state2 = dark;
    else state2 = light;
   }

  if(state2 == dark && oldstate2 == light && count) {
    Serial.print("D2:"); Serial.println(oldsensorValue2-sensorValue2);
    mics = micros()-microsec;
    Serial.print("Time (microsecs) = ");
    Serial.println(mics);    
    speed = 15.*1000000./mics;
    Serial.print("Raw speed (cm/sec) = ");
    Serial.println(speed);
    speed = 3600.*speed*87./(1000.*100.);
    Serial.print("Scale speed (km/h)= ");
    Serial.println(speed);
  }
  count = false;
  digitalWrite(13,LOW);


  
  // Now we'll blink the LED like in the first example, but we'll
  // use the sensorValue variable to change the blink speed
  // (the smaller the number, the faster it will blink).

  // Note that we're using the ledPin variable here as well:

/*   
   Serial.print("Value 1 ");                               
   Serial.println(sensorValue1);                               
   Serial.print("Value 2 ");                               
   Serial.println(sensorValue2);                               
*/
  
  // Remember that loop() repeats forever, so we'll do all this
  // again and again.
}

