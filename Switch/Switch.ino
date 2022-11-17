/*
  
*/

#include "Servo.h"

class Switch: public Servo {
public:
  Switch(int spin, int dccpin, int button, int straight, int curve): 
     Servo(),
     m_spin(spin),
     m_dccpin(dccpin),
     m_button(button),
     m_straight(straight),
     m_curve(curve),
     m_curpos(straight),
     m_rot(m_straight>m_curve?-1:1),
     m_state(true) {
      pinMode(m_dccpin,OUTPUT);
      pinMode(m_button,INPUT);
      pinMode(10,OUTPUT);
     }

  void Init() {
      writeMicroseconds(m_straight);
      attach(m_spin);
      digitalWrite(m_dccpin,LOW);
      digitalWrite(10,HIGH);
     }
  bool Change(bool straight) {
    short int j;
    if(straight) {
      for(j=m_curpos; j != m_straight; j=j-m_rot) {
        writeMicroseconds(j);
        delay(10);
        if(j==(m_curve+m_straight)/2) {
          digitalWrite(m_dccpin,LOW);
        }
      }
    }
    else {
      for(j=m_curpos; j != m_curve; j=j+m_rot) {
        writeMicroseconds(j);
        delay(10);
        if(j==(m_curve+m_straight)/2) {
          digitalWrite(m_dccpin,HIGH);
        }
      }
    }
    m_curpos = j;
    m_state = straight;
  }
private:
  unsigned char m_spin;   // pin of the servo
  unsigned char m_dccpin; // pin to control dcc
  unsigned char m_button; // pin for manual button
  short int m_straight;   // straight milliseconds
  short int m_curve;      // curve milliseconds
  short int m_curpos;     // current position
  char m_rot;             // rotation from straight to curve
  bool m_state;           // state 
};

Switch mySwitch(9,4,8,1600-283, 1600+283);

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  mySwitch.Init();
  delay(1000);
}

// the loop function runs over and over again forever
void loop() {
 
  static int highlow=1;
  mySwitch.Change(true);
  digitalWrite(LED_BUILTIN, highlow);    // turn the LED off by making the voltage LOW
  delay(1000);

 // read the state of the pushbutton value:
  int buttonState;
  buttonState = digitalRead(8);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(10, HIGH);
  } else {
    // turn LED off:
    digitalWrite(10, LOW);
  }

  
  highlow = 1-highlow;
  mySwitch.Change(false);
  digitalWrite(LED_BUILTIN, highlow);    // turn the LED off by making the voltage LOW
  delay(1000);                       // waits 15ms for the servo to reach the position
  highlow = 1-highlow;
}
