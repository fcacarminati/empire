/*
   Switch test
*/

#include "Servo.h"

/* 
 *  Switch class based derived from the Servo library class
 */
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
     }

  void Init() {
      writeMicroseconds(m_straight);
      attach(m_spin);
      digitalWrite(m_dccpin,LOW);
      pinMode(m_button,INPUT_PULLUP);
     }

  int readButton() const {
    /* Read the button */
    return digitalRead(m_button);
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
  unsigned char m_dccpin; // pin to control dcc relay
  unsigned char m_button; // pin for manual button
  short int m_straight;   // straight milliseconds
  short int m_curve;      // curve milliseconds
  short int m_curpos;     // current position
  char m_rot;             // rotation from straight to curve
  bool m_state;           // state 
};

Switch mySwitch(9,4,8,1460-300, 1460+300);

// The setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(10,OUTPUT);
  digitalWrite(10,HIGH);
  mySwitch.Init();
  delay(1000);
}

// The loop function runs over and over again forever
void loop() {
 
  static int straight = true;
  static int ledhighlow = 1;
  
  int buttonState;
  buttonState = mySwitch.readButton();

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == LOW) {
    // turn LED on:
    straight = !straight;
    mySwitch.Change(straight);
    ledhighlow = 1-ledhighlow;
    digitalWrite(10, ledhighlow);
    delay(500);
  }
}
