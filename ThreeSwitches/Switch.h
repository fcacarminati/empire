/* 
 *  Switch class based derived from the Servo library class
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
