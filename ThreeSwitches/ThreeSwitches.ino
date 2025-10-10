/*
   This is the code for the Bismark Tunnel. 
   There are three switches two sensors and the illumination of the tunnel
   Here are the assignments:
   D2  => Digital control servo 1
   D3  => DCC relay servo 1
   D4  => Button servo 1
   D5  => Digital control servo 2
   D6  => Digital relay servo 2
   D7  => Button servo 2
   D8  => Digital control servo 3
   D9  => Digital relay servo 3
   D10  => Button servo 3 
*/

#include "Switch.h"


/*
 * 
 */
int half1 = 1200;
int right1 = -180;
int curve1 = -200;
Switch Switch1(2,3,4,half1+right1,half1-curve1);
int half2 = 1600;
int right2 = 65;
int curve2 = 100;
Switch Switch2(5,6,7,half2+right2,half2-curve2);
int half3 = 1450;
int right3 = -80;//-100;
int curve3 = -95;//-80;
Switch Switch3(8,9,10,half3+right3,half3-curve3);

// The setup function runs once when you press reset or power the board
void setup() {
  Switch1.Init();
  Switch2.Init();
  Switch3.Init();
  delay(1000);
  Serial.begin(115200);
}

// The loop function runs over and over again forever
void loop() {
 
  Switch1.Change(Switch1.readButton() == HIGH);
  Switch2.Change(Switch2.readButton() == HIGH);
  Switch3.Change(Switch3.readButton() == HIGH);

}
