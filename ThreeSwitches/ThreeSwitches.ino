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
   D11 => Sensors side bridge
   D12 => Sensors side church
   D13 => Control lights tunnel
   A0  => Button lights tunnel
*/

#include "Switch.h"


/*
 * 
 */

Switch Switch1(1,2,3,1380+200,1380-200);
Switch Switch2(4,5,6,1380+200,1380-200);
Switch Switch3(7,8,9,1380+200,1380-200);

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
