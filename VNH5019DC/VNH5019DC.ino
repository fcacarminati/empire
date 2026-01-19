#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  md.init();
}

void loop()
{
  int ipot = analogRead(A2);
  int speed = map(ipot,0,1023,-400,400);
  if (abs(speed) < 20) speed = 0;

  md.setM1Speed(speed);
  stopIfFault();

  // print current about 10x/sec
  static unsigned long t0 = 0;
  if (millis() - t0 > 100) {
    t0 = millis();
    Serial.print("pot=");
    Serial.print(ipot);
    Serial.print(" speed=");
    Serial.print(speed);
    Serial.print(" mA=");
    Serial.println(md.getM1CurrentMilliamps());
  }
  return;

  Serial.println("Loop 1");
  for (int i = 0; i <= 400; i++)
  {
    md.setM1Speed(i);
    stopIfFault();
    if (i%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
    }
    delay(2);
  }
  
  Serial.println("Loop 2");
  for (int i = 400; i >= -400; i--)
  {
    md.setM1Speed(i);
    stopIfFault();
    if (i%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
    }
    delay(2);
  }
  
  Serial.println("Loop 3");
  for (int i = -400; i <= 0; i++)
  {
    md.setM1Speed(i);
    stopIfFault();
    if (i%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
    }
    delay(2);
  }

}
