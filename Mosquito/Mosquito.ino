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
  Serial.println("Loop M1-1");
  for (int i = 0; i <= 400; i++)
  {
    md.setM1Speed(i);
    stopIfFault();
    if (abs(i)%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
    }
    delay(2);
  }
  
  Serial.println("Loop M1-2");
  for (int i = 400; i >= -400; i--)
  {
    md.setM1Speed(i);
    stopIfFault();
    if (abs(i)%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
    }
    delay(2);
  }

  Serial.println("Loop M1-3");
  for (int i = -400; i <= 0; i++)
  {
    md.setM1Speed(i);
    stopIfFault();
    if (abs(i)%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
    }
    delay(2);
  }

  Serial.println("Loop M2-1");
  for (int i = 0; i <= 400; i++)
  {
    md.setM2Speed(i);
    stopIfFault();
    if (abs(i)%200 == 100)
    {
      Serial.print("M2 current: ");
      Serial.println(md.getM2CurrentMilliamps());
    }
    delay(2);
  }
  
  Serial.println("Loop M2-2");
  for (int i = 400; i >= -400; i--)
  {
    md.setM2Speed(i);
    stopIfFault();
    if (abs(i)%200 == 100)
    {
      Serial.print("M2 current: ");
      Serial.println(md.getM2CurrentMilliamps());
    }
    delay(2);
  }
  
  Serial.println("Loop M2-3");
  for (int i = -400; i <= 0; i++)
  {
    md.setM2Speed(i);
    stopIfFault();
    if (abs(i)%200 == 100)
    {
      Serial.print("M2 current: ");
      Serial.println(md.getM2CurrentMilliamps());
    }
    delay(2);
  }
}
