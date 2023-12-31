#include <Wire.h>

int oldAdd = 0x3C;
int newAdd = 0x3D;

void setup() {
  Wire.begin();
  
  Serial.begin(9600);
  while (!Serial)
    delay(10);
  Serial.println("\nI2C Change Address");

  Wire.beginTransmission(oldAdd);
  int error = Wire.endTransmission();
  if( error != 0)
  {
    Serial.print( "The sensor is not at ");
    Serial.println(oldAdd,HEX);
  }
  else
  {
    Serial.println( "The sensor is found, changing I2C address");
    Wire.beginTransmission(oldAdd);
    Wire.write( 0x53);  // password register
    Wire.write( 0xAA);  // password
    error = Wire.endTransmission();
    if(error) {
      Serial.println("could not set password");
    }
    
    delay(10);    // not described somewhere, just for safety
    
    Wire.beginTransmission(oldAdd);
    Wire.write( 0x00);  // I2C address register
    Wire.write( newAdd << 1); // new I2C address
//    Wire.write( newAdd ); // new I2C address
    error = Wire.endTransmission();
    if(error) {
      Serial.println("could not change address");
    }
  }
  Wire.beginTransmission(newAdd);
  error = Wire.endTransmission();
  if( error == 0)
  {
    Serial.print( "The sensor is now at 0x");
    Serial.println(newAdd,HEX);
  } else {
    Serial.println( "Address change failed");
  }
}

void loop() {
}
