#include <Servo.h>

Servo myservo;

void setup() {
  // put your setup code here, to run once:
  myservo.writeMicroseconds(1550);              // tell servo to go to position in variable 'pos'
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  // put your main code here, to run repeatedly:

}
