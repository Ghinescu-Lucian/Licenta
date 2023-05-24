#include <Servo.h>

Servo servoX;
Servo servoY;

int xPot = 0;
int yPot = 1;

void setup() {
  servoX.attach(9);
  servoY.attach(10);
}

void loop() {
  int xPos = analogRead(xPot);
  int yPos = analogRead(yPot);
  
  xPos = map(xPos, 0, 1023, 0, 180);
  yPos = map(yPos, 0, 1023, 0, 180);
  
  servoX.write(xPos);
  servoY.write(yPos);
  
  delay(15);
}
