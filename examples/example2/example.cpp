#include "dayz221lib.h"
#include <Servo.h>

#define SPEED -100

Motor left(false), right(true);
NikiMotors motors(&left, &right, 100, 2000);

int pins[] = {11, 10, 3, 9};
LineSensors sensors(4, pins);
Sonic sonic(13, 12);
IRSensor ir(A1);

LineFollower solver(&motors, &sensors);
WallFollower wall(&motors, &sensors, &ir);

Servo s;

void close() {
  s.write(150);
}

void open() {
  s.write(35);
} 

void setup() {
  static int cur = 0;
  s.attach(8);
  open();
  solver.lineCalibrate(4);
  delay(100);
  solver.followMilliseconds(SPEED, 2400);
  delay(100);
  motors.rotate(90);
  delay(100);
  uint32_t timer = 0;
  for (int i = 0; i<8; i++) {
    wall.followUntilLine(SPEED, 10, -0.08, 450);
    delay(100);
    if (sonic.getDist() > 0.01 && sonic.getDist() < 25) {
      tone(A0, 1000, 200);
      solver.rotateUntilLine(SPEED);
      delay(100);
      timer = millis();
      solver.followUntilCrossroad(SPEED, 250);
      close();
      delay(100);
      motors.moveMilliseconds(-SPEED, millis()-timer-100);
      delay(100);
      motors.rotate(-90);
          
      for(int j = i; j < 7; j++) {
        wall.followUntilLine(SPEED, 10, -0.08, 450);
      }
      solver.rotateUntilLine(SPEED);
      delay(100);
      solver.followUntilCrossroad(SPEED, 450);
      delay(100);
      solver.followMilliseconds(SPEED, 2700);
      delay(100);
      motors.rotate(90);
      delay(100);
      for (int j = 0; j < 7-i; j++) {
        wall.followUntilLine(SPEED, 10, -0.08, 450);
      }
      delay(100);
      solver.rotateUntilLine(SPEED);
      delay(100);
      timer = millis();
      solver.followUntilCrossroad(SPEED, 450);
      open();
      delay(100);
      motors.moveMilliseconds(-SPEED, millis()-timer-100);
      delay(100);
      motors.rotate(-90);
      for (int j = 0; j < i+1; j++) {
        wall.followUntilLine(SPEED, 10, -0.08, 450);
      }
      delay(100);
      solver.rotateUntilLine(SPEED);
      delay(100);
      solver.followUntilCrossroad(SPEED, 450);
      solver.followMilliseconds(SPEED, 2700);
      delay(100);
      motors.rotate(90);
      delay(100);
      for (int j = 0; j < i+1; j++) {
        wall.followUntilLine(SPEED, 10, -0.08, 450);
      }
    }
  }
  solver.rotateUntilLine(SPEED);
  delay(100);
  solver.followUntilCrossroad(SPEED, 450);
  
  END();
}

void loop() {

}