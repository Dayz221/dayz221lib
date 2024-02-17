#include "dayz221lib.h"

#define SPEED -100

Motor left(false), right(true);
NikiMotors motors(&left, &right, 100, 2000);

int A0s[] = {11, 10, 3, 9};
LineSensors sensors(4, A0s);
Sonic sonic(13, 12);
IRSensor ir(A1);

LineFollower solver(&motors, &sensors);
WallFollower wall(&motors, &sensors, &ir);

bool boxes[7] = {0, 0, 0, 0, 0, 0, 0};

void setup() {
  solver.lineCalibrate(4);
  delay(100);
  solver.followMilliseconds(SPEED, 2300);
  delay(100);
  motors.rotate(90);
  delay(100);
  for (int i = 0; i<8; i++) {
    wall.followUntilLine(SPEED, 10, -0.08, 450);
    delay(100);
    if (sonic.getDist() > 0.01 && sonic.getDist() < 25) {
      tone(A0, 1000, 200);
      boxes[6-i] = true;
    }
  }
  solver.rotateUntilLine(SPEED);
  delay(100);
  solver.followUntilCrossroad(SPEED, 450);
  delay(100);
  solver.followMilliseconds(SPEED, 2500);
  delay(100);
  motors.rotate(90);
  delay(100);
  for (int i = 0; i<8; i++) {
    wall.followUntilLine(SPEED, 10, -0.08, 450);
    delay(100);
    if (boxes[i]) {
      tone(A0, 1000, 200);
      solver.rotateUntilLine(SPEED);
      delay(100);
      uint32_t timer = millis();
      solver.followUntilCrossroad(SPEED);
      delay(100);
      motors.moveMilliseconds(-SPEED, millis()-timer);
      delay(100);
      motors.rotate(-90);
      delay(100);
    }
  }
  solver.rotateUntilLine(SPEED);
  delay(100);
  solver.followUntilCrossroad(SPEED);
  END();
}

void loop() {

}