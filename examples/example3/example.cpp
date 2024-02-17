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

bool etalon[7] = {0, 0, 0, 0, 0, 0, 0};
bool boxes[7] = {0, 0, 0, 0, 0, 0, 0};
uint32_t time = 0;

int getA() {
  for (int i = 0; i < 7; i++) {
    if (etalon[i] && !boxes[i]) {
      return i+1;
    }
  }
  return -1;
} 

int getB() {
  for (int i = 0; i < 7; i++) {
    if (!etalon[i] && boxes[i]) {
      return i+1;
    }
  }
  return -1;
} 

void goTo(int &cur, int next) {
  int delta = next - cur;
  if (delta < 0) delta += 8;
  for (int i = 0; i < delta; i++) {
    wall.followUntilLine(SPEED, 10, -0.08, 450);
  }
  cur = next;
}

int curPos = 0;

void setup() {
  open();
  s.attach(8);
  // получение данных о кубиках //
  solver.lineCalibrate(4);
  solver.followMilliseconds(SPEED, 2300);
  motors.rotate(90);
  for (int i = 0; i < 7; i++) {
    wall.followUntilLine(SPEED, 10, -0.08, 450);
    float d = sonic.getDist();
    if (d > 0.01 && d < 25) {
      etalon[6-i] = true;
      tone(A0, 1000, 100);
    }
  }
  wall.followUntilLine(SPEED, 10, -0.08, 450);
  solver.rotateUntilLine(SPEED);
  solver.followUntilCrossroad(SPEED, 450);
  solver.followMilliseconds(SPEED, 2550);
  motors.rotate(90);
  // ========================== //
  // просканировать 2 часть поля //
  for (int i = 0; i < 7; i++) {
    wall.followUntilLine(SPEED, 10, -0.08, 450);
    float d = sonic.getDist();
    if (d > 0.01 && d < 25) {
      boxes[i] = true;
      tone(A0, 1000, 100);
    } 
  }
  wall.followUntilLine(SPEED, 10, -0.08, 450);
  // ========================== //
  int cnt = 0;
  for (int i = 0; i < 7; i++) {
    if (etalon[i] && !boxes[i]) {
      cnt++;
    }
  }

  for (int i = 0; i < cnt; i++) {
    goTo(curPos, getB());
    solver.rotateUntilLine(SPEED);
    time = solver.followUntilCrossroad(SPEED, 150);
    close();
    boxes[curPos-1] = false;
    motors.moveMilliseconds(-SPEED, time);
    motors.rotate(-90);
    goTo(curPos, getA());
    solver.rotateUntilLine(SPEED);
    time = solver.followUntilCrossroad(SPEED, 50);
    open();
    boxes[curPos-1] = true;
    motors.moveMilliseconds(-SPEED, time);
    motors.rotate(-90);
  }
  goTo(curPos, getB());
  solver.rotateUntilLine(SPEED);
  time = solver.followUntilCrossroad(SPEED, 150);
  close();
  boxes[curPos-1] = false;
  motors.moveMilliseconds(-SPEED, time);
  motors.rotate(-90);
  goTo(curPos, 0);
  solver.rotateUntilLine(SPEED);
  solver.followUntilCrossroad(SPEED, 450);
}

void loop() {

}