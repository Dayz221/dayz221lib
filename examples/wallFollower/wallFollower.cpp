#include <Arduino.h>
#include "dayz221lib.h"


#define SPEED -100


Motor left(false), right(true);
NikiMotors motors(&left, &right, SPEED);
int pins[] = {11, 10, 3, 9};
LineSensors sensors(4, pins);
IRSensor ir(A1);
WallFollower wall(&motors, &sensors, &ir);


void setup() {
    wall.followUntilLine(SPEED, 10, -0.08, 450);    // доехать до следующей линии, ориентируясь по стенке
    delay(2000);
}

void loop() {
    wall.followWall(SPEED, 10, -0.08);  // следовать по стенке
}