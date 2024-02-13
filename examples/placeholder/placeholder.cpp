#include <Arduino.h>
#include "dayz221lib.h"

#define SPEED 140


Motor left(false), right(true);
NikiMotors motors(&left, &right, SPEED);

int pins[] = {9, 3, 10, 11};
LineSensors sensors(4, pins);

LineFollower robot(&motors, &sensors);
Sonic sonic(13, 12);


void setup() {
// алгоритм робота
}

// в цикле loop ничего не писать! Всю логику описать в setup
void loop() {}