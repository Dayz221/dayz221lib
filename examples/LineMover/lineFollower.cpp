#include <Arduino.h>
#include "dayz221lib.h"

Motor left(0);
Motor right(1);
NikiMotors motors(&left, &right);
LineSensors sensors(9, 10);
LineFollower follower(&motors, &sensors);

void setup() {}

Timer timer;
void loop() {
    if (timer.get(50)) {
        follower.follow(100);
    }
}