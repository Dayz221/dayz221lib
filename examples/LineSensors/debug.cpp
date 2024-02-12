#include "dayz221lib.h"
#include <Arduino.h>

LineSensors sensors(9, 10);

void setup() {
    Serial.begin(9600);
}

Timer timer;
void loop() {
    if (timer.get(50)) {
        sensors.debug();
    }
}