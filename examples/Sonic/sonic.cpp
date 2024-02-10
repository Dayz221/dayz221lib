#include "dayz221lib.h"
#include <Arduino.h>

Sonic sonic(2, 3);  // echo, trig

void setup() {
    Serial.begin(9600);
}

void loop() {
    Serial.println(sonic.getDist());    // получить расстояние
}