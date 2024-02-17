#include <Arduino.h>
#include "dayz221lib.h"

Sonic sonic(13, 12);    // echo, trig (подбираются вручную)

void setup() {
    Serial.begin(9600);
}


void loop() {
    Serial.println(sonic.getDist());    // получить расстояние
}