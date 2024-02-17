#include <Arduino.h>
#include "dayz221lib.h"

IRSensor ir(A1);    // аналоговый пин

void setup() {
    Serial.begin(9600);
}


void loop() {
    Serial.println(ir.getDist());    // получить расстояние
}