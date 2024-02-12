#include "dayz221lib.h"
#include <Arduino.h>

Motor leftMotor(false);
Motor rightMotor(true);
NikiMotors motors(&leftMotor, &rightMotor, 200, 3000);
LineSensor sensor(9);

void calibrate() {
    static uint32_t time = millis();
    static int state = 0;
    uint32_t timeOfOneRotate = 0;

    while (true) {
        bool data = sensor.get();
        if (state == 0 && data) {
            state = 1;
        } else if (state == 1 && !data) {
            state = 2;
            time = millis();
        } else if (state == 2 && data) {
            state = 3;
        } else if (state == 3 && !data) {
            timeOfOneRotate = 2*(millis()-time);
            break;
        }
    }

    motors.setTimeOfOneRotate(timeOfOneRotate);
}

void setup() {
    calibrate();
}


void loop() {

}


/*

Пример калибровки поворота робота

*/