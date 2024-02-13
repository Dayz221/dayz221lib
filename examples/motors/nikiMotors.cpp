#include <Arduino.h>
#include "dayz221lib.h"

#define SPEED 140


Motor left(false), right(true);
NikiMotors motors(&left, &right, SPEED);


void setup() {

    motors.move(SPEED);                             // задать скорость
    delay(500);
    motors.move(SPEED, SPEED/2);                    // задать разную скорость на левый и правый моторы
    delay(500);
    motors.moveMilliseconds(SPEED, 1000);           // двигаться со скоростью SPEED 1 секунду
    motors.moveMilliseconds(SPEED, SPEED*2, 1000);  // двигаться со скоростями двигателей 100 и 200 1 секунду
    motors.rotate(90);                              // повернуться на 90 градусов
    motors[0].setSpeed(255);                        // задать первому мотору скорость 255
    motors.setTimeOfOneRotate(2500);                // задать время на один оборот робота (для корректной работы поворота на угол)
    motors.stop();                                  // СТОП!!!!

}

// в цикле loop ничего не писать! Всю логику описать в setup
void loop() {}