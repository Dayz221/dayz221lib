#include <Arduino.h>
#include "dayz221lib.h"

#define SPEED 140


Motor left(false), right(true);
NikiMotors motors(&left, &right, SPEED);

int pins[] = {9, 3, 10, 11};
LineSensors sensors(4, pins);

LineFollower robot(&motors, &sensors);


void setup() {
    robot.lineCalibrate(4);             // откалибровать поворот робота на перекрестке с 4 дорогами (в начале)
    delay(100);                         // задержка для стабильности
    robot.followUntilCrossroad(SPEED);  // следовать по линии до первого перекрестка
    delay(100);
    robot.followUntilLineEnd(SPEED);    // следовать до обрыва линии
    delay(100);
    while (true) {
        robot.follow(SPEED, 0.8);       // функция следования по линии, обязательно использовать в цикле
    }

    // дописать функции поворота к первой линии и поворота до N линии
}

// в цикле loop ничего не писать! Всю логику описать в setup
void loop() {}